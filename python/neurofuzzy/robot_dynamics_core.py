"""
=======================================================
SYNERGETIC CONTROL SIMULATION - DELTA ROBOT 3-DOF
Version 5: With Warm-up Start (smooth initial transient)
=======================================================
Goal: generate (e, de) -> mu* dataset for Neuro-Fuzzy training.

Improvements vs v4:
    - Added WARM_UP_SEC period at the start where q_d eases in
      from 0 to the first trajectory point via smoothstep curve
    - This prevents huge initial torque spikes that corrupt
      the early portion of the training dataset
    - Warm-up samples are EXCLUDED from final dataset & statistics
      (only the steady-state portion is used for NF training)
    - All Indonesian comments translated to English

Control law (PDF Eq. 10):
    tau = M(q) * [ddq_d + c*e_dot + sigma/mu] + C(q,dq)*dq + tau_g

Plant model (with friction, real physics):
    M(q)*ddq + C(q,dq)*dq + tau_g + b*dq = tau

Statistics shown in plot title AND on-figure text box:
    RMSE, MAE, StdDev per joint + overall average
=======================================================
"""

import numpy as np
import math
from scipy.interpolate import interp1d
from scipy.integrate import solve_ivp
import pandas as pd
import matplotlib.pyplot as plt
import time

# =======================================================
# 1. PARAMETER CONFIG (Physical only - no motor electrical)
# =======================================================
# --- Synergetic ---
MU_MIN = 0.001
MU_MAX = 0.006
MU_STEPS = 100
C_MANIFOLD = 80.0
B_FRICTION = 0.01859          # Plant friction (real physics)

# --- Delta Robot Physical (meters) ---
GRAVITY = 9.81
L_A = 0.175               # Upper arm length
L_B = 0.25                # Lower arm length
M_PT = 0.317              # Moving plate mass
I_A_VAL = 0.0443          # Motor + upper arm inertia
I_A = np.diag([I_A_VAL, I_A_VAL, I_A_VAL])
M_A = 0.046               # Upper arm mass
L_C = 0.08                # Arm center of mass

# --- Geometry ---
ED_M = 0.09               # End-effector radius
F_M = 0.07                # Base radius
R_DIST = F_M - ED_M

# --- Motor electrical (current loop) ---
# Torque is NO LONGER instantaneous: tau = Kt * I, current driven by
# PID current controller through motor R-L dynamics. Makes large mu cost.
KT = 1.0
KE = 1.0
R_MOTOR = 3.4
L_MOTOR = 0.00018         # winding inductance (H)
V_SUPPLY = 12.0
PWM_MAX = 2048.0

# Current PID gains (per joint) - from old per-joint sim
KP_I = 60.0
KI_I = 5.0
KD_I = 0.015

# --- Internal Simulation ---
DT_INTERNAL = 0.001       # Internal ODE step
ODE_METHOD = 'rk4'        # 'euler' or 'rk4'

# =======================================================
# 2. USER CONFIG - CUSTOMIZABLE
# =======================================================
N_DATA = 500              # Number of logged samples (FINAL dataset size)
DT_LOG = 0.004            # Logging interval (s)

# --- Warm-up settings ---
WARM_UP_SEC = 1.0         # Seconds of smooth ease-in at the start
                          # Set to 0.0 to disable warm-up
INCLUDE_WARMUP_IN_CSV = False  # If True, warm-up samples are kept in CSV
                               # (usually False for clean NF training data)

# Per-joint trajectory configuration
JOINT_CFG = {
    1: {'max_deg': 85.0, 'n_peaks': 20, 'seed': 42},
    2: {'max_deg': 85.0, 'n_peaks': 20, 'seed': 123},
    3: {'max_deg': 85.0, 'n_peaks': 20, 'seed': 999},
}

# =======================================================
# 3. AUTO-COMPUTE T_TOTAL (now includes warm-up)
# =======================================================
N_WARMUP = int(round(WARM_UP_SEC / DT_LOG))   # warm-up samples
N_TOTAL = N_DATA + N_WARMUP                   # full simulation length
T_TOTAL = N_TOTAL * DT_LOG
T_MAIN = N_DATA * DT_LOG                      # main (post-warm-up) duration

N_INTERNAL_PER_LOG = max(1, int(round(DT_LOG / DT_INTERNAL)))
DT_SUB = DT_LOG / N_INTERNAL_PER_LOG

print("=" * 64)
print("SIMULATION CONFIGURATION (v5 - with warm-up)")
print("=" * 64)
print(f"N_DATA (logged samples)   : {N_DATA}")
print(f"WARM_UP_SEC               : {WARM_UP_SEC} s ({N_WARMUP} samples)")
print(f"N_TOTAL (with warm-up)    : {N_TOTAL}")
print(f"DT_LOG (logging interval) : {DT_LOG} s")
print(f"T_TOTAL (auto)            : {T_TOTAL:.4f} s")
print(f"T_MAIN (after warm-up)    : {T_MAIN:.4f} s")
print(f"DT_INTERNAL (ODE step)    : {DT_INTERNAL} s")
print(f"Internal sub-steps per log: {N_INTERNAL_PER_LOG}")
print(f"DT_SUB (actual)           : {DT_SUB} s")
print(f"ODE Method                : {ODE_METHOD.upper()}")
print(f"MU candidates             : {MU_STEPS} ({MU_MIN} - {MU_MAX})")
print(f"State dim                 : 6 (no electrical)")
print(f"Warm-up in CSV            : {INCLUDE_WARMUP_IN_CSV}")
print("-" * 64)
for j, cfg in JOINT_CFG.items():
    print(f"Joint {j}: max={cfg['max_deg']} deg | "
          f"peaks={cfg['n_peaks']} | seed={cfg['seed']}")
print("=" * 64)

# =======================================================
# 4. GEOMETRY SETUP 3-DOF (cached constants)
# =======================================================
TAN30 = 1.0 / math.sqrt(3)
TAN60 = math.sqrt(3)
SIN30 = 0.5
PHI_ANGLES = np.radians([0.0, 120.0, 240.0])

R_Z_CACHE = []
for phi in PHI_ANGLES:
    cp, sp = np.cos(phi), np.sin(phi)
    R_Z_CACHE.append(np.array([[cp, -sp, 0.0],
                               [sp,  cp, 0.0],
                               [0.0, 0.0, 1.0]]))

# =======================================================
# 5. GENERATE PER-JOINT TRAJECTORIES (main portion only)
# =======================================================
print("\n[5] Generating per-joint trajectories ...")
t_log_main = np.linspace(0.0, T_MAIN, N_DATA)
q_d_list_main = []

for j in range(1, 4):
    cfg = JOINT_CFG[j]
    rng = np.random.default_rng(cfg['seed'])
    n_peaks = cfg['n_peaks']
    max_deg = cfg['max_deg']

    t_ctrl = np.linspace(0.0, T_MAIN, n_peaks)
    val_ctrl = rng.uniform(-1.0, 1.0, n_peaks)

    interp_fn = interp1d(t_ctrl, val_ctrl, kind='cubic')
    smooth = interp_fn(t_log_main)
    smooth = smooth / np.max(np.abs(smooth))
    q_d_joint = np.radians(smooth * max_deg)
    q_d_list_main.append(q_d_joint)
    print(f"   Joint {j}: range [{np.degrees(q_d_joint.min()):+.2f}, "
          f"{np.degrees(q_d_joint.max()):+.2f}] deg")

q_d_main = np.column_stack(q_d_list_main)   # shape (N_DATA, 3)

# =======================================================
# 5b. WARM-UP TRAJECTORY: smooth ease-in from 0 to q_d_main[0]
# =======================================================
def smoothstep(t):
    """Smoothstep curve (0 -> 1) with zero derivatives at endpoints.
       3t^2 - 2t^3 in [0, 1]."""
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)

if N_WARMUP > 0:
    q_d_first = q_d_main[0]                       # target at end of warm-up
    s_curve = smoothstep(np.linspace(0.0, 1.0, N_WARMUP, endpoint=False))
    # shape (N_WARMUP, 3): each column eased from 0 to q_d_first[j]
    q_d_warmup = np.outer(s_curve, q_d_first)
    q_d = np.vstack([q_d_warmup, q_d_main])
else:
    q_d = q_d_main

t_log = np.arange(N_TOTAL) * DT_LOG

# =======================================================
# 6. COMPUTE DERIVATIVES (whole trajectory including warm-up)
# =======================================================
dq_d = np.gradient(q_d, DT_LOG, axis=0)
ddq_d = np.gradient(dq_d, DT_LOG, axis=0)
print(f"   q_d full matrix shape : {q_d.shape}")


# =======================================================
# FORWARD KINEMATICS
# =======================================================
def forward_kinematics(q):
    t_val = (F_M - ED_M) * TAN30 / 2.0
    a1, a2, a3 = q[0], q[1], q[2]

    y1 = -(t_val + L_A * math.cos(a1))
    z1 = -L_A * math.sin(a1)

    y2 = (t_val + L_A * math.cos(a2)) * SIN30
    x2 = y2 * TAN60
    z2 = -L_A * math.sin(a2)

    y3 = (t_val + L_A * math.cos(a3)) * SIN30
    x3 = -y3 * TAN60
    z3 = -L_A * math.sin(a3)

    dnm = (y2 - y1) * x3 - (y3 - y1) * x2
    if abs(dnm) < 1e-9:
        return np.array([0.0, 0.0, -0.3])

    w1 = y1 * y1 + z1 * z1
    w2 = x2 * x2 + y2 * y2 + z2 * z2
    w3 = x3 * x3 + y3 * y3 + z3 * z3

    a1c = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1c = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0
    a2c = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2c = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

    A = a1c * a1c + a2c * a2c + dnm * dnm
    B = 2.0 * (a1c * b1c + a2c * (b2c - y1 * dnm) - z1 * dnm * dnm)
    C = ((b2c - y1 * dnm) ** 2 + b1c * b1c
         + dnm * dnm * (z1 * z1 - L_B * L_B))

    disc = B * B - 4.0 * A * C
    if disc < 0.0:
        return np.array([0.0, 0.0, -0.3])

    z = -0.5 * (B + math.sqrt(disc)) / A
    x = (a1c * z + b1c) / dnm
    y = (a2c * z + b2c) / dnm
    return np.array([x, y, z])


# =======================================================
# JACOBIAN (with cache for dJ/dt)
# =======================================================
_JAC_CACHE = {'J_prev': np.eye(3), 't_prev': 0.0}

def compute_jacobian(t_now, q):
    pos = forward_kinematics(q)
    S_mat = np.zeros((3, 3))
    S_b_diag = np.zeros((3, 3))

    for i in range(3):
        R_zi = R_Z_CACHE[i]
        v_b = np.array([L_A * np.sin(q[i]), 0.0, L_A * np.cos(q[i])])
        b_i = R_zi @ v_b
        v_s_off = (np.array([R_DIST, 0.0, 0.0])
                   + np.array([L_A * np.cos(q[i]), 0.0, -L_A * np.sin(q[i])]))
        s_i = pos - (R_zi @ v_s_off)
        S_mat[i, :] = s_i
        S_b_diag[i, i] = np.dot(s_i, b_i)

    try:
        J_mat = -np.linalg.solve(S_mat, S_b_diag)
    except np.linalg.LinAlgError:
        J_mat = -np.linalg.pinv(S_mat) @ S_b_diag

    dt_j = t_now - _JAC_CACHE['t_prev']
    if dt_j > 1e-9:
        dJ_mat = (J_mat - _JAC_CACHE['J_prev']) / dt_j
    else:
        dJ_mat = np.zeros((3, 3))

    _JAC_CACHE['J_prev'] = J_mat
    _JAC_CACHE['t_prev'] = t_now
    return J_mat, dJ_mat


# =======================================================
# DYNAMICS MATRICES (no friction in controller)
# =======================================================
def compute_dynamics(q, t_now):
    J_mat, dJ_mat = compute_jacobian(t_now, q)
    M_hat = I_A + M_PT * (J_mat.T @ J_mat)
    C_hat = M_PT * (J_mat.T @ dJ_mat)

    tau_g_endeff = -J_mat.T @ (M_PT * np.array([0.0, 0.0, -GRAVITY]))
    tau_g_arm = -(M_A * L_C * GRAVITY * np.cos(q))
    tau_g_hat = tau_g_endeff + tau_g_arm

    try:
        M_inv = np.linalg.inv(M_hat)
    except np.linalg.LinAlgError:
        M_inv = np.linalg.pinv(M_hat)
    return M_hat, M_inv, C_hat, tau_g_hat


# =======================================================
# DISTURBANCE (white noise)
# =======================================================
tau_dist = np.random.uniform(-5.0, 5.0, 3)


# =======================================================
# SYNERGETIC CONTROL LAW (PDF Eq. 10) - NO friction
# =======================================================
def synergetic_control_law(q, dq, q_d_t, dq_d_t, ddq_d_t,
                           mu, M_hat, C_hat, tau_g_hat):
    """tau = M*(ddq_d + c*e_dot + sigma/mu) + C*dq + tau_g"""
    e = q_d_t - q
    e_dot = dq_d_t - dq
    sigma = C_MANIFOLD * e + e_dot
    acc_command = ddq_d_t + (C_MANIFOLD * e_dot) + (sigma / mu)
    tau = (M_hat @ acc_command) + (C_hat @ dq) + tau_g_hat + tau_dist
    return tau


# =======================================================
# STATE DERIVATIVE (12-dim) + solve_ivp integration
#   state = [q(3), dq(3), I(3), int_e(3)]
#   Full delta dynamics (M_hat, C_hat, tau_g) + per-joint current PID.
#   Integrated with solve_ivp (RK45 adaptive) -> handles stiff R-L.
# =======================================================
PREV_I_ERR = np.zeros(3)
CUR_INT = np.zeros(3)


def reset_pid():
    """Reset PID arus state di awal tiap rollout kandidat mu."""
    global PREV_I_ERR, CUR_INT
    PREV_I_ERR = np.zeros(3)
    CUR_INT = np.zeros(3)


# langkah diskret PID = 1 kHz (Arduino pidIntervalMicros=1000us)
DT_PID = 0.001
N_ELEC = 10                       # sub-step elektrik per langkah PID (L stiff)
DT_ELEC = DT_PID / N_ELEC


def simulate_until_next_log(state, q_d_t, dq_d_t, ddq_d_t, mu, t_start):
    """Integrasi satu interval log dengan langkah tetap 1 kHz, persis Arduino.
    PID arus dihitung sekali per DT_PID; arus & mekanik diintegrasi Euler.
    state 12-dim [q, dq, I, int_e_unused]. Returns (next_state, tau_applied)."""
    global PREV_I_ERR, CUR_INT
    s = state.copy()
    n_pid = max(1, int(round(DT_LOG / DT_PID)))
    tau_applied = np.zeros(3)
    t_cur = t_start

    for _ in range(n_pid):
        q = s[0:3]; dq = s[3:6]; I = s[6:9]
        M_hat, M_inv, C_hat, tau_g_hat = compute_dynamics(q, t_cur)

        tau_des = synergetic_control_law(q, dq, q_d_t, dq_d_t, ddq_d_t,
                                         mu, M_hat, C_hat, tau_g_hat)
        i_target = tau_des / KT

        # PID arus persis Arduino (sekali per DT_PID)
        current_error = i_target - I
        CUR_INT = np.clip(CUR_INT + KI_I * current_error * DT_PID,
                          -1500.0, 1500.0)
        curD = KD_I * (current_error - PREV_I_ERR) / DT_PID
        PREV_I_ERR = current_error.copy()
        pwm = KP_I * current_error + CUR_INT + curD
        pwm = np.clip(pwm, -PWM_MAX, PWM_MAX)
        v_cmd = (pwm / PWM_MAX) * V_SUPPLY

        # integrasi arus (L stiff) dengan sub-step elektrik, V tetap
        for _e in range(N_ELEC):
            dI = (v_cmd - R_MOTOR * I - KE * dq) / L_MOTOR
            I = I + dI * DT_ELEC

        # integrasi mekanik (Euler, langkah DT_PID)
        tau_applied = KT * I
        tau_friction = B_FRICTION * dq
        ddq = M_inv @ (tau_applied - (C_hat @ dq) - tau_g_hat - tau_friction)
        q = q + dq * DT_PID
        dq = dq + ddq * DT_PID

        s[0:3] = q; s[3:6] = dq; s[6:9] = I
        t_cur += DT_PID

    return s, tau_applied


# =======================================================
# 7. INIT STATE & HISTORY (sized for FULL run including warm-up)
# =======================================================
state = np.zeros(12)              # [q(3), dq(3), I(3), int_e(3)]
prev_tau_vec = np.zeros(3)

history_q = np.zeros((N_TOTAL, 3))
history_qd = np.zeros((N_TOTAL, 3))
history_dq = np.zeros((N_TOTAL, 3))
history_dqd = np.zeros((N_TOTAL, 3))
history_tau = np.zeros((N_TOTAL, 3))
history_mu = np.zeros(N_TOTAL)
history_t = np.zeros(N_TOTAL)

candidate_mu = np.linspace(MU_MIN, MU_MAX, MU_STEPS)

# =======================================================
