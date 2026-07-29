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
from pathlib import Path
from scipy.interpolate import interp1d
import pandas as pd
import matplotlib.pyplot as plt
import time

_python_dir = Path(__file__).resolve().parent
while _python_dir.name != "python":
    _python_dir = _python_dir.parent
_root_dir = _python_dir.parent
_OUT_DIR = _root_dir / "results" / "training_convergence" / "synergetic_delta"
_OUT_DIR.mkdir(parents=True, exist_ok=True)

# =======================================================
# 1. PARAMETER CONFIG (Physical only - no motor electrical)
# =======================================================
# --- Synergetic ---
MU_MIN = 0.001
MU_MAX = 0.006
MU_STEPS = 100

# Multi-step horizon untuk pemilihan mu: tiap kandidat mu dinilai dari
# total error sepanjang HORIZON langkah ke depan (mu ditahan konstan),
# bukan cuma 1 langkah. Ini bikin label mu* lebih deterministik/mulus
# karena efek mu pada konvergensi terasa beberapa langkah, bukan seketika.
# HORIZON=1 -> perilaku lama (greedy 1-langkah).
HORIZON = 10

# =======================================================
# MULTI-OBJECTIVE COST (tracking + chattering)
# =======================================================
# cost = error_norm + LAMBDA_CHATTER * chatter_norm
#
# Tanpa penalti chattering, mu_min selalu menang -> dataset tak ada
# variasi mu yg bisa dipelajari NF. Penalti chattering bikin trade-off
# nyata: mu kecil -> tracking bagus tapi torsi kasar; mu besar -> torsi
# halus tapi tracking longgar. mu optimal di tengah.
#
# LAMBDA_CHATTER: bobot relatif chattering vs tracking.
#   0.00 -> seperti dulu, mu_min menang
#   0.20 -> seimbang (default, mu optimal interior)
#   0.50 -> prioritas halus, mu cenderung besar
# Ubah sesuai selera. Kalau hasil mu masih flat di MIN, naikkan.
# Kalau mu loncat ke MAX terus, turunkan.
LAMBDA_CHATTER = 0.05

# Skala normalisasi (biar bobot lambda bermakna terlepas dari satuan).
# Tetapkan dari "tipikal" tracking error & dtau yg kita anggap normal.
# Tak perlu super presisi, cukup orde besaran benar.
TRACK_NORM = 0.5      # deg: error 0.5 deg = "1 satuan"
CHATTER_NORM = 5.0    # Nm:  |dtau| 1.0 Nm = "1 satuan"

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
    1: {'max_deg': 85.0, 'n_peaks': 35, 'seed': 42},
    2: {'max_deg': 85.0, 'n_peaks': 35, 'seed': 123},
    3: {'max_deg': 85.0, 'n_peaks': 35, 'seed': 999},
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
print("SIMULATION CONFIGURATION (v7 - with warm-up)")
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
print("\n[5] Generating trajectory (cartesian XYZ random -> IK) ...")
t_log_main = np.linspace(0.0, T_MAIN, N_DATA)

# Workspace bounds (mm) meniru ruang kerja deploy helix
TRAJ_SEED = 42
TRAJ_N_PEAKS = 20
WS_X = (-60.0, 60.0)
WS_Y = (-60.0, 60.0)
WS_Z = (-340.0, -190.0)
IK_ED, IK_F, IK_RE, IK_RF = 90.0, 70.0, 250.0, 175.0


def _ik_delta(x0, y0, z0):
    cos120 = -0.5
    sin120 = math.sqrt(3) / 2.0

    def calc(x0, y0, z0):
        y1 = -0.5 * 0.57735 * IK_F
        y0 = y0 - 0.5 * 0.57735 * IK_ED
        if abs(z0) < 1e-9:
            return None
        a = (x0**2 + y0**2 + z0**2 + IK_RF**2 - IK_RE**2 - y1**2) / (2 * z0)
        b = (y1 - y0) / z0
        d = -(a + b * y1)**2 + IK_RF * (b**2 * IK_RF + IK_RF)
        if d < 0:
            return None
        yj = (y1 - a * b - math.sqrt(d)) / (b**2 + 1)
        zj = a + b * yj
        return math.degrees(math.atan(-zj / (y1 - yj)))

    t1 = calc(x0, y0, z0)
    t2 = calc(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0)
    t3 = calc(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0)
    if t1 is None or t2 is None or t3 is None:
        return None
    return t1, t2, t3


_rng_traj = np.random.default_rng(TRAJ_SEED)
_t_ctrl = np.linspace(0.0, T_MAIN, TRAJ_N_PEAKS)


def _smooth_axis(lo, hi):
    val = _rng_traj.uniform(lo, hi, TRAJ_N_PEAKS)
    fn = interp1d(_t_ctrl, val, kind='cubic')
    return fn(t_log_main)


_X = _smooth_axis(*WS_X)
_Y = _smooth_axis(*WS_Y)
_Z = _smooth_axis(*WS_Z)

Q_LIMIT_DEG = 85.0      # batas mekanis joint (+/- derajat)

_q_list = []
for _k in range(N_DATA):
    _sol = _ik_delta(_X[_k], _Y[_k], _Z[_k])
    # tolak jika unreachable ATAU melewati batas mekanis +/-85 deg
    if _sol is None or any(abs(a) > Q_LIMIT_DEG for a in _sol):
        _q_list.append(_q_list[-1] if _q_list else (0.0, 0.0, 0.0))
    else:
        _q_list.append(_sol)
q_d_main = np.radians(np.array(_q_list))      # (N_DATA, 3), deg -> rad
for j in range(3):
    print(f"   Joint {j+1}: range [{np.degrees(q_d_main[:, j].min()):+.2f}, "
          f"{np.degrees(q_d_main[:, j].max()):+.2f}] deg")

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
# SYNERGETIC CONTROL LAW (PDF Eq. 10) - NO friction
# =======================================================
def synergetic_control_law(q, dq, q_d_t, dq_d_t, ddq_d_t,
                           mu, M_hat, C_hat, tau_g_hat):
    """tau = M*(ddq_d + c*e_dot + sigma/mu) + C*dq + tau_g"""
    e = q_d_t - q
    e_dot = dq_d_t - dq
    sigma = C_MANIFOLD * e + e_dot
    acc_command = ddq_d_t + (C_MANIFOLD * e_dot) + (sigma / mu)
    tau = (M_hat @ acc_command) + (C_hat @ dq) + tau_g_hat
    return tau


# =======================================================
# STATE DERIVATIVE (6-dim) - SCENARIO A: no electrical
# =======================================================
def state_derivative(state, q_d_t, dq_d_t, ddq_d_t, mu,
                     M_hat, M_inv, C_hat, tau_g_hat):
    """
    State: [q(3), dq(3)] -> 6-dim
    Assumption: tau_actual = tau_target (perfect motor tracking)
    Plant: M*ddq + C*dq + tau_g + b*dq = tau
    """
    q = state[0:3]
    dq = state[3:6]

    # Controller computes desired torque
    tau = synergetic_control_law(q, dq, q_d_t, dq_d_t, ddq_d_t,
                                  mu, M_hat, C_hat, tau_g_hat)

    # Plant dynamics (friction remains - real physics)
    tau_friction_plant = B_FRICTION * dq
    rhs = tau - (C_hat @ dq) - tau_g_hat - tau_friction_plant
    ddq = M_inv @ rhs

    d_state = np.concatenate([dq, ddq])
    return d_state, tau


# =======================================================
# RK4 / EULER STEPPER
# =======================================================
def rk4_step(state, q_d_t, dq_d_t, ddq_d_t, mu,
             M_hat, M_inv, C_hat, tau_g_hat, h_sub):
    k1, tau1 = state_derivative(state, q_d_t, dq_d_t, ddq_d_t, mu,
                                M_hat, M_inv, C_hat, tau_g_hat)
    k2, _ = state_derivative(state + 0.5 * h_sub * k1, q_d_t, dq_d_t,
                             ddq_d_t, mu, M_hat, M_inv, C_hat, tau_g_hat)
    k3, _ = state_derivative(state + 0.5 * h_sub * k2, q_d_t, dq_d_t,
                             ddq_d_t, mu, M_hat, M_inv, C_hat, tau_g_hat)
    k4, tau4 = state_derivative(state + h_sub * k3, q_d_t, dq_d_t,
                                ddq_d_t, mu, M_hat, M_inv, C_hat, tau_g_hat)
    new_state = state + (h_sub / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    return new_state, tau4


def euler_step(state, q_d_t, dq_d_t, ddq_d_t, mu,
               M_hat, M_inv, C_hat, tau_g_hat, h_sub):
    k1, tau_t = state_derivative(state, q_d_t, dq_d_t, ddq_d_t, mu,
                                 M_hat, M_inv, C_hat, tau_g_hat)
    return state + h_sub * k1, tau_t


def simulate_until_next_log(state, q_d_t, dq_d_t, ddq_d_t, mu, t_start):
    """Sub-step from t_start to t_start + DT_LOG."""
    s = state.copy()
    tau_last = np.zeros(3)
    stepper = rk4_step if ODE_METHOD == 'rk4' else euler_step
    t_cur = t_start
    for _ in range(N_INTERNAL_PER_LOG):
        M_hat, M_inv, C_hat, tau_g_hat = compute_dynamics(s[0:3], t_cur)
        s, tau_last = stepper(s, q_d_t, dq_d_t, ddq_d_t, mu,
                              M_hat, M_inv, C_hat, tau_g_hat, DT_SUB)
        t_cur += DT_SUB
    return s, tau_last


# =======================================================
# 7. INIT STATE & HISTORY (sized for FULL run including warm-up)
# =======================================================
state = np.zeros(6)                  # 6-dim (no electrical)
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
# 8-15. MAIN TIME LOOP (runs over N_TOTAL = warm-up + main)
# =======================================================
print(f"\n[Main Loop] Running {N_TOTAL} timesteps "
      f"({N_WARMUP} warm-up + {N_DATA} main) x "
      f"{MU_STEPS} mu candidates ...")
t_start_run = time.time()

for i in range(N_TOTAL):
    t_now = i * DT_LOG
    q_d_t = q_d[i]
    dq_d_t = dq_d[i]
    ddq_d_t = ddq_d[i]

    best_cost = float('inf')
    best_mu = candidate_mu[0]
    best_state_next = state.copy()
    best_tau = np.zeros(3)

    for mu_test in candidate_mu:
        _JAC_CACHE['J_prev'] = np.eye(3)
        _JAC_CACHE['t_prev'] = t_now

        # --- MULTI-STEP HORIZON EVALUATION ---
        # Jalankan mu_test ditahan konstan selama HORIZON langkah ke depan.
        # Akumulasi DUA hal sepanjang horizon:
        #   1. error tracking (deg)  -> mu kecil bagus di sini
        #   2. chattering torsi |Delta-tau| (Nm) antar langkah berturut
        #      -> mu kecil JELEK di sini (gain tinggi -> torsi loncat)
        # Cost gabungan ternormalisasi: trade-off jadi nyata, mu_min tidak
        # lagi otomatis menang.
        h_state = state.copy()
        h_err = 0.0
        h_chatter = 0.0
        prev_tau_in_horizon = prev_tau_vec   # acuan dtau langkah pertama
        first_state_next = None
        first_tau = None
        n_eval = 0
        for h in range(HORIZON):
            hi = i + h
            if hi >= N_TOTAL:
                break
            qd_h = q_d[hi]
            dqd_h = dq_d[hi]
            ddqd_h = ddq_d[hi]
            t_h = hi * DT_LOG
            s_next, tau_pred = simulate_until_next_log(
                h_state, qd_h, dqd_h, ddqd_h, mu_test, t_h)
            qd_next = q_d[hi + 1] if (hi + 1) < N_TOTAL else qd_h

            # tracking error (deg, rata-rata 3 joint)
            h_err += np.mean(np.abs(np.degrees(qd_next - s_next[0:3])))
            # chattering: |Delta-tau| max antar joint, vs tau langkah sebelum
            h_chatter += float(np.max(np.abs(tau_pred - prev_tau_in_horizon)))

            if h == 0:
                first_state_next = s_next
                first_tau = tau_pred
            prev_tau_in_horizon = tau_pred
            h_state = s_next
            n_eval += 1

        # rata-rata per langkah biar tak peduli HORIZON
        if n_eval > 0:
            avg_err = h_err / n_eval
            avg_chatter = h_chatter / n_eval
        else:
            avg_err = float('inf')
            avg_chatter = float('inf')

        # cost gabungan ternormalisasi
        cost = (avg_err / TRACK_NORM) + LAMBDA_CHATTER * (avg_chatter / CHATTER_NORM)

        if cost < best_cost:
            best_cost = cost
            best_mu = mu_test
            best_state_next = first_state_next
            best_tau = first_tau

    # fallback proteksi: kalau loop kandidat gagal total (sangat jarang)
    if not np.isfinite(best_cost):
        best_mu = candidate_mu[0]
        # re-simulate sekali utk dapat state next
        _JAC_CACHE['J_prev'] = np.eye(3)
        _JAC_CACHE['t_prev'] = t_now
        best_state_next, best_tau = simulate_until_next_log(
            state.copy(), q_d_t, dq_d_t, ddq_d_t, best_mu, t_now)

    history_q[i] = state[0:3]
    history_qd[i] = q_d_t
    history_dq[i] = state[3:6]
    history_dqd[i] = dq_d_t
    history_tau[i] = best_tau
    history_mu[i] = best_mu
    history_t[i] = t_now

    state = best_state_next
    prev_tau_vec = best_tau

    if (i + 1) % max(1, N_TOTAL // 20) == 0:
        elapsed = time.time() - t_start_run
        pct = 100.0 * (i + 1) / N_TOTAL
        phase = "warm-up" if i < N_WARMUP else "main"
        print(f"   Progress: {pct:5.1f}%  ({i + 1}/{N_TOTAL}) [{phase}]  "
              f"elapsed {elapsed:.1f}s")


elapsed_total = time.time() - t_start_run
print(f"\n[Done] Total time: {elapsed_total:.2f} s "
      f"({N_TOTAL / elapsed_total:.1f} steps/s)")

# =======================================================
# 16. PER-JOINT STATISTICS (MAIN PORTION ONLY - skip warm-up)
# =======================================================
# Slice to main portion only for clean statistics
main_slice = slice(N_WARMUP, N_TOTAL)
errors_deg_full = np.degrees(history_qd - history_q)
errors_deg = errors_deg_full[main_slice]    # used for stats

rmse_per_joint = np.zeros(3)
mae_per_joint = np.zeros(3)
std_per_joint = np.zeros(3)
max_per_joint = np.zeros(3)

print("\n" + "=" * 70)
print(f"PER-JOINT TRACKING STATISTICS (degrees) - main portion only")
print(f"(warm-up of {N_WARMUP} samples / {WARM_UP_SEC} s excluded)")
print("=" * 70)
print(f"{'Joint':<8}{'RMSE':>14}{'MAE':>14}{'StdDev':>14}{'Max|err|':>14}")
print("-" * 70)
stats_rows = []
for j in range(3):
    err_j = errors_deg[:, j]
    rmse_per_joint[j] = np.sqrt(np.mean(err_j ** 2))
    mae_per_joint[j] = np.mean(np.abs(err_j))
    std_per_joint[j] = np.std(err_j)
    max_per_joint[j] = np.max(np.abs(err_j))
    print(f"{j + 1:<8}{rmse_per_joint[j]:>14.6f}{mae_per_joint[j]:>14.6f}"
          f"{std_per_joint[j]:>14.6f}{max_per_joint[j]:>14.6f}")
    stats_rows.append({
        'joint': j + 1,
        'rmse_deg': rmse_per_joint[j],
        'mae_deg': mae_per_joint[j],
        'std_deg': std_per_joint[j],
        'max_abs_err_deg': max_per_joint[j]
    })

avg_rmse = np.mean(rmse_per_joint)
avg_mae = np.mean(mae_per_joint)
avg_std = np.mean(std_per_joint)
avg_max = np.mean(max_per_joint)

print("-" * 70)
print(f"{'AVG':<8}{avg_rmse:>14.6f}{avg_mae:>14.6f}"
      f"{avg_std:>14.6f}{avg_max:>14.6f}")
print("-" * 70)
avg_mu = np.mean(history_mu[main_slice])
print(f"Average optimal mu (main): {avg_mu:.6f}")
print("=" * 70)

# --- Per-row averages across the 3 joints (main portion only) ---
e_avg = np.mean(errors_deg, axis=1)
de_matrix_rad = history_dqd[main_slice] - history_dq[main_slice]
de_matrix_deg = np.degrees(de_matrix_rad)
de_avg = np.mean(de_matrix_deg, axis=1)

# =======================================================
# 17. COMBINED PLOT WITH METRICS BOX
# =======================================================
fig = plt.figure(figsize=(17, 11))
joint_colors = ['#dc2626', '#16a34a', '#2563eb']
joint_names = ['Joint 1', 'Joint 2', 'Joint 3']

# --- Tracking: all 3 joints (show FULL run including warm-up) ---
ax1 = plt.subplot(2, 2, 1)
for j in range(3):
    ax1.plot(history_t, np.degrees(history_qd[:, j]),
             color=joint_colors[j], linestyle='--', alpha=0.6,
             linewidth=1.2, label=f'{joint_names[j]} target')
    ax1.plot(history_t, np.degrees(history_q[:, j]),
             color=joint_colors[j], linestyle='-', linewidth=1.4,
             label=f'{joint_names[j]} actual')
# Shade the warm-up region
if N_WARMUP > 0:
    ax1.axvspan(0, WARM_UP_SEC, alpha=0.15, color='gray',
                label='warm-up (excluded)')
ax1.set_title('Position Tracking - All Joints',
              fontsize=11, fontweight='bold')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angle (deg)')
ax1.legend(loc='best', fontsize=8, ncol=2)
ax1.grid(True, alpha=0.3)

# --- Error per joint with stats box (full plot, stats from main only) ---
ax2 = plt.subplot(2, 2, 2)
for j in range(3):
    ax2.plot(history_t, errors_deg_full[:, j],
             color=joint_colors[j], linewidth=1.0,
             label=f'{joint_names[j]}')
if N_WARMUP > 0:
    ax2.axvspan(0, WARM_UP_SEC, alpha=0.15, color='gray')
ax2.set_title('Tracking Error per Joint', fontsize=11, fontweight='bold')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error (deg)')
ax2.legend(loc='upper right', fontsize=9)
ax2.grid(True, alpha=0.3)

# Stats text box on the error plot
stats_text = "TRACKING METRICS (deg)\n" + "-" * 32 + "\n"
stats_text += f"{'':<8}{'RMSE':>8}{'MAE':>8}{'Std':>8}\n"
for j in range(3):
    stats_text += (f"J{j+1:<7}{rmse_per_joint[j]:>8.4f}"
                   f"{mae_per_joint[j]:>8.4f}{std_per_joint[j]:>8.4f}\n")
stats_text += "-" * 32 + "\n"
stats_text += (f"{'AVG':<8}{avg_rmse:>8.4f}"
               f"{avg_mae:>8.4f}{avg_std:>8.4f}")

ax2.text(0.02, 0.02, stats_text, transform=ax2.transAxes,
         fontsize=8.5, family='monospace',
         verticalalignment='bottom',
         bbox=dict(boxstyle='round,pad=0.5',
                   facecolor='#fef3c7', edgecolor='#d97706',
                   alpha=0.92))

# --- Torque per joint ---
ax3 = plt.subplot(2, 2, 3)
for j in range(3):
    ax3.plot(history_t, history_tau[:, j],
             color=joint_colors[j], linewidth=1.0,
             label=f'{joint_names[j]}')
if N_WARMUP > 0:
    ax3.axvspan(0, WARM_UP_SEC, alpha=0.15, color='gray')
ax3.set_title('Control Torque (Nm)', fontsize=11, fontweight='bold')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Torque (Nm)')
ax3.legend(loc='best', fontsize=9)
ax3.grid(True, alpha=0.3)

# --- Mu over time ---
ax4 = plt.subplot(2, 2, 4)
ax4.step(history_t, history_mu, where='post',
         color='#7c3aed', linewidth=0.8)
if N_WARMUP > 0:
    ax4.axvspan(0, WARM_UP_SEC, alpha=0.15, color='gray')
ax4.set_title(f'Optimal mu vs Time (avg main: {avg_mu:.6f})',
              fontsize=11, fontweight='bold')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('mu')
ax4.grid(True, alpha=0.3)

# Overall figure title with summary
fig.suptitle(
    f'Synergetic Delta Robot - NF Training Dataset (v7)  |  '
    f'N={N_DATA} (+{N_WARMUP} warm-up)  |  Avg RMSE={avg_rmse:.4f} deg  '
    f'MAE={avg_mae:.4f} deg  Std={avg_std:.4f} deg',
    fontsize=12, fontweight='bold', y=0.995)

plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig(str(_OUT_DIR / 'plot_synergetic_delta_v7.png'), dpi=150)
print("\nSaved: plot_synergetic_delta_v7.png")

# =======================================================
# 18. OUTPUT CSV FOR NF TRAINING
# =======================================================
# By default, exclude warm-up samples from the training CSV
if INCLUDE_WARMUP_IN_CSV:
    csv_slice = slice(0, N_TOTAL)
    csv_note = "FULL (warm-up included)"
else:
    csv_slice = main_slice
    csv_note = f"MAIN ONLY (warm-up of {N_WARMUP} samples excluded)"

# Re-compute per-row averages for the chosen slice
e_csv = np.degrees(history_qd[csv_slice] - history_q[csv_slice])
de_csv = np.degrees(history_dqd[csv_slice] - history_dq[csv_slice])
e_avg_csv = np.mean(e_csv, axis=1)
de_avg_csv = np.mean(de_csv, axis=1)

# Macro variable sigma = c*e + e_dot (the actual driver of mu).
# Computed in DEGREES to match e_deg/de_deg feature scaling.
sigma_csv = C_MANIFOLD * e_csv + de_csv          # per-joint, shape (N,3)
sigma_avg_csv = C_MANIFOLD * e_avg_csv + de_avg_csv

# tau_prev_avg: |tau| langkah SEBELUMNYA, rata-rata 3 joint (Nm).
# Fitur ini diperlukan karena cost sekarang juga melihat chattering
# |Delta-tau|, jadi mu optimal tergantung tau_prev. Saat deploy, robot
# tahu tau yg baru saja dikirim -> fitur ini realistis tersedia.
tau_abs = np.abs(history_tau[csv_slice])         # (N, 3)
tau_avg = np.mean(tau_abs, axis=1)               # (N,) rata-rata |tau| per baris
# geser 1 langkah: baris i pakai tau dari baris i-1.
# baris pertama set ke 0 (tak ada langkah sebelumnya yg valid).
tau_prev_avg_csv = np.concatenate([[0.0], tau_avg[:-1]])

# Reset time so main portion starts at t = 0 (cleaner for NF training)
t_csv = history_t[csv_slice] - history_t[csv_slice][0]

data_dict = {'time': t_csv,
             'optimal_mu': history_mu[csv_slice],
             'sigma_avg': sigma_avg_csv,
             'e_deg_avg': e_avg_csv,
             'de_deg_avg': de_avg_csv,
             'tau_prev_avg': tau_prev_avg_csv}
for j in range(3):
    jp = j + 1
    data_dict[f'q_d_{jp}_deg'] = np.degrees(history_qd[csv_slice, j])
    data_dict[f'q_{jp}_deg'] = np.degrees(history_q[csv_slice, j])
    data_dict[f'e_{jp}_deg'] = e_csv[:, j]
    data_dict[f'sigma_{jp}'] = sigma_csv[:, j]
    data_dict[f'dq_d_{jp}_deg_s'] = np.degrees(history_dqd[csv_slice, j])
    data_dict[f'dq_{jp}_deg_s'] = np.degrees(history_dq[csv_slice, j])
    data_dict[f'de_{jp}_deg_s'] = de_csv[:, j]
    data_dict[f'tau_{jp}_Nm'] = history_tau[csv_slice, j]

df_dataset = pd.DataFrame(data_dict)
df_dataset.to_csv(str(_OUT_DIR / 'dataset_synergetic_delta_v7.csv'), index=False)
print(f"Saved: dataset_synergetic_delta_v7.csv  "
      f"({len(df_dataset)} rows, {len(df_dataset.columns)} cols)  [{csv_note}]")

df_stats = pd.DataFrame(stats_rows)
df_stats.loc[len(df_stats)] = {
    'joint': 'AVG', 'rmse_deg': avg_rmse, 'mae_deg': avg_mae,
    'std_deg': avg_std, 'max_abs_err_deg': avg_max
}
df_stats.to_csv(str(_OUT_DIR / 'stats_synergetic_delta_v7.csv'), index=False)
print(f"Saved: stats_synergetic_delta_v7.csv")

print("\n[FINISHED]")
plt.show()