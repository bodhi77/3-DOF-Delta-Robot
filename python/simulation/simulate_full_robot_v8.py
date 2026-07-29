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
M_PT_BASE = 0.317         # Moving plate mass (tanpa payload)
M_PT = M_PT_BASE          # Massa efektif (akan di-reassign per episode)
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
N_DATA = 500              # Number of logged samples per episode
DT_LOG = 0.004            # Logging interval (s)

# =======================================================
# 2b. MULTI-EPISODE / PAYLOAD VARIATION (v8)
# =======================================================
# Tiap episode = satu trajektori dengan payload random di [PAYLOAD_MIN_KG,
# PAYLOAD_MAX_KG]. payload ditambah ke M_PT, dan disimpan sebagai kolom
# input di CSV training -> model NF bisa belajar (e, de, payload) -> mu.
N_EPISODES        = 5         # jumlah episode (= ragam payload+trajektori)
PAYLOAD_MIN_KG    = 0.0       # batas bawah payload random
PAYLOAD_MAX_KG    = 1.0       # batas atas payload random
EPISODE_BASE_SEED = 100       # seed dasar; tiap episode pakai SEED+ep_idx

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
# 5. TRAJECTORY GENERATION (fungsi, dipanggil tiap episode)
# =======================================================
t_log_main = np.linspace(0.0, T_MAIN, N_DATA)

# Workspace bounds (mm) meniru ruang kerja deploy helix
TRAJ_N_PEAKS = 20
WS_X = (-60.0, 60.0)
WS_Y = (-60.0, 60.0)
WS_Z = (-340.0, -190.0)
IK_ED, IK_F, IK_RE, IK_RF = 90.0, 70.0, 250.0, 175.0
Q_LIMIT_DEG = 85.0


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


def smoothstep(t):
    """Smoothstep curve (0 -> 1) with zero derivatives at endpoints."""
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def generate_trajectory(traj_seed):
    """Generate q_d, dq_d, ddq_d (with warm-up) for one episode.
    Returns arrays of shape (N_TOTAL, 3)."""
    rng = np.random.default_rng(traj_seed)
    t_ctrl = np.linspace(0.0, T_MAIN, TRAJ_N_PEAKS)

    def smooth_axis(lo, hi):
        val = rng.uniform(lo, hi, TRAJ_N_PEAKS)
        fn = interp1d(t_ctrl, val, kind='cubic')
        return fn(t_log_main)

    X = smooth_axis(*WS_X)
    Y = smooth_axis(*WS_Y)
    Z = smooth_axis(*WS_Z)

    q_list = []
    for k in range(N_DATA):
        sol = _ik_delta(X[k], Y[k], Z[k])
        if sol is None or any(abs(a) > Q_LIMIT_DEG for a in sol):
            q_list.append(q_list[-1] if q_list else (0.0, 0.0, 0.0))
        else:
            q_list.append(sol)
    q_d_main = np.radians(np.array(q_list))      # (N_DATA, 3) rad

    if N_WARMUP > 0:
        q_d_first = q_d_main[0]
        s_curve = smoothstep(np.linspace(0.0, 1.0, N_WARMUP, endpoint=False))
        q_d_warmup = np.outer(s_curve, q_d_first)
        q_d = np.vstack([q_d_warmup, q_d_main])
    else:
        q_d = q_d_main

    dq_d = np.gradient(q_d, DT_LOG, axis=0)
    ddq_d = np.gradient(dq_d, DT_LOG, axis=0)
    return q_d, dq_d, ddq_d


t_log = np.arange(N_TOTAL) * DT_LOG


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
# 7. RUN ONE EPISODE — main simulation loop wrapped as function
# =======================================================
candidate_mu = np.linspace(MU_MIN, MU_MAX, MU_STEPS)


def run_episode(payload_kg, q_d, dq_d, ddq_d):
    """Run full sim for one trajectory with the given payload.
    The global M_PT is reassigned so all dynamics functions see the
    new effective mass (M_PT_BASE + payload_kg)."""
    global M_PT
    M_PT = M_PT_BASE + payload_kg

    state = np.zeros(6)
    prev_tau_vec = np.zeros(3)

    h_q   = np.zeros((N_TOTAL, 3))
    h_qd  = np.zeros((N_TOTAL, 3))
    h_dq  = np.zeros((N_TOTAL, 3))
    h_dqd = np.zeros((N_TOTAL, 3))
    h_tau = np.zeros((N_TOTAL, 3))
    h_mu  = np.zeros(N_TOTAL)
    h_t   = np.zeros(N_TOTAL)

    for i in range(N_TOTAL):
        t_now = i * DT_LOG
        q_d_t   = q_d[i]
        dq_d_t  = dq_d[i]
        ddq_d_t = ddq_d[i]

        best_cost       = float('inf')
        best_mu         = candidate_mu[0]
        best_state_next = state.copy()
        best_tau        = np.zeros(3)

        for mu_test in candidate_mu:
            _JAC_CACHE['J_prev'] = np.eye(3)
            _JAC_CACHE['t_prev'] = t_now

            h_state = state.copy()
            h_err = 0.0
            h_chatter = 0.0
            prev_tau_in_horizon = prev_tau_vec
            first_state_next = None
            first_tau = None
            n_eval = 0
            for h in range(HORIZON):
                hi = i + h
                if hi >= N_TOTAL:
                    break
                qd_h   = q_d[hi]
                dqd_h  = dq_d[hi]
                ddqd_h = ddq_d[hi]
                t_h = hi * DT_LOG
                s_next, tau_pred = simulate_until_next_log(
                    h_state, qd_h, dqd_h, ddqd_h, mu_test, t_h)
                qd_next = q_d[hi + 1] if (hi + 1) < N_TOTAL else qd_h

                h_err += np.mean(np.abs(np.degrees(qd_next - s_next[0:3])))
                h_chatter += float(np.max(np.abs(tau_pred - prev_tau_in_horizon)))

                if h == 0:
                    first_state_next = s_next
                    first_tau = tau_pred
                prev_tau_in_horizon = tau_pred
                h_state = s_next
                n_eval += 1

            if n_eval > 0:
                avg_err = h_err / n_eval
                avg_chatter = h_chatter / n_eval
            else:
                avg_err = float('inf')
                avg_chatter = float('inf')

            cost = (avg_err / TRACK_NORM) + LAMBDA_CHATTER * (avg_chatter / CHATTER_NORM)

            if cost < best_cost:
                best_cost = cost
                best_mu = mu_test
                best_state_next = first_state_next
                best_tau = first_tau

        if not np.isfinite(best_cost):
            best_mu = candidate_mu[0]
            _JAC_CACHE['J_prev'] = np.eye(3)
            _JAC_CACHE['t_prev'] = t_now
            best_state_next, best_tau = simulate_until_next_log(
                state.copy(), q_d_t, dq_d_t, ddq_d_t, best_mu, t_now)

        h_q[i]   = state[0:3]
        h_qd[i]  = q_d_t
        h_dq[i]  = state[3:6]
        h_dqd[i] = dq_d_t
        h_tau[i] = best_tau
        h_mu[i]  = best_mu
        h_t[i]   = t_now

        state = best_state_next
        prev_tau_vec = best_tau

    return h_q, h_qd, h_dq, h_dqd, h_tau, h_mu, h_t


# =======================================================
# 8. MULTI-EPISODE OUTER LOOP
# =======================================================
print(f"\n[Multi-Episode] Running {N_EPISODES} episodes "
      f"x {N_TOTAL} steps x {MU_STEPS} mu candidates ...")
print(f"  payload range: [{PAYLOAD_MIN_KG:.3f}, {PAYLOAD_MAX_KG:.3f}] kg")

ep_rng = np.random.default_rng(EPISODE_BASE_SEED)
t_start_all = time.time()

# Akumulator hasil tiap episode
all_episodes = []

for ep_idx in range(N_EPISODES):
    payload = float(ep_rng.uniform(PAYLOAD_MIN_KG, PAYLOAD_MAX_KG))
    traj_seed = EPISODE_BASE_SEED + 1000 + ep_idx

    print(f"\n[Episode {ep_idx + 1}/{N_EPISODES}]  "
          f"payload = {payload:.4f} kg  traj_seed = {traj_seed}")

    q_d_ep, dq_d_ep, ddq_d_ep = generate_trajectory(traj_seed)
    print(f"  q_d range deg per joint: "
          f"J1=[{np.degrees(q_d_ep[:, 0].min()):+.1f},"
          f"{np.degrees(q_d_ep[:, 0].max()):+.1f}] "
          f"J2=[{np.degrees(q_d_ep[:, 1].min()):+.1f},"
          f"{np.degrees(q_d_ep[:, 1].max()):+.1f}] "
          f"J3=[{np.degrees(q_d_ep[:, 2].min()):+.1f},"
          f"{np.degrees(q_d_ep[:, 2].max()):+.1f}]")

    t_ep = time.time()
    h_q, h_qd, h_dq, h_dqd, h_tau, h_mu, h_t = run_episode(
        payload, q_d_ep, dq_d_ep, ddq_d_ep)
    elapsed_ep = time.time() - t_ep
    rmse_main = np.sqrt(np.mean(
        np.degrees(h_qd[N_WARMUP:] - h_q[N_WARMUP:])**2))
    print(f"  episode done in {elapsed_ep:.1f}s  "
          f"RMSE main (deg) = {rmse_main:.4f}  "
          f"avg mu = {h_mu[N_WARMUP:].mean():.6f}")

    all_episodes.append({
        'payload': payload,
        'h_q':   h_q,   'h_qd':  h_qd,
        'h_dq':  h_dq,  'h_dqd': h_dqd,
        'h_tau': h_tau, 'h_mu':  h_mu, 'h_t': h_t,
    })

elapsed_total = time.time() - t_start_all
print(f"\n[All episodes done] total {elapsed_total:.1f}s "
      f"({elapsed_total / N_EPISODES:.1f}s per episode)")

# =======================================================
# 9. COMBINE EPISODES INTO ONE TRAINING CSV
# =======================================================
print("\n[Saving] combining episodes into one CSV ...")

if INCLUDE_WARMUP_IN_CSV:
    csv_slice = slice(0, N_TOTAL)
else:
    csv_slice = slice(N_WARMUP, N_TOTAL)

per_episode_dfs = []

for ep_idx, ep in enumerate(all_episodes):
    payload = ep['payload']
    h_q   = ep['h_q'][csv_slice]
    h_qd  = ep['h_qd'][csv_slice]
    h_dq  = ep['h_dq'][csv_slice]
    h_dqd = ep['h_dqd'][csv_slice]
    h_tau = ep['h_tau'][csv_slice]
    h_mu  = ep['h_mu'][csv_slice]
    h_t   = ep['h_t'][csv_slice]
    n_rows = len(h_mu)

    e_csv  = np.degrees(h_qd - h_q)
    de_csv = np.degrees(h_dqd - h_dq)
    e_avg_csv  = np.mean(e_csv,  axis=1)
    de_avg_csv = np.mean(de_csv, axis=1)

    sigma_csv     = C_MANIFOLD * e_csv + de_csv
    sigma_avg_csv = C_MANIFOLD * e_avg_csv + de_avg_csv

    tau_abs = np.abs(h_tau)
    tau_avg = np.mean(tau_abs, axis=1)
    tau_prev_avg_csv = np.concatenate([[0.0], tau_avg[:-1]])

    t_local = h_t - h_t[0]   # tiap episode mulai dari t = 0

    data_dict = {
        'episode':      np.full(n_rows, ep_idx, dtype=int),
        'payload_kg':   np.full(n_rows, payload, dtype=float),
        'time':         t_local,
        'optimal_mu':   h_mu,
        'sigma_avg':    sigma_avg_csv,
        'e_deg_avg':    e_avg_csv,
        'de_deg_avg':   de_avg_csv,
        'tau_prev_avg': tau_prev_avg_csv,
    }
    for j in range(3):
        jp = j + 1
        data_dict[f'q_d_{jp}_deg']    = np.degrees(h_qd[:, j])
        data_dict[f'q_{jp}_deg']      = np.degrees(h_q[:, j])
        data_dict[f'e_{jp}_deg']      = e_csv[:, j]
        data_dict[f'sigma_{jp}']      = sigma_csv[:, j]
        data_dict[f'dq_d_{jp}_deg_s'] = np.degrees(h_dqd[:, j])
        data_dict[f'dq_{jp}_deg_s']   = np.degrees(h_dq[:, j])
        data_dict[f'de_{jp}_deg_s']   = de_csv[:, j]
        data_dict[f'tau_{jp}_Nm']     = h_tau[:, j]

    per_episode_dfs.append(pd.DataFrame(data_dict))

df_dataset = pd.concat(per_episode_dfs, ignore_index=True)
df_dataset.to_csv(str(_OUT_DIR / 'dataset_synergetic_delta_v8.csv'), index=False)
print(f"  Saved: dataset_synergetic_delta_v8.csv  "
      f"({len(df_dataset)} rows, {len(df_dataset.columns)} cols, "
      f"{N_EPISODES} episodes)")

# =======================================================
# 10. QUICK SANITY PLOT (last episode only)
# =======================================================
ep_last = all_episodes[-1]
fig, axes = plt.subplots(2, 2, figsize=(15, 9))

ax1 = axes[0, 0]
for j in range(3):
    ax1.plot(ep_last['h_t'], np.degrees(ep_last['h_qd'][:, j]),
             linestyle='--', alpha=0.6, label=f'J{j+1} target')
    ax1.plot(ep_last['h_t'], np.degrees(ep_last['h_q'][:, j]),
             linestyle='-',  linewidth=1.2, label=f'J{j+1} actual')
if N_WARMUP > 0:
    ax1.axvspan(0, WARM_UP_SEC, alpha=0.15, color='gray',
                label='warm-up (excluded)')
ax1.set_title(f"Position Tracking (last episode, payload={ep_last['payload']:.3f} kg)",
              fontweight='bold')
ax1.set_xlabel('Time (s)'); ax1.set_ylabel('Angle (deg)')
ax1.legend(fontsize=8, ncol=2); ax1.grid(True, alpha=0.3)

ax2 = axes[0, 1]
err_deg = np.degrees(ep_last['h_qd'] - ep_last['h_q'])
for j in range(3):
    ax2.plot(ep_last['h_t'], err_deg[:, j], label=f'J{j+1}')
if N_WARMUP > 0:
    ax2.axvspan(0, WARM_UP_SEC, alpha=0.15, color='gray')
ax2.set_title('Tracking Error per Joint (last episode)', fontweight='bold')
ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Error (deg)')
ax2.legend(); ax2.grid(True, alpha=0.3)

ax3 = axes[1, 0]
for j in range(3):
    ax3.plot(ep_last['h_t'], ep_last['h_tau'][:, j], label=f'J{j+1}', linewidth=0.9)
if N_WARMUP > 0:
    ax3.axvspan(0, WARM_UP_SEC, alpha=0.15, color='gray')
ax3.set_title('Control Torque (last episode)', fontweight='bold')
ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Torque (Nm)')
ax3.legend(); ax3.grid(True, alpha=0.3)

ax4 = axes[1, 1]
# mu distribution across ALL episodes (main portion only)
all_mu_main = np.concatenate(
    [ep['h_mu'][N_WARMUP:] for ep in all_episodes])
all_payload_main = np.concatenate(
    [np.full(N_DATA, ep['payload']) for ep in all_episodes])
sc = ax4.scatter(all_payload_main, all_mu_main, s=4, alpha=0.4, c='#7c3aed')
ax4.set_title(f'Optimal mu vs payload (all {N_EPISODES} episodes)',
              fontweight='bold')
ax4.set_xlabel('payload (kg)'); ax4.set_ylabel('optimal mu')
ax4.grid(True, alpha=0.3)

fig.suptitle(
    f'Synergetic Delta Robot v8 — multi-episode dataset  |  '
    f'{N_EPISODES} episodes, payload [{PAYLOAD_MIN_KG}, {PAYLOAD_MAX_KG}] kg',
    fontsize=12, fontweight='bold', y=0.995)
plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig(str(_OUT_DIR / 'plot_synergetic_delta_v8.png'), dpi=150)
print("Saved: plot_synergetic_delta_v8.png")

print("\n[FINISHED]")
plt.show()
