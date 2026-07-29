"""
=====================================================================
OFFLINE SIMULATION: 3-DOF Delta Robot IMPULSE DISTURBANCE TEST
with NEURO-FUZZY SYNERGETIC control (mu from T1/IT2 model via PSO).

Adapted from Auzan's impulse-rejection experiment (4-DOF -> 3-DOF). Pick a
scenario with IMPULSE_SCENARIO below:
  Scenario 1 - impulse uncertainty added straight into the control torque at
               t = 1 s, magnitude [-1, -1, 1] N.m on the three joints
               (adapted from Auzan's [-1, -1, 1, -1]); zero at every other step.
  Scenario 2 - Task 4: an unknown end-effector impulse force (0.5 N) fired at
               t = 5 s, mapped to equivalent joint torques via J^T; zero at
               every other step.
The robot tracks the reference trajectory; the impulse is a short pulse that
the synergetic / neuro-fuzzy law must reject. All impulse settings (scenario,
timing, strength, width) live in the IMPULSE TEST block at the top of CONFIGURATION.

No serial, no Arduino. The entire plant + control law is simulated
in Python (ported from FuzzySynergeticDeltaRobotFullModelRev.ino):
  - Forward kinematics + Jacobian (J, dJ/dt)
  - Dynamics matrices M_hat = I_a*I + m_pt*J^T J,  C_hat = m_pt*J^T dJ,  tau_g
  - Synergetic law: tau = M_hat*(ddq_d + c*e_dot + sigma/mu) + C_hat*dq + tau_g
  - Inner current PID -> PWM -> voltage -> motor (electrical+mechanical model)
  - mu per-joint predicted by neuro-fuzzy model (predict_mu, same as deploy_flexible.py)

Output: CSV (31-column format like deploy) + tracking / mu / current / trajectory graphs.

=====================================================================
REVISION (anomaly fix):
  The original script used a single M_PT for BOTH the control law and the
  plant. Adding payload therefore changed the controller's own model too,
  so there was zero model mismatch: gravity/Coriolis cancelled exactly and
  a heavier (more damped) plant simply oscillated less -> torque RMS and
  chatter went DOWN with payload, which is non-physical and did not match
  the real robot.

  Fix: the controller uses the NOMINAL plate mass M_PT_NOM (it does not know
  the payload), while the plant integrates with the TRUE mass M_PT_TRUE.
  The payload is now an UNMODELLED disturbance that the synergetic /
  neuro-fuzzy law must reject, so the simulation degrades under load like
  the real platform. The Jacobian is computed once per step and reused for
  both mass values to keep the numerical dJ/dt cache consistent.
=====================================================================
"""
import json
import math
import csv
import os
import sys
from pathlib import Path
import numpy as np

_python_dir = Path(__file__).resolve().parent
while _python_dir.name != "python":
    _python_dir = _python_dir.parent
_root_dir = _python_dir.parent
sys.path.insert(0, str(_python_dir / "trajectories"))
TRAINED_ARTIFACTS_DIR = _root_dir / "data" / "trained_artifacts"

# =====================================================================
# CONFIGURATION
# =====================================================================
# =====================================================================
# ===========  IMPULSE TEST  --  EDIT ONLY THIS BLOCK  ================
# =====================================================================
#  1) Choose the scenario.   2) Tune its numbers.   Nothing else to touch.
#  If the kick is hard to see -> raise IMPULSE_GAIN, or widen IMPULSE_WIDTH.
IMPULSE_SCENARIO = 1        # 1 = impulse into the control torque  (use a STATIC reference)
                            # 2 = end-effector impulse force        (use a DYNAMIC trajectory, Task 4)

IMPULSE_GAIN     = 60.0      # overall strength multiplier   (bigger = stronger kick)
IMPULSE_WIDTH    = 0.02     # how long the pulse is applied (s); bigger = more visible; 0 = single 1 ms step
IMPULSE_TIME     = None     # firing instant (s); None = use the scenario default (S1_TIME / S2_TIME)

# ---- Scenario 1 numbers: impulse straight into the control torque ----
S1_TIME   = 1.0                            # firing instant (s)
S1_TORQUE = np.array([-1.0, -1.0, 1.0])    # N.m per joint (adapted from Auzan [-1,-1,1,-1])

# ---- Scenario 2 numbers: unknown end-effector force (Task 4) ----
S2_TIME   = 5.0                            # firing instant (s)
S2_FORCE  = 0.5                            # end-effector force magnitude (N)
S2_DIR    = np.array([0.0, 0.0, -1.0])     # push direction (auto-normalized); -Z = downward

RECOVERY_S = 1.0            # extra time kept AFTER the impulse to watch the recovery (s)
# =====================================================================

USE_NEUROFUZZY    = True         # True: use joblib model; False: constant mu (FALLBACK_MU)
FALLBACK_MU       = 0.007         # Used if model fails to load / USE_NEUROFUZZY=False
ENABLE_FEEDFORWARD = False       # True: use dq_d, ddq_d (numeric derivative of qd); False: faithful to deploy (=0)
DT                = 1e-3         # Simulation step (1 kHz, = Arduino current-PID rate)
LOG_DECIM         = 10           # Write log every N steps (1 kHz/10 = 100 Hz)
OUT_DIR           = str(_root_dir / "results" / "controller_sim" / "neuro_fuzzy_it2_nfsc")
OUT_FIG           = os.path.join(OUT_DIR, "sim_impulse_neurofuzzy.png")

# =====================================================================
# GEOMETRIC CONSTANTS (mm) - for IK & FK logging (same as deploy)
# =====================================================================
ed = 90.0    # end effector
f  = 70.0    # base
re = 250.0
rf = 175.0

sqrt3 = math.sqrt(3.0)
pi = math.pi
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60 = sqrt3
sin30 = 0.5
tan30 = 1.0 / sqrt3

# =====================================================================
# REFERENCE TRAJECTORY (User's module - same as StaticMu / deploy)
# =====================================================================
from stepTraj_static import (
    get_trajectory_sample,
    get_trajectory_index,
    TOTAL_TRAJECTORY_STEPS,
    SAMPLE_RATE,
)

# =====================================================================
# PHYSICAL DYNAMIC PARAMETERS (m, rad) - ported from ...FullModelRev.ino
# =====================================================================
PAYLOADS  = 0.0       # TRUE payload on the moving plate (kg). Set 0.0 for the unloaded run.

GRAVITY   = 9.81
L_A       = 0.175      # upper arm length (m)
L_B       = 0.25       # lower arm length (m)

# --- key fix: split the moving-plate mass into "what the controller knows"
#     and "what the plant actually carries" ---
M_PT_NOM  = 0.317                # nominal plate mass assumed by the CONTROLLER (no payload)
M_PT_TRUE = 0.317 + PAYLOADS     # actual plate mass that drives the PLANT

I_A_VAL   = 0.0443     # upper arm + motor inertia (diagonal)
M_A       = 0.046      # upper arm mass (kg)
L_C       = 0.08       # arm center of mass (m)

ED_M   = 0.09
F_M    = 0.07
R_DIST = F_M - ED_M

TAN30_D = 0.5773502691896257
TAN60_D = 1.7320508075688772
SIN30_D = 0.5

PHI     = np.array([0.0, 2.0943951023931953, 4.188790204786391])
COS_PHI = np.cos(PHI)
SIN_PHI = np.sin(PHI)

# Motor (for inner current loop + electrical plant)
Kt = 1.0          # torque constant   [!] placeholder (see chat notes)
Ke = 1.0          # back-EMF constant [!]
Ra = 3.4          # armature resistance (Ohm)
L_elec = 0.00018  # armature inductance (H)

# Synergetic + current PID (according to Rev firmware)
c_manifold = 80.0
curKp, curKi, curKd = 20.0, 5.0, 0.015
VSUPPLY    = 12.0
PWM_MAX    = 4095.0
I_TERM_LIM = 1500.0

# Output CSV for the impulse-disturbance test
OUT_CSV = os.path.join(OUT_DIR, "impulse_NeuroIT2FS-SIM.csv")

# =====================================================================
# NEURO-FUZZY MODEL (predict_mu) - logic identical to deploy_flexible.py
# =====================================================================
def load_neurofuzzy():
    """Load T1/IT2 model + metadata, return predict_mu(e_rad, de_rad) function."""
    import joblib
    from pathlib import Path
    from pyit2fls import (IT2Mamdani_ML, IT2Mamdani, IT2TSK, IT2TSK_ML,
                          IT2FS_Gaussian_UncertMean, IT2FS_Gaussian_UncertStd,
                          Optimizer)
    try:
        from pyit2fls import T1Mamdani, T1TSK
        from pyit2fls.learning import T1Mamdani_ML, T1TSK_ML
        _HAS_T1 = True
    except Exception:
        _HAS_T1 = False

    def _unmangle(cls, tag):
        pref = f"_{tag}__"
        for attr in dir(cls):
            if attr.startswith(pref):
                setattr(cls, attr.replace(pref, "__"), getattr(cls, attr))

    _unmangle(IT2Mamdani, "IT2Mamdani")
    _unmangle(IT2Mamdani_ML, "IT2Mamdani_ML")
    _unmangle(IT2TSK, "IT2Mamdani")
    _unmangle(IT2TSK_ML, "IT2Mamdani_ML")
    if _HAS_T1:
        _unmangle(T1Mamdani, "T1Mamdani")
        _unmangle(T1Mamdani_ML, "T1Mamdani_ML")
        _unmangle(T1TSK, "T1Mamdani")
        _unmangle(T1TSK_ML, "T1Mamdani_ML")

    ARTIFACT_DIR = TRAINED_ARTIFACTS_DIR
    _CANDIDATES = [
        # ("t1mamdani_pso_model.joblib", "t1mamdani_pso_metadata.json"),
        # ("it2mamdani_pso_model.joblib", "it2mamdani_pso_metadata.json"),
        ("it2mamdani_pso_model.joblib", "it2mamdani_pso_metadata.json"),
        ("t1mamdani_pso_model.joblib", "t1mamdani_pso_metadata.json"),
    ]
    MODEL_PATH = META_PATH = None
    for _mfile, _metafile in _CANDIDATES:
        if (ARTIFACT_DIR / _metafile).exists() and (ARTIFACT_DIR / _mfile).exists():
            MODEL_PATH = ARTIFACT_DIR / _mfile
            META_PATH = ARTIFACT_DIR / _metafile
            break
    if MODEL_PATH is None:
        raise FileNotFoundError(
            f"Could not find t1/it2 model in {ARTIFACT_DIR}. "
            f"Make sure *_pso_model.joblib + *_pso_metadata.json exist.")

    trained_model = joblib.load(MODEL_PATH)
    with open(META_PATH, "r", encoding="utf-8") as fmeta:
        META = json.load(fmeta)

    MODEL_TYPE = META.get("model_type", "unknown")
    print(f"[MODEL] {MODEL_TYPE}  <- {MODEL_PATH.name}")

    X_LO = np.asarray(META["input_scaling"]["lo"], dtype=float)
    X_HI = np.asarray(META["input_scaling"]["hi"], dtype=float)
    Y_LO = float(META["output_scaling"]["lo"])
    Y_HI = float(META["output_scaling"]["hi"])
    print(f"[SCALE] input lo={X_LO}  hi={X_HI}")
    print(f"[SCALE] output lo={Y_LO:.6f}  hi={Y_HI:.6f}")

    def _scale_input(x_deg):
        return np.clip((x_deg - X_LO) / (X_HI - X_LO), 0.0, 1.0)

    def _unscale_output(y_scaled):
        return y_scaled * (Y_HI - Y_LO) + Y_LO

    def predict_mu(e_rad, de_rad):
        e_deg = math.degrees(e_rad)
        de_deg = math.degrees(de_rad)
        x_deg = np.array([e_deg, de_deg], dtype=float)
        x_scaled = _scale_input(x_deg)
        y_scaled = trained_model.score(np.array([x_scaled]))
        y_scaled = float(y_scaled[0])
        if not math.isfinite(y_scaled):
            y_scaled = 0.0
        y_scaled = min(max(y_scaled, 0.0), 1.0)
        return float(_unscale_output(y_scaled))

    return predict_mu


# =====================================================================
# KINEMATICS & DYNAMICS (dynamics, in meter/rad) - ported from Rev
# =====================================================================
def forward_kinematics_dyn(q):
    """End-effector position (m) from joint angles q (rad). Ported from forwardKinematics."""
    q1, q2, q3 = q
    t_val = (F_M - ED_M) * TAN30_D * 0.5

    y1 = -(t_val + L_A * math.cos(q1)); z1 = -L_A * math.sin(q1)
    y2 = (t_val + L_A * math.cos(q2)) * SIN30_D; x2 = y2 * TAN60_D; z2 = -L_A * math.sin(q2)
    y3 = (t_val + L_A * math.cos(q3)) * SIN30_D; x3 = -y3 * TAN60_D; z3 = -L_A * math.sin(q3)

    dnm = (y2 - y1) * x3 - (y3 - y1) * x2
    if abs(dnm) < 1e-9:
        return np.array([0.0, 0.0, -0.3])

    w1 = y1*y1 + z1*z1
    w2 = x2*x2 + y2*y2 + z2*z2
    w3 = x3*x3 + y3*y3 + z3*z3

    a1c = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1c = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) * 0.5
    a2c = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2c = ((w2 - w1) * x3 - (w3 - w1) * x2) * 0.5

    A = a1c*a1c + a2c*a2c + dnm*dnm
    B = 2.0 * (a1c * b1c + a2c * (b2c - y1 * dnm) - z1 * dnm * dnm)
    C = (b2c - y1 * dnm)**2 + b1c*b1c + dnm*dnm*(z1*z1 - L_B*L_B)
    disc = B*B - 4.0*A*C
    if disc < 0.0:
        return np.array([0.0, 0.0, -0.3])

    zA = -0.5 * (B + math.sqrt(disc)) / A
    xA = (a1c * zA + b1c) / dnm
    yA = (a2c * zA + b2c) / dnm
    return np.array([xA, yA, zA])


class DynCache:
    """Stores previous J & t for numerical dJ/dt."""
    def __init__(self):
        self.J_prev = np.eye(3)
        self.t_prev = 0.0


def compute_jacobian(t_now, q, cache):
    """Return J (3x3) and dJ/dt (3x3). Ported from computeJacobian."""
    pos = forward_kinematics_dyn(q)
    px, py, pz = pos
    S_mat = np.zeros((3, 3))
    S_b_diag = np.zeros(3)

    for i in range(3):
        ci, si = COS_PHI[i], SIN_PHI[i]
        # b_i = R_zi @ [L_A sin q, 0, L_A cos q]
        vb_x = L_A * math.sin(q[i]); vb_z = L_A * math.cos(q[i])
        bi_x = ci * vb_x; bi_y = si * vb_x; bi_z = vb_z
        # s_i = pos - R_zi @ [R_DIST + L_A cos q, 0, -L_A sin q]
        vs_x = R_DIST + L_A * math.cos(q[i]); vs_z = -L_A * math.sin(q[i])
        Rvs_x = ci * vs_x; Rvs_y = si * vs_x; Rvs_z = vs_z
        si_x = px - Rvs_x; si_y = py - Rvs_y; si_z = pz - Rvs_z
        S_mat[i, :] = [si_x, si_y, si_z]
        S_b_diag[i] = si_x * bi_x + si_y * bi_y + si_z * bi_z

    # J = solve(S_mat, -diag(S_b_diag))
    Bmat = -np.diag(S_b_diag)
    try:
        J = np.linalg.solve(S_mat, Bmat)
    except np.linalg.LinAlgError:
        J = np.eye(3)

    dt_j = t_now - cache.t_prev
    if dt_j > 1e-6:
        dJ = (J - cache.J_prev) / dt_j
    else:
        dJ = np.zeros((3, 3))
    cache.J_prev = J.copy()
    cache.t_prev = t_now
    return J, dJ


def dyn_from_jacobian(J, dJ, q, m_pt):
    """Build M_hat, C_hat, tau_g for a given moving-plate mass m_pt.

    The arm gravity term (M_A) is a property of the robot itself and is the
    same for controller and plant; only the plate mass m_pt differs between
    the nominal (controller) and true (plant) models.
    """
    M_hat = I_A_VAL * np.eye(3) + m_pt * (J.T @ J)
    C_hat = m_pt * (J.T @ dJ)
    tau_g = np.empty(3)
    for i in range(3):
        tau_g_endeff = J[2, i] * m_pt * GRAVITY
        tau_g_arm = -(M_A * L_C * GRAVITY * math.cos(q[i]))
        tau_g[i] = tau_g_endeff + tau_g_arm
    return M_hat, C_hat, tau_g


# =====================================================================
# IK & FK (mm/degrees) - for setpoint & Cartesian logging (from deploy)
# =====================================================================
def inverse_kinematics_mm(x0, y0, z0):
    def calc_angle_yz(x0, y0, z0):
        y1 = -0.5 * 0.57735 * f
        y0 -= 0.5 * 0.57735 * ed
        a = (x0**2 + y0**2 + z0**2 + rf**2 - re**2 - y1**2) / (2 * z0)
        b = (y1 - y0) / z0
        d = -(a + b * y1)**2 + rf * (b**2 * rf + rf)
        if d < 0:
            return 0.0
        yj = (y1 - a * b - math.sqrt(d)) / (b**2 + 1)
        zj = a + b * yj
        return math.degrees(math.atan(-zj / (y1 - yj)))

    theta1 = calc_angle_yz(x0, y0, z0)
    theta2 = calc_angle_yz(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0)
    theta3 = calc_angle_yz(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0)
    return theta1, theta2, theta3


def delta_fwd_mm(a1_deg, a2_deg, a3_deg):
    """FK (mm) from angles (degrees). Ported from delta_calc_fwd_*."""
    t = (f - ed) * tan30 / 2
    dtr = pi / 180.0
    a1, a2, a3 = a1_deg*dtr, a2_deg*dtr, a3_deg*dtr

    y1 = -(t + rf * math.cos(a1)); z1 = -rf * math.sin(a1)
    y2 = (t + rf * math.cos(a2)) * sin30; x2 = y2 * tan60; z2 = -rf * math.sin(a2)
    y3 = (t + rf * math.cos(a3)) * sin30; x3 = -y3 * tan60; z3 = -rf * math.sin(a3)

    dnm = (y2 - y1) * x3 - (y3 - y1) * x2
    w1 = y1**2 + z1**2
    w2 = x2**2 + y2**2 + z2**2
    w3 = x3**2 + y3**2 + z3**2
    a1c = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1c = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0
    a2c = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2c = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0
    A = a1c**2 + a2c**2 + dnm**2
    B = 2 * (a1c * b1c + a2c * (b2c - y1 * dnm) - z1 * dnm**2)
    C = (b2c - y1 * dnm) ** 2 + b1c**2 + dnm**2 * (z1**2 - re**2)
    d = B**2 - 4.0 * A * C
    if d < 0 or abs(dnm) < 1e-9:
        return None, None, None
    z = -0.5 * (B + math.sqrt(d)) / A
    x = (a1c * z + b1c) / dnm
    y = (a2c * z + b2c) / dnm
    return x, y, z


# =====================================================================
# MAIN SIMULATION
# =====================================================================
def main():
    global USE_NEUROFUZZY
    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"[PAYLOAD] true plate mass = {M_PT_TRUE:.3f} kg "
          f"(controller assumes {M_PT_NOM:.3f} kg, payload = {PAYLOADS:.3f} kg unknown)")

    # --- setup mu predictor ---
    if USE_NEUROFUZZY:
        try:
            predict_mu = load_neurofuzzy()
            print("[INFO] Using neuro-fuzzy model for mu prediction.")
        except Exception as e:
            print(f"[WARN] Failed to load model ({e}). Using constant mu = {FALLBACK_MU}.")
            USE_NEUROFUZZY = False
    if not USE_NEUROFUZZY:
        def predict_mu(e_rad, de_rad):
            return FALLBACK_MU

    traj_time = TOTAL_TRAJECTORY_STEPS / float(SAMPLE_RATE)
    t_impulse = IMPULSE_TIME if IMPULSE_TIME is not None else (S1_TIME if IMPULSE_SCENARIO == 1 else S2_TIME)
    total_time = max(traj_time, t_impulse + IMPULSE_WIDTH + RECOVERY_S)   # ensure pulse + recovery are captured
    n_steps = int(round(total_time / DT))
    print(f"[SIM] scenario={IMPULSE_SCENARIO}  impulse@{t_impulse:.3f}s  width={IMPULSE_WIDTH*1000:.0f}ms  "
          f"gain={IMPULSE_GAIN}  traj_time={traj_time:.3f}s  total_time={total_time:.3f}s  steps={n_steps}  dt={DT}s")

    # --- initialize state at starting trajectory point (index 0) ---
    X0, Y0, Z0, _ = get_trajectory_sample(0)
    qd0_deg = np.array(inverse_kinematics_mm(X0, Y0, Z0))
    q  = np.radians(qd0_deg).astype(float)   # start on trajectory (initial error ~0)
    dq = np.zeros(3)
    i_curr = np.zeros(3)

    # inner current PID state
    curI = np.zeros(3)
    prevCurrErr = np.zeros(3)

    cache = DynCache()
    qd_prev = q.copy()
    dqd_prev = np.zeros(3)

    rows = []
    impulse_done = False
    for step in range(n_steps):
        t = step * DT

        # --- setpoint from trajectory ---
        idx = get_trajectory_index(t)
        if idx < 0:
            idx = 0
        if idx >= TOTAL_TRAJECTORY_STEPS:
            idx = TOTAL_TRAJECTORY_STEPS - 1
        Xs, Ys, Zs, _ = get_trajectory_sample(idx)
        qd_deg = np.array(inverse_kinematics_mm(Xs, Ys, Zs))
        qd = np.radians(qd_deg)

        # feed-forward (optional)
        if ENABLE_FEEDFORWARD:
            dqd = (qd - qd_prev) / DT
            ddqd = (dqd - dqd_prev) / DT
        else:
            dqd = np.zeros(3)
            ddqd = np.zeros(3)

        # --- error & derivative ---
        e = qd - q
        ed_err = dqd - dq

        # --- mu from neuro-fuzzy (per joint) ---
        mu = np.array([max(predict_mu(e[j], ed_err[j]), 1e-6) for j in range(3)])

        # --- Jacobian: computed ONCE per step so the numerical dJ/dt cache
        #     stays correct, then reused for both the nominal and true mass. ---
        J, dJ = compute_jacobian(t, q, cache)

        # --- CONTROL law uses the NOMINAL plate mass (payload is UNKNOWN to it) ---
        M_hat_c, C_hat_c, tau_g_c = dyn_from_jacobian(J, dJ, q, M_PT_NOM)
        sigma = c_manifold * e + ed_err
        acc_cmd = ddqd + c_manifold * ed_err + sigma / mu
        tau_ctrl = M_hat_c @ acc_cmd + C_hat_c @ dq + tau_g_c

        # --- IMPULSE DISTURBANCE: rectangular pulse over IMPULSE_WIDTH; zero elsewhere ---
        if IMPULSE_WIDTH > 0.0:
            fired = (t_impulse <= t < t_impulse + IMPULSE_WIDTH)
        else:
            fired = abs(t - t_impulse) < 0.5 * DT
        tau_dist_plant = np.zeros(3)          # equivalent joint torque added to the PLANT (scenario 2)
        if fired and IMPULSE_SCENARIO == 1:
            # Scenario 1: apply the impulse as a torque disturbance ON THE PLANT.
            # Routing it through tau_ctrl -> target_current lets the 12 V / current
            # loop saturation swallow it (max current ~12/Ra amps), so the plant is
            # barely perturbed. Adding it to the plant torque makes it the real
            # disturbance the controller must reject.
            tau_dist_plant = tau_dist_plant + IMPULSE_GAIN * S1_TORQUE
        elif fired and IMPULSE_SCENARIO == 2:
            # Scenario 2 (Task 4): unknown end-effector force -> joint torques via J^T
            f_dir = S2_DIR / (np.linalg.norm(S2_DIR) + 1e-12)
            f_ee = IMPULSE_GAIN * S2_FORCE * f_dir
            tau_dist_plant = J.T @ f_ee
        if fired and not impulse_done:
            impulse_done = True
            if IMPULSE_SCENARIO == 1:
                print(f"[IMPULSE] fired at t={t:.3f}s  scenario 1: {IMPULSE_GAIN}*{S1_TORQUE} N.m "
                      f"onto the plant torque for {IMPULSE_WIDTH*1000:.0f} ms")
            else:
                print(f"[IMPULSE] fired at t={t:.3f}s  scenario 2: {IMPULSE_GAIN*S2_FORCE:.3f} N EE force "
                      f"for {IMPULSE_WIDTH*1000:.0f} ms")

        target_current = tau_ctrl / Kt

        # --- inner current PID -> PWM -> voltage (per joint) ---
        err_c = target_current - i_curr
        curI = np.clip(curI + curKi * err_c * DT, -I_TERM_LIM, I_TERM_LIM)
        derivc = curKd * (err_c - prevCurrErr) / DT
        pwm = np.clip(curKp * err_c + curI + derivc, -PWM_MAX, PWM_MAX)
        prevCurrErr = err_c
        V = (pwm / PWM_MAX) * VSUPPLY

        # --- electrical plant (1st-order exact update, stable for large dt) ---
        i_ss = (V - Ke * dq) / Ra
        decay = math.exp(-Ra / L_elec * DT)
        i_curr = i_ss + (i_curr - i_ss) * decay

        # --- PLANT uses the TRUE plate mass (real payload on the moving plate) ---
        M_hat_p, C_hat_p, tau_g_p = dyn_from_jacobian(J, dJ, q, M_PT_TRUE)
        # mechanical plant: M_hat_p qdd + C_hat_p dq + tau_g_p = Kt*i + tau_dist_plant
        rhs = Kt * i_curr - C_hat_p @ dq - tau_g_p + tau_dist_plant
        try:
            qdd = np.linalg.solve(M_hat_p, rhs)
        except np.linalg.LinAlgError:
            qdd = np.zeros(3)
        dq = dq + qdd * DT
        q = q + dq * DT

        qd_prev = qd.copy()
        dqd_prev = dqd.copy()

        # --- logging (decimated) ---
        if step % LOG_DECIM == 0:
            ang_deg = np.degrees(q)
            set_deg = qd_deg
            xa, ya, za = delta_fwd_mm(*ang_deg)
            xs, ys, zs = delta_fwd_mm(*set_deg)
            rows.append([
                round(t, 4), e[0], e[1], e[2], ed_err[0], ed_err[1], ed_err[2],
                mu[0], mu[1], mu[2],
                set_deg[0], set_deg[1], set_deg[2],
                ang_deg[0], ang_deg[1], ang_deg[2],
                Xs, Ys, Zs,
                xa, ya, za,
                xs, ys, zs,
                target_current[0], target_current[1], target_current[2],
                i_curr[0], i_curr[1], i_curr[2],
            ])

    # --- write CSV ---
    header = ["Time","e1","e2","e3","edot1","edot2","edot3",
              "mu1","mu2","mu3",
              "SetAngle1","SetAngle2","SetAngle3",
              "ActualAngle1","ActualAngle2","ActualAngle3",
              "X0","Y0","Z0",
              "Actual_X","Actual_Y","Actual_Z",
              "X_Set","Y_Set","Z_Set",
              "refCurr1","refCurr2","refCurr3",
              "ActualCurr1","ActualCurr2","ActualCurr3"]
    with open(OUT_CSV, "w", newline="") as fcsv:
        w = csv.writer(fcsv)
        w.writerow(header)
        w.writerows(rows)
    print(f"[OK] CSV written: {OUT_CSV}  ({len(rows)} rows)")

    # --- summarize tracking error ---
    arr = np.array(rows, dtype=float)
    e_cols = arr[:, 1:4]
    rmse = np.sqrt(np.mean(e_cols**2, axis=0))
    print(f"[RMSE joint error (rad)] q1={rmse[0]:.5f}  q2={rmse[1]:.5f}  q3={rmse[2]:.5f}")
    print(f"[RMSE joint error (deg)] q1={math.degrees(rmse[0]):.4f}  "
          f"q2={math.degrees(rmse[1]):.4f}  q3={math.degrees(rmse[2]):.4f}")

    # --- plotting ---
    try:
        plot_results(arr, t_impulse)
    except Exception as e:
        print(f"[WARN] Plot failed: {e}")


def plot_results(arr, t_impulse=None):
    import matplotlib
    if os.environ.get("MPLBACKEND") is None and not _display_available():
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa

    t = arr[:, 0]

    # ==========================================
    # 1. COMBINED PLOT (Preserved from original)
    # ==========================================
    fig = plt.figure(figsize=(14, 9))

    # joint angle tracking
    for j in range(3):
        ax = fig.add_subplot(3, 3, j + 1)
        ax.plot(t, arr[:, 10 + j], 'k--', lw=1.2, label='Set')
        ax.plot(t, arr[:, 13 + j], lw=1.6, label='Actual')
        ax.set_title(f"Joint {j+1} (deg)"); ax.grid(alpha=.3)
        if j == 0: ax.legend(fontsize=8)

    # mu per joint
    ax = fig.add_subplot(3, 3, 4)
    for j in range(3):
        ax.plot(t, arr[:, 7 + j], lw=1.2, label=f'mu{j+1}')
    ax.set_title("mu (neuro-fuzzy)"); ax.legend(fontsize=8); ax.grid(alpha=.3)

    # joint error
    ax = fig.add_subplot(3, 3, 5)
    for j in range(3):
        ax.plot(t, arr[:, 1 + j], lw=1.0, label=f'e{j+1}')
    if t_impulse is not None:
        ax.axvline(t_impulse, color='r', ls=':', lw=1.2, label='impulse')
    ax.set_title("Position Error (rad)"); ax.legend(fontsize=8); ax.grid(alpha=.3)

    # actual vs target current joint 1
    ax = fig.add_subplot(3, 3, 6)
    ax.plot(t, arr[:, 25], 'k--', lw=1.0, label='ref I1')
    ax.plot(t, arr[:, 28], lw=1.2, label='act I1')
    ax.set_title("Joint 1 Current (A)"); ax.legend(fontsize=8); ax.grid(alpha=.3)

    # Cartesian trajectory: set vs actual (XY)
    ax = fig.add_subplot(3, 3, 7)
    ax.plot(arr[:, 22], arr[:, 23], 'k--', lw=1.0, label='Set')
    ax.plot(arr[:, 19], arr[:, 20], lw=1.2, label='Actual')
    ax.set_title("XY Trajectory (mm)"); ax.set_aspect('equal', 'box'); ax.legend(fontsize=8); ax.grid(alpha=.3)

    # 3D trajectory
    ax = fig.add_subplot(3, 3, 8, projection='3d')
    ax.plot(arr[:, 22], arr[:, 23], arr[:, 24], 'k--', lw=1.0, label='Set')
    ax.plot(arr[:, 19], arr[:, 20], arr[:, 21], lw=1.2, label='Actual')
    ax.set_title("3D Trajectory (mm)"); ax.legend(fontsize=7)

    # Z against time
    ax = fig.add_subplot(3, 3, 9)
    ax.plot(t, arr[:, 24], 'k--', lw=1.0, label='Z set')
    ax.plot(t, arr[:, 21], lw=1.2, label='Z act')
    ax.set_title("Z(t) (mm)"); ax.legend(fontsize=8); ax.grid(alpha=.3)

    fig.suptitle("Impulse Simulation - Neuro-Fuzzy Synergetic", fontweight='bold')
    fig.tight_layout()
    fig.savefig(OUT_FIG, dpi=130)
    print(f"[OK] Combined graph saved: {OUT_FIG}")

    # ==========================================
    # 2. INDIVIDUAL PLOTS (Saved to 'result' folder)
    # ==========================================
    RESULT_DIR = str(_root_dir / "results" / "controller_sim" / "neuro_fuzzy_it2_nfsc")
    os.makedirs(RESULT_DIR, exist_ok=True)

    # Individual Joint Angle Tracking Plots
    for j in range(3):
        fig_indiv, ax_indiv = plt.subplots()
        ax_indiv.plot(t, arr[:, 10 + j], 'k--', lw=1.2, label='Set')
        ax_indiv.plot(t, arr[:, 13 + j], lw=1.6, label='Actual')
        ax_indiv.set_title(f"Joint {j+1} Tracking (deg)")
        ax_indiv.set_xlabel("Time (s)")
        ax_indiv.set_ylabel("Angle (deg)")
        ax_indiv.grid(alpha=.3)
        ax_indiv.legend(fontsize=8)
        fig_indiv.savefig(os.path.join(RESULT_DIR, f"joint_{j+1}_tracking.png"))
        plt.close(fig_indiv)

    # ==============================================================
    # 3. NEW PLOT: ALL JOINTS COMBINED (Shared X and Y)
    # ==============================================================
    fig_all, ax_all = plt.subplots(figsize=(10, 6))
    colors = ['r', 'g', 'b'] # Red (Joint 1), Green (Joint 2), Blue (Joint 3)

    for j in range(3):
        # Garis putus-putus untuk nilai Set (target)
        ax_all.plot(t, arr[:, 10 + j], '--', color=colors[j], alpha=0.6, lw=1.5, label=f'Set J{j+1}')
        # Garis lurus untuk nilai Actual (aktual)
        ax_all.plot(t, arr[:, 13 + j], '-', color=colors[j], lw=2.0, label=f'Act J{j+1}')

    ax_all.set_title("All Joints Tracking (deg) - Set vs Actual")
    ax_all.set_xlabel("Time (s)")
    ax_all.set_ylabel("Angle (deg)")
    ax_all.legend(fontsize=8, ncol=3) # Mengatur legenda menjadi 3 kolom agar rapi
    ax_all.grid(alpha=0.3)

    combined_joint_path = os.path.join(RESULT_DIR, "joint_all_tracking.png")
    fig_all.savefig(combined_joint_path, dpi=130)
    plt.close(fig_all)
    print(f"[OK] Gabungan plot Joint 1, 2, 3 disimpan: {combined_joint_path}")
    # ==============================================================

    # Mu Plot
    fig_indiv, ax_indiv = plt.subplots()
    for j in range(3):
        ax_indiv.plot(t, arr[:, 7 + j], lw=1.2, label=f'mu{j+1}')
    ax_indiv.set_title("Mu (Neuro-Fuzzy)")
    ax_indiv.legend(fontsize=8)
    ax_indiv.grid(alpha=.3)
    fig_indiv.savefig(os.path.join(RESULT_DIR, "mu_neurofuzzy.png"))
    plt.close(fig_indiv)

    # Position Error Plot
    fig_indiv, ax_indiv = plt.subplots()
    for j in range(3):
        ax_indiv.plot(t, arr[:, 1 + j], lw=1.0, label=f'e{j+1}')
    if t_impulse is not None:
        ax_indiv.axvline(t_impulse, color='r', ls=':', lw=1.2, label='impulse')
    ax_indiv.set_title("Position Error (rad)")
    ax_indiv.legend(fontsize=8)
    ax_indiv.grid(alpha=.3)
    fig_indiv.savefig(os.path.join(RESULT_DIR, "position_error.png"))
    plt.close(fig_indiv)

    # Joint 1 Current Plot
    fig_indiv, ax_indiv = plt.subplots()
    ax_indiv.plot(t, arr[:, 25], 'k--', lw=1.0, label='ref I1')
    ax_indiv.plot(t, arr[:, 28], lw=1.2, label='act I1')
    ax_indiv.set_title("Joint 1 Current (A)")
    ax_indiv.legend(fontsize=8)
    ax_indiv.grid(alpha=.3)
    fig_indiv.savefig(os.path.join(RESULT_DIR, "current_joint1.png"))
    plt.close(fig_indiv)

    # XY Trajectory Plot
    fig_indiv, ax_indiv = plt.subplots()
    ax_indiv.plot(arr[:, 22], arr[:, 23], 'k--', lw=1.0, label='Set')
    ax_indiv.plot(arr[:, 19], arr[:, 20], lw=1.2, label='Actual')
    ax_indiv.set_title("XY Trajectory (mm)")
    ax_indiv.set_aspect('equal', 'box')
    ax_indiv.legend(fontsize=8)
    ax_indiv.grid(alpha=.3)
    fig_indiv.savefig(os.path.join(RESULT_DIR, "xy_trajectory.png"))
    plt.close(fig_indiv)

    # 3D Trajectory Plot
    fig_indiv = plt.figure()
    ax_indiv = fig_indiv.add_subplot(111, projection='3d')
    ax_indiv.plot(arr[:, 22], arr[:, 23], arr[:, 24], 'k--', lw=1.0, label='Set')
    ax_indiv.plot(arr[:, 19], arr[:, 20], arr[:, 21], lw=1.2, label='Actual')
    ax_indiv.set_title("3D Trajectory (mm)")
    ax_indiv.legend(fontsize=7)
    fig_indiv.savefig(os.path.join(RESULT_DIR, "3d_trajectory.png"))
    plt.close(fig_indiv)

    # Z against time Plot
    fig_indiv, ax_indiv = plt.subplots()
    ax_indiv.plot(t, arr[:, 24], 'k--', lw=1.0, label='Z set')
    ax_indiv.plot(t, arr[:, 21], lw=1.2, label='Z act')
    ax_indiv.set_title("Z(t) (mm)")
    ax_indiv.legend(fontsize=8)
    ax_indiv.grid(alpha=.3)
    fig_indiv.savefig(os.path.join(RESULT_DIR, "z_over_time.png"))
    plt.close(fig_indiv)

    print(f"[OK] Individual plots successfully saved to '{RESULT_DIR}' folder.")

    try:
        plt.show()
    except Exception:
        pass


def _display_available():
    return bool(os.environ.get("DISPLAY")) or os.name == "nt"


if __name__ == "__main__":
    main()