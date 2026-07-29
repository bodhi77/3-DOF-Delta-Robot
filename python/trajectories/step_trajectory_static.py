# stepTraj_static.py  --  STEP REFERENCE for IMPULSE Scenario 1
# =====================================================================
# Same step idea as your original stepTraj: start at the stable standby
# (-200), then STEP down to the operating point (-250). The ONLY change:
# STEP_TIME is moved EARLY (0.2 s) so the step transient settles BEFORE
# the impulse at t = 1 s and does not overlap it. (The old stepTraj
# stepped at t = 1 s, right on top of the impulse.)
#
# Same module interface as the original. Use for SCENARIO 1 only.
# =====================================================================

# =====================================================================
# CONFIGURATION
# =====================================================================
SAMPLE_RATE = 1000            # 1 kHz (matches DT = 1e-3 in the simulation)
DURATION    = 5.0             # total run time (s)
TOTAL_TRAJECTORY_STEPS = int(DURATION * SAMPLE_RATE)

# Pure Z (X = Y = 0)
X_START, Y_START, Z_START    = 0.0, 0.0, -200.0   # stable standby; robot INITIALIZES here
X_TARGET, Y_TARGET, Z_TARGET = 0.0, 0.0, -250.0   # operating point held during/after the impulse
STEP_TIME = 0.2   # (s) when the robot steps down. MUST be well before the impulse (t = 1 s)
                  # so the step transient settles first. Move it earlier if it hasn't settled by t = 1.

# =====================================================================
# PRE-COMPUTE TRAJECTORY ARRAYS (O(1) lookup)
# =====================================================================
_x_traj = [0.0] * TOTAL_TRAJECTORY_STEPS
_y_traj = [0.0] * TOTAL_TRAJECTORY_STEPS
_z_traj = [0.0] * TOTAL_TRAJECTORY_STEPS
_t_traj = [0.0] * TOTAL_TRAJECTORY_STEPS

for i in range(TOTAL_TRAJECTORY_STEPS):
    t = i / float(SAMPLE_RATE)
    _t_traj[i] = t
    # Hold at the standby until STEP_TIME, then jump to the operating point.
    if t < STEP_TIME:
        _x_traj[i] = X_START
        _y_traj[i] = Y_START
        _z_traj[i] = Z_START
    else:
        _x_traj[i] = X_TARGET
        _y_traj[i] = Y_TARGET
        _z_traj[i] = Z_TARGET

# =====================================================================
# REQUIRED INTERFACE FUNCTIONS (identical signatures to the original)
# =====================================================================
def get_trajectory_sample(index):
    """Return (X, Y, Z, elapsed_time) by index. Out-of-bounds safe."""
    if index < 0:
        index = 0
    elif index >= TOTAL_TRAJECTORY_STEPS:
        index = TOTAL_TRAJECTORY_STEPS - 1
    return _x_traj[index], _y_traj[index], _z_traj[index], _t_traj[index]


def get_trajectory_index(elapsed_time):
    """Convert elapsed time (s) to the nearest trajectory index."""
    idx = int(round(elapsed_time * SAMPLE_RATE))
    if idx < 0:
        idx = 0
    elif idx >= TOTAL_TRAJECTORY_STEPS:
        idx = TOTAL_TRAJECTORY_STEPS - 1
    return idx