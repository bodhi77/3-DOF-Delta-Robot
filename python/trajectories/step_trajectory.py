# stepTraj.py

# =====================================================================
# CONFIGURATION
# =====================================================================
SAMPLE_RATE = 1000  # 1 kHz (sesuai dengan DT = 1e-3 di simulasi)
DURATION = 5.0      # Total durasi lintasan (detik)
TOTAL_TRAJECTORY_STEPS = int(DURATION * SAMPLE_RATE)

# Parameter Posisi Step (dalam mm)
# MURNI SUMBU Z: X dan Y selalu 0.0
X_START, Y_START, Z_START = 0.0, 0.0, -200.0  # Posisi standby awal (atas)
X_TARGET, Y_TARGET, Z_TARGET = 0.0, 0.0, -250.0 # Posisi tujuan step (turun ke bawah)
STEP_TIME = 1.0  # Waktu (detik) di mana robot menghentak turun

# =====================================================================
# PRE-COMPUTE TRAJECTORY ARRAYS (O(1) Lookup)
# =====================================================================
_x_traj = [0.0] * TOTAL_TRAJECTORY_STEPS
_y_traj = [0.0] * TOTAL_TRAJECTORY_STEPS
_z_traj = [0.0] * TOTAL_TRAJECTORY_STEPS
_t_traj = [0.0] * TOTAL_TRAJECTORY_STEPS

for i in range(TOTAL_TRAJECTORY_STEPS):
    t = i / float(SAMPLE_RATE)
    _t_traj[i] = t
    
    # Logika Step Response: Jika waktu belum mencapai STEP_TIME, diam di posisi awal.
    # Jika sudah lewat, langsung melompat (hentak) ke posisi target.
    if t < STEP_TIME:
        _x_traj[i] = X_START
        _y_traj[i] = Y_START
        _z_traj[i] = Z_START
    else:
        _x_traj[i] = X_TARGET
        _y_traj[i] = Y_TARGET
        _z_traj[i] = Z_TARGET

# =====================================================================
# REQUIRED INTERFACE FUNCTIONS
# =====================================================================
def get_trajectory_sample(index):
    """
    Mengembalikan (X, Y, Z, elapsed_time) berdasarkan indeks.
    Aman dari Out-of-Bounds.
    """
    if index < 0:
        index = 0
    elif index >= TOTAL_TRAJECTORY_STEPS:
        index = TOTAL_TRAJECTORY_STEPS - 1
        
    return _x_traj[index], _y_traj[index], _z_traj[index], _t_traj[index]


def get_trajectory_index(elapsed_time):
    """
    Mengonversi waktu berjalan (detik) menjadi indeks lintasan terdekat.
    """
    idx = int(round(elapsed_time * SAMPLE_RATE))
    
    if idx < 0:
        idx = 0
    elif idx >= TOTAL_TRAJECTORY_STEPS:
        idx = TOTAL_TRAJECTORY_STEPS - 1
        
    return idx