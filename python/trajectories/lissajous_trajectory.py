import math

# =====================================================================
# CONFIGURATION
# =====================================================================
SAMPLE_RATE = 1000  # 1 kHz (sesuai dengan DT = 1e-3 di simulasi)
DURATION = 10.0     # Total durasi lintasan (detik)
TOTAL_TRAJECTORY_STEPS = int(DURATION * SAMPLE_RATE)

# Parameter Dimensi Angka 8 (dalam mm)
Z_CENTER = -200.0   # Ketinggian absolut end-effector
X_AMP = 40.0        # Lebar maksimal ke kiri-kanan (Total lebar = 80 mm)
Y_AMP = 60.0        # Tinggi maksimal ke atas-bawah (Total tinggi = 120 mm)

# Kecepatan putaran (Hz)
# 0.2 Hz artinya robot butuh waktu 5 detik untuk menyelesaikan 1 bentuk angka 8 penuh.
# Dengan durasi 10 detik, robot akan menggambar angka 8 persis 2 kali.
FREQUENCY = 0.2     

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
    
    # Sudut dasar
    theta = 2.0 * math.pi * FREQUENCY * t
    
    # Rumus Parametrik Angka 8 Vertikal
    # x menggunakan frekuensi ganda (sin(2*theta)) agar bolak-balik kiri-kanan lebih cepat
    # y menggunakan frekuensi tunggal (sin(theta)) untuk ayunan atas-bawah
    _x_traj[i] = X_AMP * math.sin(2.0 * theta)
    _y_traj[i] = Y_AMP * math.sin(theta)
    _z_traj[i] = Z_CENTER

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