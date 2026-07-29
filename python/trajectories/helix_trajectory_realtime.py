import numpy as np
import time

# Variabel global default
DURATION = 5               # Durasi lintasan helix (detik)
SAMPLE_RATE = 25            # Frekuensi sampel (Hz)
TOTAL_STEPS = int(DURATION * SAMPLE_RATE)

RADIUS = 50
Z_START = -184
Z_END = -350

# Posisi awal dan posisi awal helix
X_START, Y_START, Z_START = 0, 0, -184
X_HELIX_START, Y_HELIX_START, Z_HELIX_START = 0, 50, -184

# Durasi transisi ke titik awal helix
TRANSITION_DURATION = 1.5
TRANSITION_STEPS = int(TRANSITION_DURATION * SAMPLE_RATE)

# Durasi total termasuk transisi
TOTAL_DURATION = TRANSITION_DURATION + DURATION
TOTAL_TRAJECTORY_STEPS = TRANSITION_STEPS + TOTAL_STEPS


def generate_transition_trajectory():
    """Gerakan transisi dari posisi awal ke titik awal helix."""
    X = np.linspace(X_START, X_HELIX_START, TRANSITION_STEPS)
    Y = np.linspace(Y_START, Y_HELIX_START, TRANSITION_STEPS)
    Z = np.linspace(Z_START, Z_HELIX_START, TRANSITION_STEPS)
    time_step = 1.0 / SAMPLE_RATE
    return X, Y, Z, time_step


def generate_helix_trajectory():
    """Lintasan helix utama."""
    t = np.linspace(0, 8 * np.pi, TOTAL_STEPS)
    Y = RADIUS * np.cos(t)
    X = -RADIUS * np.sin(t)
    Z = np.linspace(Z_START, Z_END, TOTAL_STEPS)
    time_step = 1.0 / SAMPLE_RATE
    return X, Y, Z, time_step


# Precompute sampled trajectory points
_X_TRANS, _Y_TRANS, _Z_TRANS, _DT_TRANS = generate_transition_trajectory()
_X_HELIX, _Y_HELIX, _Z_HELIX, _DT_HELIX = generate_helix_trajectory()

TRAJ_X = np.concatenate([_X_TRANS, _X_HELIX])
TRAJ_Y = np.concatenate([_Y_TRANS, _Y_HELIX])
TRAJ_Z = np.concatenate([_Z_TRANS, _Z_HELIX])


def get_trajectory_index(elapsed_time):
    """
    Konversi waktu aktual ke indeks sample trajectory.
    Cocok untuk sistem real-time yang ingin tetap time-based.
    """
    idx = int(np.floor(max(elapsed_time, 0.0) * SAMPLE_RATE))
    return idx


def get_trajectory_sample(index):
    """Ambil satu titik trajectory berdasarkan indeks sample."""
    idx = int(np.clip(index, 0, TOTAL_TRAJECTORY_STEPS - 1))
    elapsed_time = idx / SAMPLE_RATE
    return float(TRAJ_X[idx]), float(TRAJ_Y[idx]), float(TRAJ_Z[idx]), float(elapsed_time)


def get_trajectory_at_time(elapsed_time):
    """
    Ambil titik trajectory langsung dari waktu aktual.
    Dipertahankan agar pola pemakaian sama seperti versi parabolic realtime.
    """
    idx = get_trajectory_index(elapsed_time)
    if idx >= TOTAL_TRAJECTORY_STEPS:
        idx = TOTAL_TRAJECTORY_STEPS - 1
    return get_trajectory_sample(idx)


def run_trajectory(start_time=None):
    """
    Generator kompatibilitas.
    Tetap time-based, tetapi berjalan berdasarkan sample trajectory yang sudah diprecompute.
    """
    if start_time is None:
        start_time = time.monotonic()

    for idx in range(TOTAL_TRAJECTORY_STEPS):
        target_time = start_time + idx / SAMPLE_RATE
        x, y, z, elapsed_time = get_trajectory_sample(idx)
        yield x, y, z, elapsed_time

        sleep_time = target_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
