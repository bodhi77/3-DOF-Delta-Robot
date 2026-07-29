from pathlib import Path
import pandas as pd
import numpy as np

_python_dir = Path(__file__).resolve().parent
while _python_dir.name != "python":
    _python_dir = _python_dir.parent
_root_dir = _python_dir.parent
_STAGING_DIR = _root_dir / "results" / "data_processing_staging"

def susutkan_csv(input_file, target_rows=160):
    # Buat nama file output (contoh: jadi "160_0helixFuzzySynergeticT1-SIM.csv")
    input_path = _STAGING_DIR / input_file
    output_path = _STAGING_DIR / ("160_" + input_file)

    print(f"Membaca {input_file}...")
    df = pd.read_csv(input_path)
    total_rows = len(df)

    if total_rows > target_rows:
        # Mengambil 160 titik indeks secara merata dari baris pertama hingga terakhir
        indices = np.round(np.linspace(0, total_rows - 1, target_rows)).astype(int)
        df_sampled = df.iloc[indices]

        # Simpan ke CSV baru
        df_sampled.to_csv(output_path, index=False)
        print(f"[BERHASIL] Data disusutkan dari {total_rows} menjadi {len(df_sampled)} baris.")
        print(f"Tersimpan sebagai: {output_path}\n")
    else:
        print(f"[INFO] Baris data hanya {total_rows}, tidak disusutkan.")
        df.to_csv(output_path, index=False)

# =========================================================
# MASUKKAN NAMA FILE CSV ANDA DI SINI
# =========================================================
daftar_file = [
    "0helixFuzzySynergeticT1-SIM.csv",
    "0helixSynergetic-SIM.csv",
    "0helixFuzzySynergeticT2-SIM.csv",
    "0helixNeuroIT2FS-SIM.csv",
    "0helixNeuroMamdaniT1-SIM.csv",
    "500helixFuzzySynergeticT1-SIM.csv",
    "500helixSynergetic-SIM.csv",
    "500helixFuzzySynergeticT2-SIM.csv",
    "500helixNeuroIT2FS-SIM.csv",
    "500helixNeuroMamdaniT1-SIM.csv",
    "1000helixFuzzySynergeticT1-SIM.csv",
    "1000helixSynergetic-SIM.csv",
    "1000helixFuzzySynergeticT2-SIM.csv",
    "1000helixNeuroIT2FS-SIM.csv",
    "1000helixNeuroMamdaniT1-SIM.csv",
    "impulse_s1_FuzzyT1-SIM.csv",
    "impulse_s1_FuzzyT2-SIM.csv",
    "impulse_s1_NeuroIT2FS-SIM.csv",
    "impulse_s1_NeuroMamdaniT1-SIM.csv",
    "impulse_s1_StaticMu-SIM.csv",
    "impulse_s2_FuzzyT1-SIM.csv",
    "impulse_s2_FuzzyT2-SIM.csv",
    "impulse_s2_NeuroIT2FS-SIM.csv",
    "impulse_s2_NeuroMamdaniT1-SIM.csv",
    "impulse_s2_StaticMu-SIM.csv"
]

for file_csv in daftar_file:
    try:
        susutkan_csv(file_csv, target_rows=160)
    except FileNotFoundError:
        print(f"[ERROR] File {file_csv} tidak ditemukan di folder ini.\n")