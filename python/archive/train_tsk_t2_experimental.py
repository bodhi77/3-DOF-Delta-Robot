#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from datetime import datetime
from pathlib import Path

import joblib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pyit2fls import IT2TSK_ML, IT2FS_Gaussian_UncertMean

# =====================================================================
# 1. IMPLEMENTASI OSTRA (One-Step Type Reducer Algorithm)
# =====================================================================
def ostra_reduction(wi, wi_bar, fi):
    # Print titik kecil untuk indikator progress training
    print(".", end="", flush=True)
    wi = np.asarray(wi, dtype=float).ravel()
    wi_bar = np.asarray(wi_bar, dtype=float).ravel()
    fi = np.asarray(fi, dtype=float).ravel()
    
    if not (wi.shape == wi_bar.shape == fi.shape):
        raise ValueError("wi, wi_bar, and fi must have the same shape.")
    
    idx = np.argsort(wi)
    wn = wi[idx]
    wn_bar = wi_bar[idx]
    
    w_avg = 0.5 * (wn + wn_bar)
    denom = np.sum(w_avg)
    if denom <= 0:
        return float(np.mean(fi))
        
    y_ref = np.sum(w_avg * wn) / denom
    y_bar_ref = np.sum(w_avg * wn_bar) / denom
    
    l = np.searchsorted(wn, y_ref, side="right") - 1
    r = np.searchsorted(wn, y_bar_ref, side="right") - 1
    l = int(np.clip(l, -1, len(wn) - 1))
    r = int(np.clip(r, -1, len(wn) - 1))
    
    X_low = np.concatenate((wn[:l + 1], wn_bar[l + 1:])).astype(float)
    X_high = np.concatenate((wn_bar[:r + 1], wn[r + 1:])).astype(float)
    
    sum_low = np.sum(X_low)
    sum_high = np.sum(X_high)
    
    if sum_low > 0:
        X_low /= sum_low
    else:
        X_low = np.full_like(X_low, 1.0 / len(X_low))
        
    if sum_high > 0:
        X_high /= sum_high
    else:
        X_high = np.full_like(X_high, 1.0 / len(X_high))
    
    X_final_sorted = 0.5 * (X_low + X_high)
    X_final = np.zeros_like(wi)
    X_final[idx] = X_final_sorted
    
    return float(np.dot(X_final, fi))

# =====================================================================
# 2. FUNGSI PENYIMPANAN ARTIFAK
# =====================================================================
def save_trained_artifacts(model, metadata, output_dir="trained_artifacts"):
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    model_path = output_path / "synergetic_it2TSK_ostra_ga.joblib"
    metadata_path = output_path / "synergetic_it2TSK_ostra_ga_metadata.json"

    joblib.dump(model, model_path)
    with metadata_path.open("w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)

    return model_path, metadata_path

# =====================================================================
# 3. PIPELINE PELATIHAN UTAMA
# =====================================================================
def main():
    # np.random.seed(88) # Aktifkan jika butuh hasil yang repeatable
    
    NAMA_FILE_CSV  = "dataset_synergetic_degrees.csv" 
    HEADER_INPUT_1 = "e_deg"
    HEADER_INPUT_2 = "de_deg"
    HEADER_OUTPUT  = "optimal_mu"
    
    print(f"[INFO] Memuat dataset '{NAMA_FILE_CSV}'...", flush=True)
    try:
        df = pd.read_csv(NAMA_FILE_CSV) 
        X_data = df[[HEADER_INPUT_1, HEADER_INPUT_2]].values
        y_data = df[HEADER_OUTPUT].values
    except FileNotFoundError:
        print("[WARNING] File tidak ditemukan. Menggunakan data dummy.", flush=True)
        X_data = np.random.uniform(-180, 180, (200, 2))
        y_data = np.random.uniform(0.001, 0.1, 200)

    # =================================================================
    # DATA PREPROCESSING (CLEANING & SCALING)
    # =================================================================
    # 1. Clipping: Paksa data simulasi yang overshoot balik ke batas fisik [-180, 180]
    X_data_cleaned = np.clip(X_data, -180.0, 180.0)

    # 2. Min-Max Scaling: Geser rentang [-180, 180] ke [0, 1]
    # Rumus: (x + 180) / 360
    X_data_scaled = (X_data_cleaned + 180.0) / 360.0
    
    # 3. Final Safety: Pastikan tidak ada floating point error yang keluar dari 0-1
    X_data_scaled = np.clip(X_data_scaled, 0.0, 1.0)
    # =================================================================
    
    split_idx = int(0.8 * len(X_data_scaled))
    X_train, X_test = X_data_scaled[:split_idx], X_data_scaled[split_idx:]
    y_train, y_test = y_data[:split_idx], y_data[split_idx:]

    n_input = 2
    m_rules = 4    
    
    # Bounds GA dipasang positif (0, 1) agar koefisien TSK selalu positif.
    # Ditambah input yang sudah dipositifkan [0, 1], maka output Mu PASTI POSITIF.
    bounds = (0.0, 1.0) 
    ga_full_params = [50, 200, 20, 10, 0.05] 
    
    model_utama = IT2TSK_ML(
        n_input, m_rules, IT2FS_Gaussian_UncertMean, bounds, 
        algorithm="GA", algorithm_params=ga_full_params
    )
    model_utama.type_reduction = ostra_reduction

    # ---------------------------------------------------------
    # FASE 1: VALIDASI AWAL & SMOKE TEST
    # ---------------------------------------------------------
    print("[CHECK] Validasi awal...", flush=True)
    assert X_train.shape[0] == y_train.shape[0], "Jumlah data mismatch"
    # Cek apakah scaling berhasil (tidak akan error lagi karena sudah di-clip)
    assert np.min(X_train) >= 0 and np.max(X_train) <= 1, "Scaling gagal melewati batas 0-1"

    print("[CHECK] Smoke test...", flush=True)
    model_test = IT2TSK_ML(n_input, m_rules, IT2FS_Gaussian_UncertMean, bounds, 
                           algorithm="GA", algorithm_params=[50, 5, 20, 10, 0.05])
    model_test.type_reduction = ostra_reduction
    model_test.fit(X_train[:10], y_train[:10])
    y_smoke = model_test.score(X_train[:5])
    assert np.isfinite(y_smoke).all(), "Output mengandung NaN/Inf"

    print("[CHECK] Lolos Preprocessing. Memulai Full Training...", flush=True)

    # ---------------------------------------------------------
    # FASE 2: FULL TRAINING
    # ---------------------------------------------------------
    model_utama.fit(X_train, y_train)
    
    y_pred_train = model_utama.score(X_train)
    y_pred_test = model_utama.score(X_test)

    train_rmse = np.sqrt(np.mean((y_train - y_pred_train) ** 2))
    test_rmse = np.sqrt(np.mean((y_test - y_pred_test) ** 2))

    print("\n[INFO] Training Selesai.")
    print(f"Train RMSE = {train_rmse:.6f}")
    print(f"Test  RMSE = {test_rmse:.6f}")

    # ---------------------------------------------------------
    # FASE 3: PENYIMPANAN
    # ---------------------------------------------------------
    metadata = {
        "model_name": "Synergetic_IT2TSK_PositiveMu",
        "preprocessing": "Clip(-180, 180) -> Scale(0, 1)",
        "input_transform_formula": "(x + 180) / 360",
        "bounds_used": list(bounds),
        "metrics": {"train_rmse": float(train_rmse), "test_rmse": float(test_rmse)},
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    save_trained_artifacts(model_utama, metadata)

    # ---------------------------------------------------------
    # FASE 4: VISUALISASI
    # ---------------------------------------------------------
    plt.figure(figsize=(12, 6))
    plt.plot(y_train, label="Target (Train)", color='blue', alpha=0.5)
    plt.plot(y_pred_train, label="Prediksi (Train)", color='orange', linestyle="--")
    
    # Geser index test agar nyambung di grafik
    test_range = np.arange(len(y_train), len(y_train) + len(y_test))
    plt.plot(test_range, y_test, label="Target (Test)", color='green', alpha=0.5)
    plt.plot(test_range, y_pred_test, label="Prediksi (Test)", color='red', linestyle="-.")
    
    plt.axhline(y=0, color='black', linewidth=1.5, label="Batas Nol (Output)")
    plt.title("Hasil Identifikasi TSK: Input Scaled [0, 1] & Output Non-Negatif")
    plt.xlabel("Index Sampel")
    plt.ylabel("Optimal Mu")
    plt.legend(loc='upper right')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("hasil_training_tsk_positif.png", dpi=300)
    plt.show()

if __name__ == "__main__":
    main()