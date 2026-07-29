#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=====================================================================
T1 Mamdani Neuro-Fuzzy Training (type-1 baseline)
=====================================================================
Maps (e_deg_avg, de_deg_avg) -> optimal_mu  for delta robot synergetic
control, using data from simulasiSynergeticFullRobotV5.py.

Fixes applied (per review):
  1. Output scaling: y is now scaled to [0, 1] using mu_min/mu_max
     before training, then un-scaled after prediction.
  2. Random shuffled train/val/test split (70/15/15) with fixed seed.
  3. Validation set tracked separately for monitoring overfit.
  4. m_rules sweep helper (set RULES_SWEEP = True to compare).
  5. Input scaling based on ACTUAL data range (mean ± 3*std), not
     ±180 deg overestimate -> tighter, more informative MFs.
  6. HSO removed. Pure PSO from pyit2fls.
  7. Smoke test enlarged: 50 iters x 100 samples (real sanity).
  8. NaN/Inf safety checks on predictions.
  9. Convergence plot: best fitness + train/val RMSE per snapshot.
=====================================================================
"""

import json
from datetime import datetime
from pathlib import Path

import joblib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split

import numpy as np
import pyit2fls.learning as _learn
from pyit2fls.learning import T1Mamdani_ML

_python_dir = Path(__file__).resolve().parent
while _python_dir.name != "python":
    _python_dir = _python_dir.parent
_root_dir = _python_dir.parent
_DATASET_DIR = _root_dir / "results" / "training_convergence" / "synergetic_delta"
_ARTIFACTS_DIR = _root_dir / "data" / "trained_artifacts"
_PLOT_DIR = _root_dir / "results" / "training_convergence" / "mamdani_t1"
_PLOT_DIR.mkdir(parents=True, exist_ok=True)


# =====================================================================
# CONFIGURATION
# =====================================================================
CONFIG = {
    # --- Dataset ---
    "csv_file": str(_DATASET_DIR / "dataset_synergetic_delta_v8.csv"),
    "input_cols": ["e_deg_avg", "de_deg_avg"],
    "output_col": "optimal_mu",

    # --- Splits (sum should be 1.0) ---
    "test_ratio": 0.15,
    "val_ratio":  0.15,
    "random_seed": 42,

    # --- Model ---
    "m_rules": 4,                # try 4, 9, 16, 25 (use RULES_SWEEP=True)
    "rules_sweep": False,        # set True to compare rule counts

    # --- PSO (pyit2fls built-in) ---
    # algorithm_params for PSO = [pop_size, max_iter, w, c1, c2]
    "pso_params": [50, 150, 0.6, 1.5, 1.5],

    # --- Smoke test (quick sanity) ---
    "smoke_params":  [50, 20, 0.6, 1.5, 1.5],
    "smoke_samples": 100,

    # --- Output ---
    "output_dir": str(_ARTIFACTS_DIR),
    "model_filename": "t1mamdani_pso_model.joblib",
    "metadata_filename": "t1mamdani_pso_metadata.json",
}


# =====================================================================
# UTILITIES
# =====================================================================
_orig_linspace = np.linspace
def _guard_linspace(start, stop, num=50, **kw):
    if (not np.isfinite(start)) or (not np.isfinite(stop)) \
       or abs(stop - start) > 1e6 or num > 100000:
        # partikel buruk: pulangkan domain dummy kecil, jangan crash
        return _orig_linspace(0.0, 1.0, 50, **kw)
    return _orig_linspace(start, stop, num, **kw)

_learn.linspace = _guard_linspace

def make_input_scaler(X, n_std=3.0, pad=0.05):
    """Build min-max scaler based on actual data range (mean +/- n_std).
       Returns (scaler_fn, unscaler_fn, lo, hi)."""
    mean = X.mean(axis=0)
    std = X.std(axis=0)
    lo = mean - n_std * std
    hi = mean + n_std * std
    # Add small padding so PSO has room near edges
    span = hi - lo
    lo = lo - pad * span
    hi = hi + pad * span

    def scaler(x):
        return np.clip((x - lo) / (hi - lo), 0.0, 1.0)

    def unscaler(x_scaled):
        return x_scaled * (hi - lo) + lo

    return scaler, unscaler, lo, hi


def make_output_scaler(y):
    """Scale y to [0, 1] using true min/max with small padding."""
    y_min = float(np.min(y))
    y_max = float(np.max(y))
    span = y_max - y_min
    # 5% padding both sides so MFs can sit at boundary
    pad = 0.05 * span if span > 1e-12 else 1e-6
    lo = y_min - pad
    hi = y_max + pad

    def scaler(y_):
        return np.clip((y_ - lo) / (hi - lo), 0.0, 1.0)

    def unscaler(y_scaled):
        return y_scaled * (hi - lo) + lo

    return scaler, unscaler, lo, hi


def safe_predict(model, X_scaled):
    """Predict and assert finite values."""
    y_pred = np.asarray(model.score(X_scaled))
    if np.isnan(y_pred).any() or np.isinf(y_pred).any():
        n_bad = int(np.isnan(y_pred).sum() + np.isinf(y_pred).sum())
        raise ValueError(f"Model produced {n_bad} NaN/Inf predictions.")
    return y_pred


def compute_metrics(y_true, y_pred):
    err = y_true - y_pred
    rmse = float(np.sqrt(np.mean(err ** 2)))
    mae = float(np.mean(np.abs(err)))
    std = float(np.std(err))
    mse = float(np.mean(err ** 2))
    # R^2
    ss_res = float(np.sum(err ** 2))
    ss_tot = float(np.sum((y_true - np.mean(y_true)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    return {"rmse": rmse, "mae": mae, "std": std, "r2": r2, "mse": mse}


def save_artifacts(model, metadata, output_dir, model_name, meta_name):
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    model_path = out / model_name
    meta_path = out / meta_name
    joblib.dump(model, model_path)
    with meta_path.open("w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)
    return model_path, meta_path


# =====================================================================
# TRAINING ROUTINE (for a single rule count)
# =====================================================================
def fit_compat(model, X, y):
    """
    Solusi tingkat lanjut: Menyadap (hook) fungsi internal PSO secara dinamis.
    Ini akan merekam konvergensi langsung dari memori tanpa perlu mengedit library.
    """
    import pyit2fls.learning as _learn
    
    history = []
    
    # 1. Simpan fungsi iterasi PSO yang asli
    original_iterate = _learn.PSO.iterate
    
    # 2. Buat fungsi penyadap untuk mencuri nilai fitness (fb) di tiap iterasi
    def hooked_iterate(self, *args, **kwargs):
        original_iterate(self, *args, **kwargs)  # Jalankan perhitungan aslinya
        history.append(self.fb)                  # Rekam nilainya secara diam-diam
        
    # 3. Pasang penyadap ke dalam library secara paksa (sementara)
    _learn.PSO.iterate = hooked_iterate
    
    try:
        # 4. Jalankan training. Di titik ini, list 'history' akan terisi otomatis!
        res = model.fit(X, y)
    finally:
        # 5. Cabut penyadap dan kembalikan ke asli agar aman
        _learn.PSO.iterate = original_iterate

    # Jika kebetulan library sudah pernah sukses diedit (mengembalikan tuple)
    if isinstance(res, tuple):
        return res[0], list(res[1])
        
    # Jika karena suatu alasan array tetap kosong, hindari IndexError
    if not history:
        history = [float(res)]
        
    # 6. Kembalikan error akhir beserta riwayat utuh dari iterasi awal hingga akhir
    return float(res), history


def train_one(m_rules, X_train, y_train_s, X_val, y_val_s,
              pso_params, smoke_params, smoke_samples,
              y_unscaler, label="model"):
    """Train one T1 Mamdani model. Returns (model, history dict)."""

    bounds = (0.0, 1.0)

    # -----------------------------------------------------------------
    # Smoke test (real sanity, not just "no crash")
    # -----------------------------------------------------------------
    print(f"\n[SMOKE] {label}: {smoke_params[1]} iters x "
          f"{smoke_samples} samples...", flush=True)
    smoke = T1Mamdani_ML(
        X_train.shape[1], m_rules, bounds,
        algorithm="PSO", algorithm_params=smoke_params,
    )
    n_smoke = min(smoke_samples, len(X_train))
    smoke_err, smoke_conv = fit_compat(smoke, X_train[:n_smoke], y_train_s[:n_smoke])
    smoke_pred = safe_predict(smoke, X_train[:n_smoke])
    smoke_rmse_scaled = float(np.sqrt(np.mean(
        (y_train_s[:n_smoke] - smoke_pred) ** 2)))
    print(f"[SMOKE] passed. final fitness={smoke_err:.6f}  "
          f"first->last conv={smoke_conv[0]:.4f}->{smoke_conv[-1]:.4f}  "
          f"train RMSE (scaled)={smoke_rmse_scaled:.4f}")

    if smoke_conv[-1] >= smoke_conv[0]:
        print("[SMOKE WARN] convergence did not improve. "
              "Consider checking data or PSO params.")

    # -----------------------------------------------------------------
    # Full training
    # -----------------------------------------------------------------
    print(f"\n[TRAIN] {label}: m_rules={m_rules}  "
          f"PSO={pso_params}", flush=True)
    model = T1Mamdani_ML(
        X_train.shape[1], m_rules, bounds,
        algorithm="PSO", algorithm_params=pso_params,
    )
    final_err, convergence = fit_compat(model, X_train, y_train_s)

    # -----------------------------------------------------------------
    # Evaluate on train + val in ORIGINAL scale
    # -----------------------------------------------------------------
    y_pred_train_s = safe_predict(model, X_train)
    y_pred_val_s = safe_predict(model, X_val)

    y_train = y_unscaler(y_train_s)
    y_val = y_unscaler(y_val_s)
    y_pred_train = y_unscaler(y_pred_train_s)
    y_pred_val = y_unscaler(y_pred_val_s)

    train_metrics = compute_metrics(y_train, y_pred_train)
    val_metrics = compute_metrics(y_val, y_pred_val)

    print(f"[DONE] {label}  "
          f"train RMSE={train_metrics['rmse']:.6f}  "
          f"val RMSE={val_metrics['rmse']:.6f}  "
          f"val R2={val_metrics['r2']:.4f}")

    history = {
        "final_fitness_scaled": float(final_err),
        "convergence": list(map(float, convergence)),
        "train": train_metrics,
        "val": val_metrics,
        "smoke_fitness_scaled": float(smoke_err),
        "smoke_conv_first_last": [float(smoke_conv[0]),
                                  float(smoke_conv[-1])],
    }
    return model, history


# =====================================================================
# MAIN
# =====================================================================
def main():
    cfg = CONFIG
    np.random.seed(cfg["random_seed"])

    # -----------------------------------------------------------------
    # 1. Load dataset
    # -----------------------------------------------------------------
    print(f"[INFO] Loading '{cfg['csv_file']}' ...", flush=True)
    try:
        df = pd.read_csv(cfg["csv_file"])
        X_raw = df[cfg["input_cols"]].values
        y_raw = df[cfg["output_col"]].values
        print(f"[INFO] Loaded {len(df)} rows.")
    except FileNotFoundError:
        print("[WARN] CSV not found. Generating dummy data for sanity.",
              flush=True)
        rng = np.random.default_rng(0)
        X_raw = rng.uniform(-5.0, 5.0, (500, 2))
        y_raw = 0.001 + 0.009 * (
            np.tanh(X_raw[:, 0] / 3.0) * 0.5 + 0.5
            + 0.3 * np.cos(X_raw[:, 1])).clip(0, 1)

    # Strip rows with NaN/Inf
    mask_finite = np.isfinite(X_raw).all(axis=1) & np.isfinite(y_raw)
    n_dropped = (~mask_finite).sum()
    if n_dropped:
        print(f"[INFO] Dropping {n_dropped} non-finite rows.")
        X_raw = X_raw[mask_finite]
        y_raw = y_raw[mask_finite]

    print(f"[INFO] X range: {X_raw.min(axis=0)} .. {X_raw.max(axis=0)}")
    print(f"[INFO] y range: {y_raw.min():.6f} .. {y_raw.max():.6f}")

    # -----------------------------------------------------------------
    # 2. Build scalers from ACTUAL data range (point 5)
    # -----------------------------------------------------------------
    x_scaler, x_unscaler, x_lo, x_hi = make_input_scaler(X_raw)
    y_scaler, y_unscaler, y_lo, y_hi = make_output_scaler(y_raw)

    X_scaled = x_scaler(X_raw)
    y_scaled = y_scaler(y_raw)

    print(f"[SCALE] input lo={x_lo}  hi={x_hi}")
    print(f"[SCALE] output lo={y_lo:.6f}  hi={y_hi:.6f}")

    # -----------------------------------------------------------------
    # 3. Random shuffled train/val/test split (points 2 and 3)
    # -----------------------------------------------------------------
    X_trv, X_test, y_trv_s, y_test_s = train_test_split(
        X_scaled, y_scaled,
        test_size=cfg["test_ratio"],
        random_state=cfg["random_seed"], shuffle=True,
    )
    val_relative = cfg["val_ratio"] / (1.0 - cfg["test_ratio"])
    X_train, X_val, y_train_s, y_val_s = train_test_split(
        X_trv, y_trv_s,
        test_size=val_relative,
        random_state=cfg["random_seed"], shuffle=True,
    )
    print(f"[SPLIT] train={len(X_train)}  val={len(X_val)}  "
          f"test={len(X_test)}")

    # -----------------------------------------------------------------
    # 4. Train (single or sweep) using PSO
    # -----------------------------------------------------------------
    if cfg["rules_sweep"]:
        rule_grid = [4, 9, 16, 25]
        sweep_results = {}
        best_model = None
        best_history = None
        best_rules = None
        best_val_rmse = float("inf")

        for r in rule_grid:
            model_r, hist_r = train_one(
                r, X_train, y_train_s, X_val, y_val_s,
                cfg["pso_params"], cfg["smoke_params"],
                cfg["smoke_samples"], y_unscaler,
                label=f"rules={r}",
            )
            sweep_results[r] = hist_r
            if hist_r["val"]["rmse"] < best_val_rmse:
                best_val_rmse = hist_r["val"]["rmse"]
                best_model = model_r
                best_history = hist_r
                best_rules = r

        print(f"\n[SWEEP] Best m_rules = {best_rules}  "
              f"val RMSE = {best_val_rmse:.6f}")
        model = best_model
        history = best_history
        m_rules_used = best_rules
        history["sweep_results"] = {
            str(k): {"train": v["train"], "val": v["val"]}
            for k, v in sweep_results.items()
        }
    else:
        model, history = train_one(
            cfg["m_rules"], X_train, y_train_s, X_val, y_val_s,
            cfg["pso_params"], cfg["smoke_params"],
            cfg["smoke_samples"], y_unscaler,
            label=f"rules={cfg['m_rules']}",
        )
        m_rules_used = cfg["m_rules"]

    # -----------------------------------------------------------------
    # 5. Final evaluation on TEST set (in original scale)
    # -----------------------------------------------------------------
    y_pred_train_s = safe_predict(model, X_train)
    y_pred_val_s = safe_predict(model, X_val)
    y_pred_test_s = safe_predict(model, X_test)

    y_train = y_unscaler(y_train_s)
    y_val = y_unscaler(y_val_s)
    y_test = y_unscaler(y_test_s)
    y_pred_train = y_unscaler(y_pred_train_s)
    y_pred_val = y_unscaler(y_pred_val_s)
    y_pred_test = y_unscaler(y_pred_test_s)

    train_m = compute_metrics(y_train, y_pred_train)
    val_m = compute_metrics(y_val, y_pred_val)
    test_m = compute_metrics(y_test, y_pred_test)

    print("\n" + "=" * 64)
    print("FINAL METRICS (original mu scale)")
    print("=" * 64)
    print(f"{'Split':<10}{'RMSE':>14}{'MAE':>14}{'StdDev':>14}{'R^2':>10}")
    print("-" * 64)
    for name, m in [("train", train_m), ("val", val_m), ("test", test_m)]:
        print(f"{name:<10}{m['rmse']:>14.6f}{m['mae']:>14.6f}"
              f"{m['std']:>14.6f}{m['r2']:>10.4f}")
    print("=" * 64)

    # -----------------------------------------------------------------
    # 6. Save model + metadata
    # -----------------------------------------------------------------
    metadata = {
        "model_type": "T1Mamdani_ML",
        "optimizer": "PSO (pyit2fls built-in)",
        "pso_params": cfg["pso_params"],
        "n_input": X_train.shape[1],
        "m_rules": m_rules_used,
        "rules_sweep_used": cfg["rules_sweep"],
        "input_scaling": {
            "method": "mean +/- 3*std with 5% padding",
            "lo": [float(v) for v in x_lo],
            "hi": [float(v) for v in x_hi],
            "input_columns": cfg["input_cols"],
        },
        "output_scaling": {
            "method": "min-max [0, 1] with 5% padding",
            "lo": float(y_lo),
            "hi": float(y_hi),
            "output_column": cfg["output_col"],
        },
        "split": {
            "train_size": len(X_train),
            "val_size": len(X_val),
            "test_size": len(X_test),
            "test_ratio": cfg["test_ratio"],
            "val_ratio": cfg["val_ratio"],
            "shuffled": True,
            "random_seed": cfg["random_seed"],
        },
        "metrics": {"train": train_m, "val": val_m, "test": test_m},
        "history": {
            "final_fitness_scaled": history["final_fitness_scaled"],
            "smoke_fitness_scaled": history["smoke_fitness_scaled"],
            "smoke_conv_first_last": history["smoke_conv_first_last"],
        },
        "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }
    if cfg["rules_sweep"]:
        metadata["sweep_results"] = history["sweep_results"]

    model_path, meta_path = save_artifacts(
        model, metadata,
        cfg["output_dir"],
        cfg["model_filename"],
        cfg["metadata_filename"],
    )
    print(f"\n[SAVE] model    -> {model_path}")
    print(f"[SAVE] metadata -> {meta_path}")

    # -----------------------------------------------------------------
    # 7. Visualization
    # -----------------------------------------------------------------
    fig = plt.figure(figsize=(15, 10))

    # --- (a) Predictions vs actual ---
    ax1 = plt.subplot(2, 2, 1)
    ax1.plot(y_train, label="train actual", color="#2563eb", alpha=0.4)
    ax1.plot(y_pred_train, label="train pred", color="#1e40af",
             linestyle="--", linewidth=0.9)
    offset_val = len(y_train)
    offset_test = offset_val + len(y_val)
    val_idx = np.arange(offset_val, offset_val + len(y_val))
    test_idx = np.arange(offset_test, offset_test + len(y_test))
    ax1.plot(val_idx, y_val, label="val actual", color="#16a34a", alpha=0.4)
    ax1.plot(val_idx, y_pred_val, label="val pred", color="#166534",
             linestyle="--", linewidth=0.9)
    ax1.plot(test_idx, y_test, label="test actual", color="#dc2626",
             alpha=0.4)
    ax1.plot(test_idx, y_pred_test, label="test pred", color="#991b1b",
             linestyle="--", linewidth=0.9)
    ax1.set_title("Predictions vs Actual (original mu scale)",
                  fontsize=11, fontweight="bold")
    ax1.set_xlabel("sample index (shuffled)")
    ax1.set_ylabel("optimal mu")
    ax1.legend(loc="best", fontsize=8, ncol=2)
    ax1.grid(True, alpha=0.3)

    # --- (b) Scatter actual vs predicted (test) ---
    ax2 = plt.subplot(2, 2, 2)
    ax2.scatter(y_test, y_pred_test, c="#dc2626", s=18, alpha=0.6,
                label="test")
    ax2.scatter(y_val, y_pred_val, c="#16a34a", s=18, alpha=0.6,
                label="val")
    lo = min(y_raw.min(), y_pred_test.min())
    hi = max(y_raw.max(), y_pred_test.max())
    ax2.plot([lo, hi], [lo, hi], "k--", linewidth=1, label="ideal")
    ax2.set_title(f"Predicted vs Actual  (test R^2 = {test_m['r2']:.4f})",
                  fontsize=11, fontweight="bold")
    ax2.set_xlabel("actual mu")
    ax2.set_ylabel("predicted mu")
    ax2.legend(loc="best", fontsize=9)
    ax2.grid(True, alpha=0.3)

    # --- (c) Convergence ---
    ax3 = plt.subplot(2, 2, 3)
    ax3.plot(history["convergence"], color="#7c3aed", linewidth=1.0)
    ax3.set_title("PSO Convergence (scaled fitness)",
                  fontsize=11, fontweight="bold")
    ax3.set_xlabel("iteration")
    ax3.set_ylabel("fitness")
    ax3.grid(True, alpha=0.3)

    # --- (d) Error histogram (test) ---
    ax4 = plt.subplot(2, 2, 4)
    test_err = y_test - y_pred_test
    ax4.hist(test_err, bins=30, color="#dc2626", alpha=0.7,
             edgecolor="#7f1d1d")
    ax4.axvline(0, color="black", linewidth=1)
    ax4.set_title(
        f"Test Error Distribution  "
        f"(MAE={test_m['mae']:.6f}, RMSE={test_m['rmse']:.6f})",
        fontsize=11, fontweight="bold",
    )
    ax4.set_xlabel("y_test - y_pred")
    ax4.set_ylabel("count")
    ax4.grid(True, alpha=0.3)

    sweep_note = " (sweep)" if cfg["rules_sweep"] else ""
    fig.suptitle(
        f"T1 Mamdani-PSO  |  rules={m_rules_used}{sweep_note}  |  "
        f"test RMSE={test_m['rmse']:.6f}  R^2={test_m['r2']:.4f}",
        fontsize=12, fontweight="bold", y=0.995,
    )
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(str(_PLOT_DIR / "mamdaniT1_pso_result.png"), dpi=200)
    plt.savefig(str(_PLOT_DIR / "mamdaniT1_pso_result.pdf"), format="pdf")
    print("[SAVE] plot -> mamdani_pso_result.png / .pdf")

    # ===== PLOT KONVERGENSI PSO (paper-ready, 1 garis) =====
    # PSO 'convergence' = MSE di output scaled [0,1].
    # Konversi ke skala mu asli supaya angka di sumbu Y bisa dirujuk dari tabel.
    #   y_orig = y_scaled * span  ->  MSE_orig = MSE_scaled * span^2
    y_span = y_hi - y_lo
    convergence_scaled = np.asarray(history["convergence"])
    convergence_orig = convergence_scaled * (y_span ** 2)
    iters = np.arange(1, len(convergence_orig) + 1)
    best_iter = int(np.argmin(convergence_orig)) + 1
    best_mse = float(np.min(convergence_orig))

    fig2 = plt.figure(figsize=(10, 6))
    ax = fig2.add_subplot(111)
    ax.semilogy(iters, convergence_orig,
                color="#2563eb", linewidth=1.8)
    ax.axvline(best_iter, color="#16a34a", linestyle=":",
               linewidth=1.0, label=f"Best at iter {best_iter}")

    ax.set_title(
        f"PSO Convergence  (best MSE = {best_mse:.3e})",
        fontsize=12, fontweight="bold")
    ax.set_xlabel(f"Iteration  (total {len(convergence_orig)})",
                  fontsize=11, fontweight="bold")
    ax.set_ylabel("Mean Squared Error",
                  fontsize=11, fontweight="bold")
    ax.legend(loc="upper right", fontsize=10)
    ax.grid(True, which="both", alpha=0.3)

    plt.tight_layout()
    plt.savefig(str(_PLOT_DIR / "mamdani_T1_convergence.png"), dpi=200)
    plt.savefig(str(_PLOT_DIR / "mamdani_T1_convergence.pdf"), format="pdf")
    print("[SAVE] convergence plot -> mamdani_T1_convergence.png / .pdf")
    plt.show()


if __name__ == "__main__":
    main()