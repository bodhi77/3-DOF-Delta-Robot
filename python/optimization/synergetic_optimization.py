"""
=======================================================
SYNERGETIC CONTROL OPTIMIZATION  (constant mu, per joint)
=======================================================
Mencari nilai mu STATIS terbaik untuk synergetic control biasa
(tanpa neuro-fuzzy). Output: 3 nilai mu (mu1, mu2, mu3), satu per motor,
siap dipakai langsung di synergetic / firmware.

Metode: sweep 1-dimensi.
  - Untuk tiap kandidat mu, jalankan SIMULASI PENUH (mu konstan, sama di
    semua joint), hitung RMSE per joint sepanjang trajektori.
  - Untuk tiap joint, pilih mu yang RMSE joint itu terkecil.
  - (Opsi) cost = RMSE + w_tau * mean|d_tau| untuk seimbangkan kehalusan.

Model: V6 dengan current-PID (v6core) -> dekat hardware (mu besar dihukum).

Catatan: delta robot terkopling, jadi mu per-joint dicari dgn mu seragam
saat rollout (pendekatan praktis). Hasil = baseline mu konstan untuk
pembanding melawan neuro-fuzzy.
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

_python_dir = Path(__file__).resolve().parent
while _python_dir.name != "python":
    _python_dir = _python_dir.parent
_root_dir = _python_dir.parent
sys.path.insert(0, str(_python_dir / "neurofuzzy"))
_RESULTS_DIR = _root_dir / "results" / "optimization"
_RESULTS_DIR.mkdir(parents=True, exist_ok=True)

import robot_dynamics_core as sim


# =======================================================
# CONFIG
# =======================================================
MU_MIN = 0.001
MU_MAX = 0.006
MU_STEPS = 100
W_TAU = 0.0            # bobot kehalusan torsi (0 = murni RMSE). coba 0.05-0.1
OUT_PNG = str(_RESULTS_DIR / "synergetic_optimization.png")


def rollout_constant_mu(mu_scalar):
    """Jalankan simulasi penuh dengan mu konstan (seragam semua joint).
    Return (rmse_per_joint[3], mean_dtau)."""
    sim.reset_pid()
    state = np.zeros(12)                 # [q, dq, I, int_e]
    n = sim.N_TOTAL
    warm = sim.N_WARMUP
    mu_vec = np.full(3, mu_scalar)
    sq = np.zeros(3)
    cnt = 0
    dtau_sum = 0.0
    prev_tau = np.zeros(3)

    for i in range(n):
        q_d_t = sim.q_d[i]
        dq_d_t = sim.dq_d[i]
        ddq_d_t = sim.ddq_d[i]
        t_now = i * sim.DT_LOG
        sim._JAC_CACHE['J_prev'] = np.eye(3)
        sim._JAC_CACHE['t_prev'] = t_now
        state, tau = sim.simulate_until_next_log(
            state, q_d_t, dq_d_t, ddq_d_t, mu_vec, t_now)
        if not np.all(np.isfinite(state)):
            return np.full(3, 1e6), 1e6
        if warm <= i < n:
            err_deg = np.degrees(q_d_t - state[0:3])
            sq += err_deg ** 2
            dtau_sum += float(np.sum(np.abs(tau - prev_tau)))
            cnt += 1
        prev_tau = tau

    if cnt == 0:
        return np.full(3, 1e6), 1e6
    rmse = np.sqrt(sq / cnt)
    return rmse, dtau_sum / cnt


def main():
    print("=" * 60)
    print("SYNERGETIC OPTIMIZATION - constant mu per joint (V6 + PID arus)")
    print("=" * 60)
    mus = np.linspace(MU_MIN, MU_MAX, MU_STEPS)
    rmse_all = np.zeros((MU_STEPS, 3))
    dtau_all = np.zeros(MU_STEPS)

    for k, mu in enumerate(mus):
        rmse, dtau = rollout_constant_mu(mu)
        rmse_all[k] = rmse
        dtau_all[k] = dtau
        print(f"  mu={mu:.5f}  RMSE(j1,j2,j3)="
              f"[{rmse[0]:.4f}, {rmse[1]:.4f}, {rmse[2]:.4f}] deg  "
              f"meanDtau={dtau:.3f}")

    # cost per joint = RMSE + W_TAU * dtau (dtau global, sama utk semua joint)
    cost_all = rmse_all + W_TAU * dtau_all[:, None]

    best_mu = np.zeros(3)
    best_rmse = np.zeros(3)
    for j in range(3):
        kbest = int(np.argmin(cost_all[:, j]))
        best_mu[j] = mus[kbest]
        best_rmse[j] = rmse_all[kbest, j]

    print("\n" + "=" * 60)
    print("HASIL - mu statis terbaik per motor (siap pakai):")
    for j in range(3):
        print(f"  mu{j+1} = {best_mu[j]:.6f}   (RMSE joint {j+1} = "
              f"{best_rmse[j]:.4f} deg)")
    print("=" * 60)
    print(f"\nmu_vector = [{best_mu[0]:.6f}, {best_mu[1]:.6f}, "
          f"{best_mu[2]:.6f}]")

    # ---- plot RMSE vs mu per joint ----
    plt.figure(figsize=(10, 6))
    colors = ['#028090', '#00A896', '#B85042']
    for j in range(3):
        plt.plot(mus, rmse_all[:, j], color=colors[j], lw=2,
                 label=f'Joint {j+1}')
        plt.scatter([best_mu[j]], [best_rmse[j]], color=colors[j],
                    s=80, zorder=5, edgecolor='k')
    plt.xlabel('mu (manifold time constant)')
    plt.ylabel('RMSE (deg)')
    plt.title('Synergetic Optimization: RMSE vs constant mu (per joint)')
    plt.legend(); plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT_PNG, dpi=200)
    print(f"\nsaved plot -> {OUT_PNG}")


if __name__ == "__main__":
    main()
