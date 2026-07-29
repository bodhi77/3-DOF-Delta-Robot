import numpy as np
import itertools
import pandas as pd
import matplotlib.pyplot as plt
import sys

# =================================================================
# 1. PARAMETER MOTOR DC & INNER LOOP (FIXED)
# =================================================================
Kt = 1.0         # Konstanta torsi  [!] PLACEHOLDER - lihat CATATAN di bawah
Ke = 1.0         # Konstanta Back-EMF [!] PLACEHOLDER
R = 3.4          # Resistansi Armatur (Ohm)
L = 0.00018      # Induktansi Armatur (Henry)
J = 0.0443       # Momen Inersia (Kg-m^2)
b = 0.01859      # Koefisien Friksi viscous (N.m.s/rad)

# Friksi Coulomb / stiction (N.m). Default 0 = TIDAK mengubah fisika lama.
# Set > 0 (mis. 0.05) jika motormu butuh dorongan minimum untuk mulai bergerak.
TAU_COULOMB = 0.0

# Parameter Arus (Sudah Fix dari Arduino) - satuan gain ini = PWM-count per Amper
Kpc, Kic, Kdc = 20.0, 10.0, 0.015

# =================================================================
# [REVISI] KONSTANTA JALUR DAYA - HARUS SAMA DENGAN FIRMWARE
# Firmware: output PID arus = PWM count (bukan Volt), lalu
#           voltage = (pwm / PWM_MAX) * VSUPPLY
# Ini inti perbaikan: loop arus simulasi jadi selemah firmware.
# =================================================================
VSUPPLY = 12.0       # tegangan suplai motor (Volt)
PWM_MAX = 4095.0     # resolusi 12-bit
I_TERM_LIMIT = 1500.0  # klem integral arus (sama dengan constrain di firmware)

# Laju kontrol vs fisika (firmware: PID arus di-gate ~1 kHz)
DT_PHYS = 1e-5       # 10 us, cukup kecil untuk L/R = 53 us (stabil)
DT_CTRL = 1e-3       # 1 ms = 1 kHz, ZOH (Zero-Order Hold)


# =================================================================
# 2. SIMULASI FISIKA MOTOR & CASCADE PID (jalur sinyal = firmware)
# =================================================================
def evaluate_cascade_pid(Kp, Ki, Kd, t_end=2.0):
    decim = int(round(DT_CTRL / DT_PHYS))   # 100 langkah fisika per langkah kontrol
    n_phys = int(round(t_end / DT_PHYS))
    n_ctrl = n_phys // decim + 1

    theta, omega, current = 0.0, 0.0, 0.0
    theta_ref = 1.0   # rad  (CATATAN: setpoint lapangan biasanya < 1 rad)

    err_pos_prev, integral_pos = 0.0, 0.0
    err_curr_prev, I_term_curr = 0.0, 0.0

    voltage = 0.0
    target_current = 0.0

    itae = 0.0
    theta_arr = np.zeros(n_ctrl)
    time_arr = np.zeros(n_ctrl)
    k = 0

    for i in range(n_phys):
        t = i * DT_PHYS

        # ---- KONTROL (ZOH @ 1 kHz) ----
        if i % decim == 0:
            # OUTER LOOP (PID Posisi) -> target arus (Amper)
            err_pos = theta_ref - theta
            integral_pos = np.clip(integral_pos + err_pos * DT_CTRL, -2.0, 2.0)
            deriv_pos = (err_pos - err_pos_prev) / DT_CTRL
            target_current = (Kp * err_pos) + (Ki * integral_pos) + (Kd * deriv_pos)
            err_pos_prev = err_pos

            # INNER LOOP (PID Arus Fixed) -> [REVISI] OUTPUT = PWM, bukan Volt
            err_curr = target_current - current
            I_term_curr = np.clip(I_term_curr + Kic * err_curr * DT_CTRL,
                                  -I_TERM_LIMIT, I_TERM_LIMIT)
            deriv_curr = (err_curr - err_curr_prev) / DT_CTRL
            pwm = (Kpc * err_curr) + I_term_curr + (Kdc * deriv_curr)
            pwm = np.clip(pwm, -PWM_MAX, PWM_MAX)
            err_curr_prev = err_curr

            # [REVISI] PWM -> Volt (inilah faktor ~1/341 yang hilang sebelumnya)
            voltage = (pwm / PWM_MAX) * VSUPPLY

        # ---- FISIKA MOTOR (Euler @ 100 kHz) ----
        di_dt = (voltage - R * current - Ke * omega) / L
        current += di_dt * dt_safe(DT_PHYS)

        # Torsi: Kt*i - friksi viscous - friksi Coulomb
        tau_friction = b * omega + TAU_COULOMB * np.sign(omega)
        dw_dt = (Kt * current - tau_friction) / J
        omega += dw_dt * DT_PHYS
        theta += omega * DT_PHYS

        # ---- LOG & ITAE (pada laju kontrol) ----
        if i % decim == 0:
            theta_arr[k] = theta
            time_arr[k] = t
            itae += t * abs(theta_ref - theta) * DT_CTRL
            k += 1

        if np.isnan(theta) or abs(theta) > 50:
            return float('inf'), time_arr[:k], theta_arr[:k]

    # Buang slot trailing yang belum terisi
    theta_arr = theta_arr[:k]
    time_arr = time_arr[:k]

    # [REVISI OBJEKTIF] Hukum BERAT jika gagal mencapai/menahan target.
    # error steady-state = rata-rata |err| pada 25% waktu terakhir.
    # Tanpa ini, respons lambat yang mandek (mis. Kp kecil) malah menang
    # karena ITAE jendela pendek + lolos penalti overshoot.
    n_tail = max(1, len(theta_arr) // 4)
    ss_err = np.mean(np.abs(theta_ref - theta_arr[-n_tail:]))
    itae += 500.0 * ss_err

    # [REVISI] Penalti overshoot dilembutkan (dulu 100x = tebing yang
    # menenggelamkan segalanya). Toleransi 5%, lalu linear ringan.
    overshoot = np.max(theta_arr) - theta_ref
    if overshoot > 0.05:
        itae += 20.0 * (overshoot - 0.05)

    return itae, time_arr, theta_arr


def dt_safe(dt):
    return dt


# =================================================================
# 3. ALGORITMA GRID SEARCH
# =================================================================
def main():
    # [REVISI] Rentang Kp dinaikkan drastis. Dengan jalur PWM yang benar,
    # optimum jatuh ke ratusan (cocok dengan trial-error Kp~1050), BUKAN 5.
    Kp_vals = np.linspace(100.0, 2000.0, 20)   # 100 .. 2000
    Ki_vals = np.linspace(0.0, 20.0, 5)        # termasuk Ki = 10 (temuanmu)
    Kd_vals = np.linspace(0.0, 0.05, 4)        # termasuk Kd = 0.015 (temuanmu)

    total_combinations = len(Kp_vals) * len(Ki_vals) * len(Kd_vals)
    print(f"Mulai Grid Search untuk {total_combinations} kombinasi parameter...")

    results = []
    best_cost = float('inf')
    best_params = None
    best_response = None
    best_t = None

    counter = 0
    for Kp, Ki, Kd in itertools.product(Kp_vals, Ki_vals, Kd_vals):
        counter += 1
        sys.stdout.write(f"\rMengevaluasi... {counter}/{total_combinations}")
        sys.stdout.flush()

        itae, t_arr, y_arr = evaluate_cascade_pid(Kp, Ki, Kd)

        if itae != float('inf'):
            results.append({'Kp': Kp, 'Ki': Ki, 'Kd': Kd, 'ITAE': itae})
            if itae < best_cost:
                best_cost = itae
                best_params = (Kp, Ki, Kd)
                best_response = y_arr
                best_t = t_arr

    print("\n\n==========================================")
    print("GRID SEARCH SELESAI!")
    print("==========================================")

    if len(results) == 0:
        print("Semua kombinasi pada rentang ini menghasilkan sistem yang tidak stabil.")
        return

    df_results = pd.DataFrame(results)
    top_5 = df_results.sort_values('ITAE').head(5)

    print("--- TOP 5 PARAMETER TERBAIK ---")
    print(top_5.to_string(index=False))

    print("\n--- PARAMETER PALING OPTIMAL ---")
    print(f"Kp Posisi = {best_params[0]:.4f}")
    print(f"Ki Posisi = {best_params[1]:.4f}")
    print(f"Kd Posisi = {best_params[2]:.4f}")
    print(f"Nilai Error (ITAE) = {best_cost:.5f}")

    plt.figure(figsize=(9, 5))
    plt.plot(best_t, best_response, lw=2.5, color='#1f77b4',
             label=f'Optimal Grid Search\nKp={best_params[0]:.1f}, Ki={best_params[1]:.3f}, Kd={best_params[2]:.4f}')
    plt.axhline(1, color='black', linestyle='--', label='Target Posisi (1 Radian)')
    plt.title('Step Response: Cascade PID (jalur sinyal = firmware)', fontweight='bold')
    plt.xlabel('Waktu (Detik)')
    plt.ylabel('Posisi Sudut (Radian)')
    plt.legend()
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.show()


if __name__ == "__main__":
    main()

# =================================================================
# CATATAN PENTING (baca sebelum percaya angkanya):
# -----------------------------------------------------------------
# 1. Perbaikan utama: loop arus sekarang keluarkan PWM (0..4095) lalu
#    diskalakan ke Volt (pwm/4095*12), PERSIS firmware. Versi lama
#    memperlakukan output sebagai Volt langsung -> loop arus ~341x
#    terlalu kuat -> Kp kecil (5) sudah cukup. Itu sebabnya tak cocok.
#
# 2. Kt = Ke = 1.0 MASIH placeholder. Ini bikin plant terlalu perkasa.
#    Ukur Kt asli motormu (Nm/A) dan Ke asli (V.s/rad), masukkan di atas.
#    Dengan Kt kecil yang benar, target arus & Kp optimal akan bergeser
#    ke skala yang lebih masuk akal. Selama Kt=1, angka absolut tetap
#    perkiraan kasar - tapi ORDE-nya kini benar (ratusan, bukan satuan).
#
# 3. TAU_COULOMB = 0 secara default. Jika di lapangan motor "ngotot diam"
#    sampai dorongan tertentu, naikkan (mis. 0.03-0.08) agar simulasi
#    meniru stiction; ini juga menggeser Kp optimal ke atas.
#
# 4. theta_ref = 1.0 rad. Setpoint lapangamu mungkin lebih kecil; error
#    awal yang lebih kecil = butuh Kp lebih besar untuk arus yang sama.
# =================================================================