from pyit2fls import (T1Mamdani, T1FS, gaussian_mf,rgaussian_mf, lgaussian_mf, trapezoid_mf, T1FS_plot, )
from numpy import (linspace, meshgrid, zeros, )
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import (LinearLocator, FormatStrFormatter, )


# Domain
domain_e = linspace(-0.1, 0.1, 200)
domain_edot = linspace(-3.0, 3.0, 200)
domain_kpp = linspace(0, 1, 200)
domain_kdp = linspace(0, 1, 200)
domain_alpha = linspace(1, 6, 200)
domain_mu = linspace(0.001, 0.006, 200)
domain_eta = linspace(650, 750, 200)

# Error membership
e_NB = T1FS(domain_e, rgaussian_mf, [-0.1, 0.01, 1.])
e_NM = T1FS(domain_e, gaussian_mf, [-0.06667, 0.01, 1.])
e_NS = T1FS(domain_e, gaussian_mf, [-0.03333, 0.01, 1.])
e_ZO = T1FS(domain_e, gaussian_mf, [0, 0.01, 1.])
e_PS = T1FS(domain_e, gaussian_mf, [0.03333, 0.01, 1.])
e_PM = T1FS(domain_e, gaussian_mf, [0.06667, 0.01, 1.])
e_PB = T1FS(domain_e, lgaussian_mf, [0.1, 0.01, 1.])
# T1FS_plot(e_NB, e_NM, e_NS, e_ZO, e_PS, e_PM, e_PB,
#         legends=["NB", "NM", "NS", "ZO", "PS", "PM", "PB"])

edot_NB = T1FS(domain_edot, rgaussian_mf, [-3, 0.3, 1.])
edot_NM = T1FS(domain_edot, gaussian_mf,  [-1.5, 0.3, 1.])
edot_NS = T1FS(domain_edot, gaussian_mf,  [-0.7, 0.3, 1.])
edot_ZO = T1FS(domain_edot, gaussian_mf,  [ 0.0, 0.3, 1.])
edot_PS = T1FS(domain_edot, gaussian_mf,  [ 0.7, 0.3, 1.])
edot_PM = T1FS(domain_edot, gaussian_mf,  [ 1.5, 0.3, 1.])
edot_PB = T1FS(domain_edot, lgaussian_mf, [ 3, 0.3, 1.])
# T1FS_plot(edot_NB, edot_NM, edot_NS, edot_ZO, edot_PS, edot_PM, edot_PB,
#         legends=["NB", "NM", "NS", "ZO", "PS", "PM", "PB"])

# kpp membership
kpp_S = T1FS(domain_kpp, gaussian_mf, [0, 0.5, 1])
kpp_B = T1FS(domain_kpp, gaussian_mf, [1, 0.5, 1.])
# T1FS_plot(kpp_S, kpp_B,
#         legends=["S", "B"])

# kdp membership t1_fuzzy_output_mf
kdp_S = T1FS(domain_kdp, gaussian_mf, [0, 0.5, 1])
kdp_B = T1FS(domain_kdp, gaussian_mf, [1, 0.5, 1.])
# T1FS_plot(kdp_S, kdp_B,
#         legends=["S", "B"])


# alpha membership
alpha_S = T1FS(domain_alpha, gaussian_mf,[2, 0.1, 1])
alpha_MS = T1FS(domain_alpha, gaussian_mf,[3, 0.1, 1])
alpha_MB = T1FS(domain_alpha, gaussian_mf,[4, 0.1, 1])
alpha_B = T1FS(domain_alpha, gaussian_mf,[5, 0.1, 1])
T1FS_plot(alpha_S, alpha_MS, alpha_MB, alpha_B,
        legends=["S", "MS", "MB", "B"])

mu_S = T1FS(domain_mu, gaussian_mf, [0.001, 0.001, 1])
mu_B = T1FS(domain_mu, gaussian_mf, [0.006, 0.001, 1])
# T1FS_plot(mu_S, mu_B,
#         legends=["S", "B"])

# eta membership
eta_S = T1FS(domain_eta, gaussian_mf, [650.0, 10, 1])
eta_M = T1FS(domain_eta, gaussian_mf, [700.0, 10, 1])
eta_B = T1FS(domain_eta, gaussian_mf, [750.0, 10, 1])
# T1FS_plot(eta_S, eta_M, eta_B,
#         legends=["S", "M", "B"])

myT1Mamdani = T1Mamdani(engine="Product", defuzzification="CoG")
# Input
myT1Mamdani.add_input_variable("e")
myT1Mamdani.add_input_variable("edot")

# Output
myT1Mamdani.add_output_variable("kpp")
myT1Mamdani.add_output_variable("kdp")
myT1Mamdani.add_output_variable("alpha")
myT1Mamdani.add_output_variable("mu")
myT1Mamdani.add_output_variable("eta")

# Rules 1-7
myT1Mamdani.add_rule([("e", e_NB),  ("edot", edot_NB)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_B),])
myT1Mamdani.add_rule([("e", e_NB),  ("edot", edot_NM)], [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_B),])
myT1Mamdani.add_rule([("e", e_NB),  ("edot", edot_NS)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_NB),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_NB),  ("edot", edot_PS)], [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_S),])
myT1Mamdani.add_rule([("e", e_NB),  ("edot", edot_PM)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_S),])
myT1Mamdani.add_rule([("e", e_NB),  ("edot", edot_PB)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_S),])

# Rules 8-14
myT1Mamdani.add_rule([("e", e_NM),  ("edot", edot_NB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_B),])
myT1Mamdani.add_rule([("e", e_NM),  ("edot", edot_NM)], [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_B),])
myT1Mamdani.add_rule([("e", e_NM),  ("edot", edot_NS)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_NM),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_NM),  ("edot", edot_PS)], [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_NM),  ("edot", edot_PM)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_S),])
myT1Mamdani.add_rule([("e", e_NM),  ("edot", edot_PB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_B), ("eta", eta_S),])

# Rules 15-21
myT1Mamdani.add_rule([("e", e_NS),  ("edot", edot_NB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MB), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_NS),  ("edot", edot_NM)], [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_NS),  ("edot", edot_NS)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_NS),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_NS),  ("edot", edot_PS)], [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_NS),  ("edot", edot_PM)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_B), ("eta", eta_S),])
myT1Mamdani.add_rule([("e", e_NS),  ("edot", edot_PB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MB), ("mu", mu_B), ("eta", eta_S),])

# Rules 22-28
myT1Mamdani.add_rule([("e", e_ZO),  ("edot", edot_NB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_B), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_ZO),  ("edot", edot_NM)], [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MB), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_ZO),  ("edot", edot_NS)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_ZO),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_S),])
myT1Mamdani.add_rule([("e", e_ZO),  ("edot", edot_PS)], [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_ZO),  ("edot", edot_PM)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MB), ("mu", mu_B), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_ZO),  ("edot", edot_PB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_B), ("mu", mu_B), ("eta", eta_M),])

# Rules 29-35
myT1Mamdani.add_rule([("e", e_PS),  ("edot", edot_NB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_B), ("mu", mu_S), ("eta", eta_S),])
myT1Mamdani.add_rule([("e", e_PS),  ("edot", edot_NM)], [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MB), ("mu", mu_S), ("eta", eta_S),])
myT1Mamdani.add_rule([("e", e_PS),  ("edot", edot_NS)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_PS),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_PS),  ("edot", edot_PS)], [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_PS),  ("edot", edot_PM)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MB), ("mu", mu_B), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_PS),  ("edot", edot_PB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_B), ("mu", mu_B), ("eta", eta_M),])

# Rules 36-42
myT1Mamdani.add_rule([("e", e_PM),  ("edot", edot_NB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_S),])
myT1Mamdani.add_rule([("e", e_PM),  ("edot", edot_NM)], [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_S),])
myT1Mamdani.add_rule([("e", e_PM),  ("edot", edot_NS)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_B), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_PM),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_B), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_PM),  ("edot", edot_PS)], [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_B), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_PM),  ("edot", edot_PM)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_B), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_PM),  ("edot", edot_PB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_B), ("eta", eta_B),])

# Rules 43-49
myT1Mamdani.add_rule([("e", e_PB),  ("edot", edot_NB)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_S),])
myT1Mamdani.add_rule([("e", e_PB),  ("edot", edot_NM)], [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_B), ("eta", eta_S),])
myT1Mamdani.add_rule([("e", e_PB),  ("edot", edot_NS)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_B), ("eta", eta_S),])
myT1Mamdani.add_rule([("e", e_PB),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_B), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_PB),  ("edot", edot_PS)], [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_B), ("eta", eta_M),])
myT1Mamdani.add_rule([("e", e_PB),  ("edot", edot_PM)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_B), ("eta", eta_B),])
myT1Mamdani.add_rule([("e", e_PB),  ("edot", edot_PB)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_B), ("eta", eta_B),])



# # 1. Membuat grid E dan EDOT
# E, EDOT = meshgrid(domain_e, domain_edot)

# # 2. Menyiapkan matriks kosong untuk menyimpan hasil evaluasi ketiga variabel
# O_kpp = zeros(shape=(len(domain_e), len(domain_edot)))
# O_kdp = zeros(shape=(len(domain_e), len(domain_edot)))
# O_alpha = zeros(shape=(len(domain_e), len(domain_edot)))

# # 3. Proses Evaluasi Type-1
# # Perbaikan: Memastikan zip memasangkan range dan domain yang benar
# # 3. Proses Evaluasi Type-1
# for i, e in zip(range(len(domain_e)), domain_e):
#     for j, edot in zip(range(len(domain_edot)), domain_edot):
        
#         # Evaluasi T1Mamdani menghasilkan (firing_strengths, crisp_outputs)
#         s, c = myT1Mamdani.evaluate({"e": e, "edot": edot})
        
#         # Menyimpan nilai crisp ke masing-masing matriks
#         O_kpp[i, j] = c["kpp"]
#         O_kdp[i, j] = c["kdp"]
#         O_alpha[i, j] = c["alpha"]
        
#         # Menampilkan progres perhitungan ke terminal
#         print(f"E = {e:.4f}, EDOT = {edot:.4f} →→ KPP = {c['kpp']:.4f}, KDP = {c['kdp']:.4f}, ALPHA = {c['alpha']:.4f}")

# # 4. Plotting (1 Figure, 3 Subplots)
# fig = plt.figure(figsize=(18, 5))

# # --- Subplot 1: KPP Surface ---
# ax1 = fig.add_subplot(1, 3, 1, projection="3d")
# surf1 = ax1.plot_surface(E, EDOT, O_kpp, cmap=cm.coolwarm,
#                          linewidth=0, antialiased=False)
# ax1.zaxis.set_major_locator(LinearLocator(10))
# ax1.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
# ax1.set_title("KPP Surface")
# fig.colorbar(surf1, ax=ax1, shrink=0.5, aspect=5)

# # --- Subplot 2: KDP Surface ---
# ax2 = fig.add_subplot(1, 3, 2, projection="3d")
# surf2 = ax2.plot_surface(E, EDOT, O_kdp, cmap=cm.coolwarm,
#                          linewidth=0, antialiased=False)
# ax2.zaxis.set_major_locator(LinearLocator(10))
# ax2.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
# ax2.set_title("KDP Surface")
# fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=5)

# # --- Subplot 3: Alpha Surface ---
# ax3 = fig.add_subplot(1, 3, 3, projection="3d")
# surf3 = ax3.plot_surface(E, EDOT, O_alpha, cmap=cm.coolwarm,
#                          linewidth=0, antialiased=False)
# ax3.zaxis.set_major_locator(LinearLocator(10))
# ax3.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
# ax3.set_title("Alpha Surface")
# fig.colorbar(surf3, ax=ax3, shrink=0.5, aspect=5)

# # Merapikan jarak dan menampilkan grafik
# plt.tight_layout()
# plt.show()