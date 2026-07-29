from pyit2fls import IT2Mamdani, IT2FS_Gaussian_UncertStd, IT2FS_Gaussian_UncertMean, IT2FS_plot, \
                    min_t_norm, max_s_norm, crisp, \
                    R_IT2FS_Gaussian_UncertStd, L_IT2FS_Gaussian_UncertStd
from numpy import linspace

from numpy import linspace, meshgrid, zeros
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

# Domain
domain_e = linspace(-0.1, 0.1, 200)
domain_edot = linspace(-3.0, 3.0, 200)
domain_kpp = linspace(0, 1, 100)
domain_kdp = linspace(0, 1, 100)
domain_alpha = linspace(1, 6, 100)
domain_mu = linspace(0.001, 0.006, 200)
domain_eta = linspace(650, 750, 200) 

# Error membership
e_NB = R_IT2FS_Gaussian_UncertStd(domain_e, [-0.1, 0.01, 0.01, 1])
e_NM = IT2FS_Gaussian_UncertStd(domain_e, [-0.0667, 0.01, 0.01, 1])
e_NS = IT2FS_Gaussian_UncertStd(domain_e, [-0.0333, 0.01, 0.01, 1])
e_ZO = IT2FS_Gaussian_UncertStd(domain_e, [0, 0.01, 0.01, 1])
e_PS = IT2FS_Gaussian_UncertStd(domain_e, [0.0333, 0.01, 0.01, 1])
e_PM = IT2FS_Gaussian_UncertStd(domain_e, [0.0667, 0.01, 0.01, 1])
e_PB = L_IT2FS_Gaussian_UncertStd(domain_e, [0.1, 0.01, 0.01, 1])

# IT2FS_plot(e_NB, e_NM, e_NS, e_ZO, e_PS, e_PM, e_PB,
#         legends=["NB", "NM", "NS", "ZO", "PS", "PM", "PB"])

# dError membership

edot_NB = R_IT2FS_Gaussian_UncertStd(domain_edot, [-2.5, 0.5, 0.1, 1])
edot_NM = IT2FS_Gaussian_UncertStd(domain_edot,   [-1.5, 0.5, 0.1, 1])
edot_NS = IT2FS_Gaussian_UncertStd(domain_edot,   [-0.7, 0.4, 0.1, 1])
edot_ZO = IT2FS_Gaussian_UncertStd(domain_edot,   [ 0.0, 0.3, 0.1, 1])
edot_PS = IT2FS_Gaussian_UncertStd(domain_edot,   [ 0.7, 0.4, 0.1, 1])
edot_PM = IT2FS_Gaussian_UncertStd(domain_edot,   [ 1.5, 0.5, 0.1, 1])
edot_PB = L_IT2FS_Gaussian_UncertStd(domain_edot, [ 2.5, 0.5, 0.1, 1])


# IT2FS_plot(edot_NB, edot_NM, edot_NS, edot_ZO, edot_PS, edot_PM, edot_PB,
#         legends=["NB", "NM", "NS", "ZO", "PS", "PM", "PB"])

# kpp membership
kpp_S = IT2FS_Gaussian_UncertStd(domain_kpp, [0, 0.2, 0.2, 1])
kpp_B = IT2FS_Gaussian_UncertStd(domain_kpp, [1, 0.2, 0.2, 1.])
# IT2FS_plot(kpp_S, kpp_B,
#         legends=["S", "B"])

# kdp membership
kdp_S = IT2FS_Gaussian_UncertStd(domain_kdp, [0, 0.2, 0.2, 1.])
kdp_B = IT2FS_Gaussian_UncertStd(domain_kdp, [1, 0.2, 0.2, 1.])
# IT2FS_plot(kdp_S, kdp_B,
#         legends=["S", "B"])

# alpha membership
alpha_S = IT2FS_Gaussian_UncertStd(domain_alpha, [2, 0.1, 0.1, 1])
alpha_MS = IT2FS_Gaussian_UncertStd(domain_alpha, [3, 0.1, 0.1, 1.])
alpha_MB = IT2FS_Gaussian_UncertStd(domain_alpha, [4, 0.1, 0.1, 1.])
alpha_B = IT2FS_Gaussian_UncertStd(domain_alpha, [5, 0.1, 0.1, 1.])

# IT2FS_plot(alpha_S, alpha_MS, alpha_MB, alpha_B,
#         legends=["S", "MS", "MB", "B"])

mu_S = IT2FS_Gaussian_UncertStd(domain_mu, [0.001, 0.0004, 0.002, 1])
mu_B = IT2FS_Gaussian_UncertStd(domain_mu, [0.006, 0.0004, 0.002, 1])
# IT2FS_plot(mu_S, mu_B,
#         legends=["S", "B"])

# eta membership
eta_S = IT2FS_Gaussian_UncertStd(domain_eta, [650.0, 10, 5, 1])
eta_M = IT2FS_Gaussian_UncertStd(domain_eta, [700.0, 10, 5, 1])
eta_B = IT2FS_Gaussian_UncertStd(domain_eta, [750.0, 10, 5, 1])
IT2FS_plot(eta_S, eta_M, eta_B,
        legends=["S", "M", "B"])

myIT2FLS = IT2Mamdani(min_t_norm, max_s_norm)

myIT2FLS.add_input_variable("e")
myIT2FLS.add_input_variable("edot")

myIT2FLS.add_output_variable("kpp")
myIT2FLS.add_output_variable("kdp")
myIT2FLS.add_output_variable("alpha")
myIT2FLS.add_output_variable("mu")
myIT2FLS.add_output_variable("eta")

##### Rules #####################################################
# Rules 1-7 (e_NB)
myIT2FLS.add_rule([("e", e_NB),  ("edot", edot_NB)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_B),])
myIT2FLS.add_rule([("e", e_NB),  ("edot", edot_NM)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_B),])
myIT2FLS.add_rule([("e", e_NB),  ("edot", edot_NS)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_NB),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_NB),  ("edot", edot_PS)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_S),])
myIT2FLS.add_rule([("e", e_NB),  ("edot", edot_PM)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_S),])
myIT2FLS.add_rule([("e", e_NB),  ("edot", edot_PB)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_S),])

# Rules 8-14 (e_NM)
myIT2FLS.add_rule([("e", e_NM),  ("edot", edot_NB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_B),])
myIT2FLS.add_rule([("e", e_NM),  ("edot", edot_NM)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_B),])
myIT2FLS.add_rule([("e", e_NM),  ("edot", edot_NS)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_NM),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_NM),  ("edot", edot_PS)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_NM),  ("edot", edot_PM)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_S),])
myIT2FLS.add_rule([("e", e_NM),  ("edot", edot_PB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_S),])

# Rules 15-21 (e_NS)
myIT2FLS.add_rule([("e", e_NS),  ("edot", edot_NB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MB), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_NS),  ("edot", edot_NM)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_NS),  ("edot", edot_NS)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_B), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_NS),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_B), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_NS),  ("edot", edot_PS)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_B), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_NS),  ("edot", edot_PM)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_S),])
myIT2FLS.add_rule([("e", e_NS),  ("edot", edot_PB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MB), ("mu", mu_S), ("eta", eta_S),])

# Rules 22-28 (e_ZO)
myIT2FLS.add_rule([("e", e_ZO),  ("edot", edot_NB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_B), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_ZO),  ("edot", edot_NM)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MB), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_ZO),  ("edot", edot_NS)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_B), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_ZO),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_B), ("eta", eta_S),])
myIT2FLS.add_rule([("e", e_ZO),  ("edot", edot_PS)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_B), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_ZO),  ("edot", edot_PM)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MB), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_ZO),  ("edot", edot_PB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_B), ("mu", mu_S), ("eta", eta_M),])

# Rules 29-35 (e_PS)
myIT2FLS.add_rule([("e", e_PS),  ("edot", edot_NB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_B), ("mu", mu_S), ("eta", eta_S),])
myIT2FLS.add_rule([("e", e_PS),  ("edot", edot_NM)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MB), ("mu", mu_S), ("eta", eta_S),])
myIT2FLS.add_rule([("e", e_PS),  ("edot", edot_NS)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_B), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_PS),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_B), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_PS),  ("edot", edot_PS)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_B), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_PS),  ("edot", edot_PM)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MB), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_PS),  ("edot", edot_PB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_B), ("mu", mu_S), ("eta", eta_M),])

# Rules 36-42 (e_PM)
myIT2FLS.add_rule([("e", e_PM),  ("edot", edot_NB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_S),])
myIT2FLS.add_rule([("e", e_PM),  ("edot", edot_NM)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_S),])
myIT2FLS.add_rule([("e", e_PM),  ("edot", edot_NS)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_PM),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_PM),  ("edot", edot_PS)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_PM),  ("edot", edot_PM)],  [("kpp", kpp_B), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_PM),  ("edot", edot_PB)],  [("kpp", kpp_S), ("kdp", kdp_B), ("alpha", alpha_MS), ("mu", mu_S), ("eta", eta_B),])

# Rules 43-49 (e_PB)
myIT2FLS.add_rule([("e", e_PB),  ("edot", edot_NB)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_S),])
myIT2FLS.add_rule([("e", e_PB),  ("edot", edot_NM)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_S),])
myIT2FLS.add_rule([("e", e_PB),  ("edot", edot_NS)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_S),])
myIT2FLS.add_rule([("e", e_PB),  ("edot", edot_ZO)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_PB),  ("edot", edot_PS)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_M),])
myIT2FLS.add_rule([("e", e_PB),  ("edot", edot_PM)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_B),])
myIT2FLS.add_rule([("e", e_PB),  ("edot", edot_PB)],  [("kpp", kpp_B), ("kdp", kdp_S), ("alpha", alpha_S), ("mu", mu_S), ("eta", eta_B),])

# E, EDOT = meshgrid(domain_e, domain_edot)
# inVar1 = zeros(shape=(len(domain_e), len(domain_edot)))
# inVar2 = zeros(shape=(len(domain_e), len(domain_edot)))
# inVar3 = zeros(shape=(len(domain_e), len(domain_edot)))
# for i, e in zip(range(len(domain_e)), domain_e):
#     for j, edot in zip(range(len(domain_edot)), domain_edot):
#         it2out, tr = myIT2FLS.evaluate({"e":e, "edot":edot})

#         # Output kpp
#         kpp_crisp = crisp(tr["kpp"])
#         inVar1[i, j] = kpp_crisp

#         # Output kdp
#         kdp_crisp = crisp(tr["kdp"])
#         inVar2[i, j] = kdp_crisp

#         # Output alpha
#         alpha_crisp = crisp(tr["alpha"])
#         inVar3[i, j] = alpha_crisp

#         print(f"E = {e:.4f}, EDOT = {edot:.4f} →→ KPP = {kpp_crisp:.4f}, KDP = {kdp_crisp:.4f}, ALPHA = {alpha_crisp:.4f}")

# Membuat satu figure utama dengan ukuran yang cukup lebar
# fig = plt.figure(figsize=(18, 5))

# # --- Subplot 1: kpp surface ---
# ax1 = fig.add_subplot(1, 3, 1, projection="3d")
# surf1 = ax1.plot_surface(E, EDOT, inVar1, cmap=cm.coolwarm,
#                          linewidth=0, antialiased=False)
# ax1.zaxis.set_major_locator(LinearLocator(10))
# ax1.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
# ax1.set_title("KPP Surface")
# fig.colorbar(surf1, ax=ax1, shrink=0.5, aspect=5)

# # --- Subplot 2: kdp surface ---
# ax2 = fig.add_subplot(1, 3, 2, projection="3d")
# surf2 = ax2.plot_surface(E, EDOT, inVar2, cmap=cm.coolwarm,
#                          linewidth=0, antialiased=False)
# ax2.zaxis.set_major_locator(LinearLocator(10))
# ax2.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
# ax2.set_title("KDP Surface")
# fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=5)

# # --- Subplot 3: alpha surface ---
# ax3 = fig.add_subplot(1, 3, 3, projection="3d")
# surf3 = ax3.plot_surface(E, EDOT, inVar3, cmap=cm.coolwarm,
#                          linewidth=0, antialiased=False)
# ax3.zaxis.set_major_locator(LinearLocator(10))
# ax3.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
# ax3.set_title("Alpha Surface")
# fig.colorbar(surf3, ax=ax3, shrink=0.5, aspect=5)

# # Merapikan jarak antar subplot agar tidak terpotong
# plt.tight_layout()
# plt.show()