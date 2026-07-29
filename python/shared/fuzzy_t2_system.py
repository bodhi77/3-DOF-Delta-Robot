from pyit2fls import IT2Mamdani, IT2FS_Gaussian_UncertStd, R_IT2FS_Gaussian_UncertStd, L_IT2FS_Gaussian_UncertStd, min_t_norm, max_s_norm
from numpy import linspace

def myfuzzyT2_sys():
    # Domain
    domain_e = linspace(-0.1, 0.1, 200)
    domain_edot = linspace(-3.0, 3.0, 200)
    domain_kpp = linspace(0, 1, 200)
    domain_kdp = linspace(0, 1, 200)
    domain_alpha = linspace(1, 6, 200)
    domain_mu = linspace(0.001, 0.006, 200)
    domain_eta = linspace(650, 750, 200) 


##### Membership functions ####################################
    # Error membership
    e_NB = R_IT2FS_Gaussian_UncertStd(domain_e, [-0.1, 0.1, 0.01, 1])
    e_NM = IT2FS_Gaussian_UncertStd(domain_e, [-0.0667, 0.01, 0.01, 1])
    e_NS = IT2FS_Gaussian_UncertStd(domain_e, [-0.0333, 0.01, 0.01, 1])
    e_ZO = IT2FS_Gaussian_UncertStd(domain_e, [0, 0.01, 0.01, 1])
    e_PS = IT2FS_Gaussian_UncertStd(domain_e, [0.0333, 0.01, 0.01, 1])
    e_PM = IT2FS_Gaussian_UncertStd(domain_e, [0.0667, 0.01, 0.01, 1])
    e_PB = L_IT2FS_Gaussian_UncertStd(domain_e, [0.1, 0.01, 0.01, 1])

    # dError membership
    # edot_NB = IT2FS_Gaussian_UncertStd(domain_edot, [-10000, 800, 1000, 1])
    # edot_NM = IT2FS_Gaussian_UncertStd(domain_edot, [-6667, 800, 1000, 1])
    # edot_NS = IT2FS_Gaussian_UncertStd(domain_edot, [-3333, 800, 1000, 1])
    # edot_ZO = IT2FS_Gaussian_UncertStd(domain_edot, [0, 800, 1000, 1.])
    # edot_PS = IT2FS_Gaussian_UncertStd(domain_edot, [3333, 800, 1000, 1])
    # edot_PM = IT2FS_Gaussian_UncertStd(domain_edot, [6667, 800, 1000, 1])
    # edot_PB = IT2FS_Gaussian_UncertStd(domain_edot, [10000, 800, 1000, 1])

    edot_NB = R_IT2FS_Gaussian_UncertStd(domain_edot, [-2.5, 0.5, 0.1, 1])
    edot_NM = IT2FS_Gaussian_UncertStd(domain_edot,   [-1.5, 0.5, 0.1, 1])
    edot_NS = IT2FS_Gaussian_UncertStd(domain_edot,   [-0.7, 0.4, 0.1, 1])
    edot_ZO = IT2FS_Gaussian_UncertStd(domain_edot,   [ 0.0, 0.3, 0.1, 1])
    edot_PS = IT2FS_Gaussian_UncertStd(domain_edot,   [ 0.7, 0.4, 0.1, 1])
    edot_PM = IT2FS_Gaussian_UncertStd(domain_edot,   [ 1.5, 0.5, 0.1, 1])
    edot_PB = L_IT2FS_Gaussian_UncertStd(domain_edot, [ 2.5, 0.5, 0.1, 1])

    # kpp membership
    kpp_S = IT2FS_Gaussian_UncertStd(domain_kpp, [0, 0.2, 0.2, 1.])
    kpp_B = IT2FS_Gaussian_UncertStd(domain_kpp, [1, 0.2, 0.2, 1.])

    # kdp membership
    kdp_S = IT2FS_Gaussian_UncertStd(domain_kdp, [0, 0.2, 0.2, 1.])
    kdp_B = IT2FS_Gaussian_UncertStd(domain_kdp, [1, 0.2, 0.2, 1.])

    # alpha membership
    alpha_S = IT2FS_Gaussian_UncertStd(domain_alpha, [2, 0.1, 0.1, 1])
    alpha_MS = IT2FS_Gaussian_UncertStd(domain_alpha, [3, 0.1, 0.1, 1.])
    alpha_MB = IT2FS_Gaussian_UncertStd(domain_alpha, [4, 0.1, 0.1, 1.])
    alpha_B = IT2FS_Gaussian_UncertStd(domain_alpha, [5, 0.1, 0.1, 1.])

    # mu membership
    # mu_S = IT2FS_Gaussian_UncertStd(domain_mu, [2.5, 0.3, 0.2, 1])
    # mu_B = IT2FS_Gaussian_UncertStd(domain_mu, [3.0, 0.3, 0.2, 1])
    # mu_B = IT2FS_Gaussian_UncertStd(domain_mu, [3.5, 0.3, 0.2, 1])
    mu_S = IT2FS_Gaussian_UncertStd(domain_mu, [0.001, 0.0004, 0.002, 1])
    mu_B = IT2FS_Gaussian_UncertStd(domain_mu, [0.006, 0.0001, 0.001, 1])

    # eta membership
    eta_S = IT2FS_Gaussian_UncertStd(domain_eta, [650.0, 1, 1, 1])
    eta_M = IT2FS_Gaussian_UncertStd(domain_eta, [700.0, 1, 1, 1])
    eta_B = IT2FS_Gaussian_UncertStd(domain_eta, [750.0, 1, 1, 1])


##### Setup fuzzy system ############################
    myIT2FLS = IT2Mamdani(min_t_norm, max_s_norm)

    # Input
    myIT2FLS.add_input_variable("e")
    myIT2FLS.add_input_variable("edot")

    # Output
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


    return myIT2FLS, domain_e, domain_edot
