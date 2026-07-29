from pyit2fls import T1Mamdani, T1FS, gaussian_mf, rgaussian_mf, lgaussian_mf
from numpy import linspace

def myfuzzyT1_sys():
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
    # e_NB = T1FS(domain_e, gaussian_mf, [-1.0, 0.1, 1.])
    # e_NM = T1FS(domain_e, gaussian_mf, [-0.6667, 0.1, 1.])
    # e_NS = T1FS(domain_e, gaussian_mf, [-0.3333, 0.1, 1.])
    # e_ZO = T1FS(domain_e, gaussian_mf, [0, 0.1, 1.])
    # e_PS = T1FS(domain_e, gaussian_mf, [0.3333, 0.1, 1.])
    # e_PM = T1FS(domain_e, gaussian_mf, [0.6667, 0.1, 1.])
    # e_PB = T1FS(domain_e, gaussian_mf, [1.0, 0.1, 1.])

    e_NB = T1FS(domain_e, rgaussian_mf, [-0.1, 0.01, 1.])
    e_NM = T1FS(domain_e, gaussian_mf, [-0.06667, 0.01, 1.])
    e_NS = T1FS(domain_e, gaussian_mf, [-0.03333, 0.01, 1.])
    e_ZO = T1FS(domain_e, gaussian_mf, [0, 0.01, 1.])
    e_PS = T1FS(domain_e, gaussian_mf, [0.03333, 0.01, 1.])
    e_PM = T1FS(domain_e, gaussian_mf, [0.06667, 0.01, 1.])
    e_PB = T1FS(domain_e, lgaussian_mf, [0.1, 0.01, 1.])

    # dError membership
    edot_NB = T1FS(domain_edot, rgaussian_mf, [-2.5, 0.5, 1.])
    edot_NM = T1FS(domain_edot, gaussian_mf,  [-1.5, 0.5, 1.])
    edot_NS = T1FS(domain_edot, gaussian_mf,  [-0.7, 0.4, 1.])
    edot_ZO = T1FS(domain_edot, gaussian_mf,  [ 0.0, 0.3, 1.])
    edot_PS = T1FS(domain_edot, gaussian_mf,  [ 0.7, 0.4, 1.])
    edot_PM = T1FS(domain_edot, gaussian_mf,  [ 1.5, 0.5, 1.])
    edot_PB = T1FS(domain_edot, lgaussian_mf, [ 2.5, 0.5, 1.])

    # kpp membership
    kpp_S = T1FS(domain_kpp, gaussian_mf, [0, 0.5, 1])
    kpp_B = T1FS(domain_kpp, gaussian_mf, [1, 0.5, 1.])

    # kdp membership
    kdp_S = T1FS(domain_kdp, gaussian_mf, [0, 0.5, 1])
    kdp_B = T1FS(domain_kdp, gaussian_mf, [1, 0.5, 1.])

    # alpha membership
    alpha_S = T1FS(domain_alpha, gaussian_mf,[2, 0.1, 1])
    alpha_MS = T1FS(domain_alpha, gaussian_mf,[3, 0.1, 1])
    alpha_MB = T1FS(domain_alpha, gaussian_mf,[4, 0.1, 1])
    alpha_B = T1FS(domain_alpha, gaussian_mf,[5, 0.1, 1])

    mu_S = T1FS(domain_mu, gaussian_mf, [0.001, 0.001, 1])
    mu_B = T1FS(domain_mu, gaussian_mf, [0.006, 0.001, 1])

    # eta membership
    eta_S = T1FS(domain_eta, gaussian_mf, [650.0, 1, 1])
    eta_M = T1FS(domain_eta, gaussian_mf, [700.0, 1, 1])
    eta_B = T1FS(domain_eta, gaussian_mf, [750.0, 1, 1])

##### Setup fuzzy system ############################
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

##### Rules #####################################################
##### Rules #####################################################
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

    ##################################
    return myT1Mamdani, domain_e, domain_edot
