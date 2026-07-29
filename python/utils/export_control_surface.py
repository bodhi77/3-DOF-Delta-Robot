# export_control_surface.py
# ---------------------------------------------------------------------
# STEP 1 (run this ONCE, in Python, in the same folder as your sim).
# It asks the trained IT2-MNFS model "for this (e, e_dot), what mu?" over a
# whole grid, and writes the answers to  control_surface.csv  so MATLAB can
# draw the surface. No robot run needed -- it just probes the model.
# ---------------------------------------------------------------------
import csv
import math
import sys
from pathlib import Path
import numpy as np

_python_dir = Path(__file__).resolve().parent
while _python_dir.name != "python":
    _python_dir = _python_dir.parent
_root_dir = _python_dir.parent
sys.path.insert(0, str(_python_dir / "impulse_tests"))
_OUT_CSV = _root_dir / "results" / "optimization" / "control_surface.csv"

# Reuse the loader already in your simulation file (any of the impulse scripts
# has load_neurofuzzy). Change the module name if yours is different.
from impulse_s2_ee_force import load_neurofuzzy

predict_mu = load_neurofuzzy()          # the trained model as a mu-calculator

# Range of situations to test (in DEGREES). Match this to the error / error-rate
# range the model was trained on (the X_LO..X_HI in your model metadata).
E_DEG  = np.linspace(-20, 20, 80)       # error
DE_DEG = np.linspace(-20, 20, 80)       # error rate

with open(_OUT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["e_deg", "edot_deg", "mu"])
    for e in E_DEG:                      # e on the outside
        for de in DE_DEG:               # e_dot on the inside
            mu = predict_mu(math.radians(e), math.radians(de))   # model takes RADIANS
            w.writerow([e, de, mu])

print("Done -> control_surface.csv  ({} rows). Now run plot_control_surface.m in MATLAB."
      .format(E_DEG.size * DE_DEG.size))
