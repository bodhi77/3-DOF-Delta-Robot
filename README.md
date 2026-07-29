# IT2-NFSC Delta Robot — Usage Guide

This codebase implements the **Interval Type-2 Neuro-Fuzzy Synergetic Control (IT2-NFSC)** thesis for a 3-DOF delta robot, along with 10 comparable controllers (PID, SMC, Synergetic Control, and their fuzzy/neuro-fuzzy variants). Full design reference is in Chapters 3 and 5 of the thesis (`CLAUDE OUTPUTS/THESIS/sections/ch3_method.tex` and `ch5_IT2-NFSC.tex`).

## Table of Contents
1. [Folder structure](#folder-structure)
2. [Installation](#installation)
3. [Controller map (all 11)](#controller-map-all-11)
4. [End-to-end workflow](#end-to-end-workflow)
5. [Running each stage](#running-each-stage)
6. [Deploying to hardware](#deploying-to-hardware)
7. [Arduino firmware](#arduino-firmware)
8. [Note on `archive/`](#note-on-archive)
9. [Troubleshooting](#troubleshooting)

---

## Folder structure

```
FULL CODE/
├── arduino/                     Arduino firmware (per controller)
├── python/
│   ├── controllers/{pid,smc,synergetic}/   baseline + fuzzy T1/T2 (deploy-ready, serial-driven)
│   ├── neurofuzzy/               robot dynamics core + IT2-NFSC/T1-NFSC training & deployment
│   ├── simulation/                offline simulation (dataset generation, 11-controller benchmark)
│   ├── trajectories/              trajectory generators (helix, lissajous, step)
│   ├── impulse_tests/             disturbance-rejection scenarios (S1: torque impulse, S2: end-effector force)
│   ├── optimization/              grid search for PID & synergetic baseline gains
│   ├── shared/                    fuzzy T1/T2 modules shared across all controllers
│   ├── utils/                     utilities (export data for MATLAB, downsampling, etc.)
│   └── archive/                   old drafts/experiments, NOT maintained (see note below)
├── data/
│   ├── collected_data/            csv/rar logs from every run (real hardware & simulation)
│   └── trained_artifacts/         trained IT2-MNFS/T1-MNFS models (.joblib) + metadata
└── results/
    ├── training_convergence/      PSO training convergence plots (mamdani_t1, mamdani_t2, tsk, synergetic_delta)
    ├── optimization/               grid search outputs (control_surface.csv, etc.)
    ├── controller_sim/             plots & csv from each controller's simulation runs
    └── data_processing_staging/    staging csv for manual data post-processing (downsampling, etc.)
```

Every Python script above has been fixed so its paths automatically follow this structure — **you no longer need to keep every file in one flat folder like before.** Each script resolves its own `python/` root via `__file__`, so it can be run directly with `python path/to/script.py` from anywhere.

## Installation

Requires Python 3.9+ (this code was developed and tested on Python 3.9). Install dependencies:

```bash
pip install numpy scipy pandas matplotlib scikit-learn joblib pyserial pyit2fls
```

For firmware: **Arduino IDE** with an **Arduino DUE** board (used in the thesis to drive 3 planetary-geared DC motors over serial — see Section 5.3).

## Controller map (all 11)

| Controller (thesis) | Family | Baseline / grid search | Manual fuzzy (T1/IT2) | Neuro-fuzzy (PSO) |
|---|---|---|---|---|
| **IT2-NFSC*** (proposed) | Synergetic | — | — | `neurofuzzy/train_it2_nfsc.py` |
| T1-NFSC | Synergetic | — | — | `neurofuzzy/train_t1_nfsc.py` |
| SC | Synergetic | `optimization/synergetic_optimization.py` | — | — |
| T1-FSC | Synergetic | — | `controllers/synergetic/fuzzy_synergetic_t1.py` | — |
| IT2-FSC | Synergetic | — | `controllers/synergetic/fuzzy_synergetic_t2.py` | — |
| PID | PID | `optimization/pid_optimization.py` | — | — |
| T1-FPID | PID | — | `controllers/pid/fuzzy_pid_t1.py` | — |
| IT2-FPID | PID | — | `controllers/pid/fuzzy_pid_t2.py` | — |
| SMC | SMC | (same grid-search style as PID; constant gain lives in `controllers/smc/static_eta_smc.py`) | — | — |
| T1-FSMC | SMC | — | `controllers/smc/fuzzy_smc_t1.py` | — |
| IT2-FSMC | SMC | — | `controllers/smc/fuzzy_smc_t2.py` | — |

*The plain-mathematical baselines (PID, SC, SMC) with constant gains live in `controllers/pid/static_pid.py`, `controllers/synergetic/static_mu_synergetic.py`, `controllers/smc/static_eta_smc.py`.*

## End-to-end workflow

```
1. simulation/simulate_full_robot_v8.py
        │  (5 episodes, randomized payload + trajectory, per-timestep grid search for mu)
        ▼
   results/training_convergence/synergetic_delta/dataset_synergetic_delta_v8.csv
        │
        ├──► 2a. neurofuzzy/train_it2_nfsc.py   ──► data/trained_artifacts/it2mamdani_pso_model.joblib   (IT2-NFSC, proposed)
        └──► 2b. neurofuzzy/train_t1_nfsc.py    ──► data/trained_artifacts/t1mamdani_pso_model.joblib    (T1-NFSC, comparison)
        ▼
3. simulation/simulate_helix_neurofuzzy_revised.py
        (closed-loop validation of every controller on the helix trajectory, Stage 3 — software only, 1 kHz)
        ▼
4. neurofuzzy/deploy_trained_controller.py
        (Stage 4 — runs on the physical robot via Arduino DUE, ~25 Hz)
```

Comparable controllers (PID, SMC, SC, and their manual fuzzy variants) need no training — just run them directly (see [Running each stage](#running-each-stage)).

## Running each stage

### 1. Generate the training dataset (IT2-NFSC / T1-NFSC only)

```bash
python "FULL CODE/python/simulation/simulate_full_robot_v8.py"
```

Runs a full delta-robot simulation (5 episodes, randomized payload & trajectory, 100-candidate grid search for mu at every timestep — see Algorithm 5.2 in the thesis). Output is saved automatically to `results/training_convergence/synergetic_delta/`:
- `dataset_synergetic_delta_v8.csv` — dataset (e, de, optimal mu) for training
- `stats_synergetic_delta_v8.csv`, `plot_synergetic_delta_v8.png` — summary & plot

`simulate_full_robot_v7.py` / `simulate_full_robot_v7_dtaufilter.py` are the earlier (v7) revisions, kept for comparison/reproducibility.

### 2. Train the IT2-MNFS / T1-MNFS (PSO)

```bash
python "FULL CODE/python/neurofuzzy/train_it2_nfsc.py"   # proposed method (IT2-NFSC)
python "FULL CODE/python/neurofuzzy/train_t1_nfsc.py"    # comparison baseline (T1-NFSC)
```

Loads the dataset from step 1, splits train/val/test 70/15/15, and trains with PSO (50 particles, 150 iterations — see Algorithm 5.2 & Section 5.2.4). Output:
- `data/trained_artifacts/{it2,t1}mamdani_pso_model.joblib` + `.json` metadata
- `results/training_convergence/mamdani_t1/` or `mamdani_t2/` — PSO convergence & prediction-vs-actual plots

Set `CONFIG["rules_sweep"] = True` inside the script to compare rule counts (4/9/16/25) in one run.

### 3. Grid search for baselines (plain PID, SC, SMC)

```bash
python "FULL CODE/python/optimization/pid_optimization.py"
python "FULL CODE/python/optimization/synergetic_optimization.py"
```

Searches for the best constant gains using the modified ITAE cost (Eq. 5.20 in the thesis). `pid_optimization.py` simulates the motor + cascade PID on its own (no external dependency); the resulting gains are then hand-copied into the `Kp/Ki/Kd` constants in `controllers/pid/static_pid.py`. `synergetic_optimization.py` uses the robot dynamics model from `neurofuzzy/robot_dynamics_core.py`, with plots saved to `results/optimization/`.

### 4. Simulation validation (11-controller benchmark on the helix trajectory)

```bash
python "FULL CODE/python/simulation/simulate_helix_neurofuzzy_revised.py"
```

This is the thesis's **Stage 3** (Section 5.3): closed-loop software validation at 1 kHz, using the same helix trajectory as Chapter 6. It auto-loads the trained model from `data/trained_artifacts/` (auto-detects IT2 or T1). Output goes to `results/controller_sim/neuro_fuzzy_it2_nfsc/`.

For the other comparable controllers, run the matching simulation/controller file directly:
- `simulation/simulate_synergetic_control.py` — SC (constant mu)
- `simulation/simulate_fuzzy_synergetic_t1.py` / `simulate_fuzzy_synergetic_t2.py` — T1-FSC / IT2-FSC
- `controllers/**/*.py` — can be run either as a simulation OR a hardware run (see the deployment section below)

### 5. Disturbance-rejection tests (impulse tests)

```bash
python "FULL CODE/python/impulse_tests/impulse_s1_control_torque.py"   # S1: torque impulse
python "FULL CODE/python/impulse_tests/impulse_s2_ee_force.py"         # S2: end-effector force impulse
```

Per-controller variants exist as `impulse_s{1,2}_{fuzzy_t2,mamdani_t1,static_mu}.py`. All of them save output automatically to the matching `results/controller_sim/` subfolder (e.g. `impulse_combined/` for the combined log, `fuzzy_t1/` for controller-specific plots).

## Deploying to hardware

**Prerequisite**: the Arduino DUE must already be flashed with the matching sketch from `arduino/` (see the table below), connected via USB, with its COM port known.

```bash
python "FULL CODE/python/neurofuzzy/deploy_trained_controller.py"
```

This is the thesis's **Stage 4** (Section 5.3) — it auto-detects an IT2 or T1 model from `data/trained_artifacts/`, reads motor state over a binary serial frame from the Arduino, computes the per-joint mu, and sends it back every cycle (~25 Hz). Before running:
1. Open the file and change `port='COM6'` in the `SerCOM(port='COM6', ...)` line to match your Arduino's port.
2. Run the script, then type `S` in the terminal for motor ON/standby, `R` to start the trajectory, `X` to stop.
3. The log is saved automatically to `data/collected_data/1000helixNeuroMamdaniT1-1.csv` (rename the output file in the script if you want a separate log per run).

The other controllers (`controllers/**/*.py`) share the same serial structure (a `SerCOM` class, default port `COM6`) — just run whichever controller file you need.

## Arduino firmware

| `arduino/` folder | Used with |
|---|---|
| `pid/` | Baseline PID (host: `controllers/pid/*.py`) |
| `smc/` | Baseline SMC (host: `controllers/smc/*.py`) |
| `synergetic_full_model/` | Base synergetic control |
| `synergetic_full_model_rev/` | Latest revision — used by `deploy_trained_controller.py` & `controllers/synergetic/*.py` |
| `synergetic_full_model_kalman/` | Variant with a Kalman filter (extra experiment) |
| `synergetic_quadrature/` | Variant with quadrature encoder reading |

Upload the matching `.ino` sketch via the Arduino IDE to the Arduino DUE before running the corresponding Python script on the host.

## Note on `archive/`

`python/archive/` contains drafts/experiments that are **not part of the official thesis pipeline** and whose paths/imports were **not fixed** (they still use the old paths and will not run as-is):
- `own_rules_t1_draft.py`, `own_rules_t2_simple_draft.py` — alternative fuzzy rule-base drafts
- `train_tsk_t2_experimental.py` — TSK model experiment (not part of the final 11 controllers)
- `deploy_legacy_v2.py` — older version of `deploy_trained_controller.py`, IT2-only (superseded by the flexible version that auto-detects T1/IT2)

Kept for historical reference only. **Do not use these to reproduce thesis results.**

> Correction note: `shared/fuzzy_t1_system.py` and `shared/fuzzy_t2_system.py` (formerly `MyFuzzyT1.py`/`MyFuzzyT2.py`) were briefly mistaken for drafts too, but they are actually core modules imported by nearly every fuzzy controller (`controllers/`, `simulation/`, `impulse_tests/`) to compute the error/error-rate fuzzy sets. They have been moved to `shared/` and every importer has been fixed accordingly.

## Troubleshooting

- **`ModuleNotFoundError: No module named 'pyit2fls'`** → `pip install pyit2fls`. If a different pip version has a different internal structure (private attribute names changed), check the `_unmangle` patch near the top of `deploy_trained_controller.py`.
- **`FileNotFoundError` when loading a model** → make sure training (step 2) has been run first; the model must exist under `data/trained_artifacts/`.
- **A simulation/controller script can't find `helix_trajectory_realtime` or another fuzzy module** → every script has an automatic `sys.path` bootstrap that locates the `python/` folder from its own file location; if it still fails, make sure this folder layout (`arduino/`, `python/`, `data/`, `results/`) hasn't been partially moved or renamed.
- **Serial won't connect to the Arduino** → check the COM port in Device Manager (Windows), update `port='COM6'` to the actual port, and make sure the baud rate (115200) matches the uploaded sketch.
