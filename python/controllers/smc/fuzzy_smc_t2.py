import numpy as np
import math
import serial
import time
import threading
import struct
import csv
import sys
from pathlib import Path
_python_dir = Path(__file__).resolve().parent
while _python_dir.name != "python":
    _python_dir = _python_dir.parent
_root_dir = _python_dir.parent
for _p in (_python_dir / "trajectories", _python_dir / "shared", _python_dir / "neurofuzzy"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
COLLECTED_DATA_DIR = _root_dir / "data" / "collected_data"
TRAINED_ARTIFACTS_DIR = _root_dir / "data" / "trained_artifacts"

from fuzzy_t2_system import myfuzzyT2_sys
from fuzzy_t1_system import myfuzzyT1_sys
from pyit2fls import crisp

# Robot geometry constants
ed = 90.0  # end effector
f = 70.0  # base
re = 250.0
rf = 175.0

# Trigonometric constants
sqrt3 = math.sqrt(3.0)
pi = math.pi
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60 = sqrt3
sin30 = 0.5
tan30 = 1 / sqrt3

csv_file = open(str(COLLECTED_DATA_DIR / "500helixFuzzySMCT2-10.csv"), 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Time","e1", "e2", "e3", "edot1","edot2", "edot3",
                     "eta1", "eta2", "eta3",  
                     "SetAngle1", "SetAngle2", "SetAngle3",
                     "ActualAngle1", "ActualAngle2", "ActualAngle3",
                     "X0", "Y0", "Z0",
                     "Actual_X", "Actual_Y", "Actual_Z",
                     "X_Set", "Y_Set", "Z_Set",
                     "refCurr1", "refCurr2", "refCurr3",
                     "ActualCurr1", "ActualCurr2", "ActualCurr3"])

# Serial setup (ubah 'COM3' ke port Arduino Anda)
# arduino = serial.Serial(port='COM3', baudrate=115200, timeout=1)


from helix_trajectory_realtime import (
    get_trajectory_sample,
    get_trajectory_index,
    TOTAL_TRAJECTORY_STEPS,
    SAMPLE_RATE,
)
# from circular_trajectory import run_trajectory
# from parabolic_trajectory_realtime import (
#     get_trajectory_sample,
#     get_trajectory_index,
#     TOTAL_TRAJECTORY_STEPS,
#     SAMPLE_RATE,
# )
# from oval_trajectory import run_trajectory
# from stepTraj import run_trajectory
# from stepTraj2 import run_trajectory
# from linear_ztrajectory import run_trajectory

myIT2FLS, domain_e, domain_edot = myfuzzyT2_sys()
def fuzzy_m1(e1, edot1):
    _, tr = myIT2FLS.evaluate({"e": e1, "edot": edot1})
    eta1 = crisp(tr["eta"])
    return eta1

def fuzzy_m2(e2, edot2):
    _, tr = myIT2FLS.evaluate({"e": e2, "edot": edot2})
    eta2 = crisp(tr["eta"])
    return eta2

def fuzzy_m3(e3, edot3):
    _, tr = myIT2FLS.evaluate({"e": e3, "edot": edot3})
    eta3 = crisp(tr["eta"])
    return eta3

# ==== Binary frame helpers (Arduino -> Python) ====
HDR = b"\xAA\x55"

def _xor_sum(b: bytes) -> int:
    x = 0
    for v in b:
        x ^= v
    return x & 0xFF

def _read_exact(ser, n: int):
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)

def read_binary_frame(ser):
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b == b"\xAA":
            b2 = ser.read(1)
            if b2 == b"\x55":
                break
    cnt_b = _read_exact(ser, 1)
    if not cnt_b:
        return None
    count = cnt_b[0]
    pay = _read_exact(ser, count * 4)
    if not pay:
        return None
    chk_b = _read_exact(ser, 1)
    if not chk_b:
        return None
    if _xor_sum(cnt_b + pay) != chk_b[0]:
        return None
    vals = list(struct.unpack("<" + "f"*count, pay))
    return vals

class SerCOM:
    def __init__(self, port='COM6', baudrate=115200):
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self.setangle1 = 0.0
        self.setangle2 = 0.0
        self.setangle3 = 0.0
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.angle3 = 0.0
        
        # --- STATE MACHINE ---
        # State bisa berupa: 'IDLE', 'STANDBY' (Motor ON, diam), 'RUNNING' (Trajectory jalan)
        self.motor_state = 'IDLE'

    def read_from_arduino(self):
        try:
            self.arduino.reset_input_buffer()
        except Exception:
            pass

        trajectory_start_time = None
        trajectory_finished = False
        last_traj_index = -1

        while True:
            try:
                # === TERIMA BINER ===
                vals = read_binary_frame(self.arduino)
                if not vals:
                    time.sleep(0.001)
                    continue
                if len(vals) != 15:
                    continue

                e1, e2, e3, edot1, edot2, edot3, \
                self.angle1, self.angle2, self.angle3, \
                refCurr1, refCurr2, refCurr3,\
                ActualCurr1, ActualCurr2, ActualCurr3 = vals

                # Pengganti fuzzy: nilai eta statis
                eta1 = fuzzy_m1(e1, edot1)
                eta2 = fuzzy_m2(e2, edot2)
                eta3 = fuzzy_m3(e3, edot3)

                # ==========================================
                # KONDISI 1: IDLE / STANDBY (Menekan S)
                # ==========================================
                if self.motor_state == 'IDLE' or self.motor_state == 'STANDBY':
                    # Ambil posisi pertama (index 0) dan diamkan di posisi itu
                    X0, Y0, Z0, _ = get_trajectory_sample(0)
                    theta1, theta2, theta3 = self.inverse_kinematics(X0, Y0, Z0)
                    self.setangle1 = theta1
                    self.setangle2 = theta2
                    self.setangle3 = theta3

                    data_to_send = f"{eta1:.4f},{eta2:.4f},{eta3:.4f},{self.setangle1:.4f},{self.setangle2:.4f},{self.setangle3:.4f}\n"
                    self.arduino.write(data_to_send.encode("ascii"))
                    
                    # Pastikan timer selalu reset agar waktu mulai nol saat di-Run
                    trajectory_start_time = None
                    # -> [!] CSV TIDAK DITULIS DI SINI <-

                # ==========================================
                # KONDISI 2: RUNNING (Menekan R)
                # ==========================================
                elif self.motor_state == 'RUNNING':
                    if trajectory_start_time is None:
                        trajectory_start_time = time.monotonic()

                    traj_elapsed = time.monotonic() - trajectory_start_time
                    traj_index = get_trajectory_index(traj_elapsed)

                    if traj_index <= last_traj_index:
                        traj_index = last_traj_index + 1
                    if traj_index >= TOTAL_TRAJECTORY_STEPS:
                        traj_index = TOTAL_TRAJECTORY_STEPS - 1

                    X0, Y0, Z0, traj_elapsed = get_trajectory_sample(traj_index)
                    last_traj_index = traj_index
                    theta1, theta2, theta3 = self.inverse_kinematics(X0, Y0, Z0)
                    self.setangle1 = theta1
                    self.setangle2 = theta2
                    self.setangle3 = theta3

                    data_to_send = f"{eta1:.4f},{eta2:.4f},{eta3:.4f},{self.setangle1:.4f},{self.setangle2:.4f},{self.setangle3:.4f}\n"
                    self.arduino.write(data_to_send.encode("ascii"))

                    # --- FK & PENULISAN CSV ---
                    xActual, yActual, zActual = self.delta_calc_fwd_actual()
                    xSet, ySet, zSet = self.delta_calc_fwd_set()
                    elapsed_time = round(traj_elapsed, 3)

                    csv_writer.writerow([
                        elapsed_time, e1, e2, e3, edot1, edot2, edot3,
                        eta1, eta2, eta3,
                        self.setangle1, self.setangle2, self.setangle3,
                        self.angle1, self.angle2, self.angle3,
                        X0, Y0, Z0,
                        xActual, yActual, zActual,
                        xSet, ySet, zSet,
                        refCurr1, refCurr2, refCurr3,
                        ActualCurr1, ActualCurr2, ActualCurr3
                    ])

                    # Jika selesai, matikan motor dan akhiri state
                    if traj_index >= TOTAL_TRAJECTORY_STEPS - 1 and not trajectory_finished:
                        trajectory_finished = True
                        self.motor_state = 'IDLE'
                        try:
                            self.arduino.write(b'X') # Perintah manual matikan Arduino
                        except Exception:
                            pass
                        print("\nTrajectory selesai. Motor dimatikan secara otomatis.")
                        break

            except Exception as error:
                print(f"Error reading loop: {error}")
                time.sleep(1)
                try:
                    self.arduino.write(b'X')
                except Exception:
                    pass
                break

    
    def delta_calc_fwd_actual(self):
        t = (f - ed) * tan30 / 2
        dtr = pi / 180.0

        angle1 = self.angle1*dtr
        angle2 = self.angle2*dtr
        angle3 = self.angle3*dtr

        y1 = -(t + rf * math.cos(angle1))
        z1 = -rf * math.sin(angle1)

        y2 = (t + rf * math.cos(angle2)) * sin30
        x2 = y2 * tan60
        z2 = -rf * math.sin(angle2)

        y3 = (t + rf * math.cos(angle3)) * sin30
        x3 = -y3 * tan60
        z3 = -rf * math.sin(angle3)

        dnm = (y2 - y1) * x3 - (y3 - y1) * x2

        w1 = y1**2 + z1**2
        w2 = x2**2 + y2**2 + z2**2
        w3 = x3**2 + y3**2 + z3**2

        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        a = a1**2 + a2**2 + dnm**2
        b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm**2)
        c = (b2 - y1 * dnm) ** 2 + b1**2 + dnm**2 * (z1**2 - re**2)

        d = b**2 - 4.0 * a * c
        if d < 0:
            return None, None, None  # Non-existing point

        zActual = -0.5 * (b + math.sqrt(d)) / a
        xActual = (a1 * zActual + b1) / dnm
        yActual = (a2 * zActual + b2) / dnm

        return xActual, yActual, zActual

    def delta_calc_fwd_set(self):
        t = (f - ed) * tan30 / 2
        dtr = pi / 180.0

        setangle1 = self.setangle1*dtr
        setangle2 = self.setangle2*dtr
        setangle3 = self.setangle3*dtr

        y1 = -(t + rf * math.cos(setangle1))
        z1 = -rf * math.sin(setangle1)

        y2 = (t + rf * math.cos(setangle2)) * sin30
        x2 = y2 * tan60
        z2 = -rf * math.sin(setangle2)

        y3 = (t + rf * math.cos(setangle3)) * sin30
        x3 = -y3 * tan60
        z3 = -rf * math.sin(setangle3)

        dnm = (y2 - y1) * x3 - (y3 - y1) * x2

        w1 = y1**2 + z1**2
        w2 = x2**2 + y2**2 + z2**2
        w3 = x3**2 + y3**2 + z3**2

        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        a = a1**2 + a2**2 + dnm**2
        b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm**2)
        c = (b2 - y1 * dnm) ** 2 + b1**2 + dnm**2 * (z1**2 - re**2)

        d = b**2 - 4.0 * a * c
        if d < 0:
            return None, None, None  # Non-existing point

        zSet = -0.5 * (b + math.sqrt(d)) / a
        xSet = (a1 * zSet + b1) / dnm
        ySet = (a2 * zSet + b2) / dnm

        return xSet, ySet, zSet
    
    def inverse_kinematics(self, x0, y0, z0):
        t = (f - ed) * tan30 / 2
        dtr = pi / 180.0

        def calc_angle_yz(x0, y0, z0):
            y1 = -0.5 * 0.57735 * f
            y0 -= 0.5 * 0.57735 * ed
            a = (x0**2 + y0**2 + z0**2 + rf**2 - re**2 - y1**2) / (2 * z0)
            b = (y1 - y0) / z0
            d = -(a + b * y1)**2 + rf * (b**2 * rf + rf)
            if d < 0:
                return 0.0  # atau raise error/handle sesuai kebutuhan
            yj = (y1 - a * b - math.sqrt(d)) / (b**2 + 1)
            zj = a + b * yj
            return math.degrees(math.atan(-zj / (y1 - yj)))

        theta1 = calc_angle_yz(x0, y0, z0)
        theta2 = calc_angle_yz(
            x0 * cos120 + y0 * sin120,
            y0 * cos120 - x0 * sin120,
            z0
        )
        theta3 = calc_angle_yz(
            x0 * cos120 - y0 * sin120,
            y0 * cos120 + x0 * sin120,
            z0
        )

        return theta1, theta2, theta3
    

if __name__ == "__main__":
    print("Program berjalan. Membaca data dari serial...")

    robot = SerCOM(port='COM6', baudrate=115200)

    read_thread = threading.Thread(target=robot.read_from_arduino, daemon=True)
    read_thread.start()

    try:
        while True:
            # Menu Kendali yang baru: S untuk Standby, R untuk Run, X untuk Stop
            command = input("\nKetik 'S' (Motor ON/Standby), 'R' (Run Trajectory), atau 'X' (Matikan Motor):\n").strip().upper()
            
            if command == 'S':
                robot.motor_state = 'STANDBY'
                robot.arduino.write(b'S')
                print("-> [S] Ditekan: Motor ON. Mengunci posisi initial (index 0). Menunggu 'R'...")
                
            elif command == 'R':
                if robot.motor_state == 'STANDBY':
                    robot.motor_state = 'RUNNING'
                    # Kita juga kirim R ke Arduino (sebagai sekadar penanda/sinkronisasi jika diperlukan di masa depan)
                    robot.arduino.write(b'R') 
                    print("-> [R] Ditekan: Trajectory berjalan. Merekam ke CSV...")
                else:
                    print("-> [!] Motor harus dihidupkan dulu dengan 'S' sebelum bisa 'R' (Run)!")

            elif command == 'X':
                robot.motor_state = 'IDLE'
                robot.arduino.write(b'X')
                print("-> [X] Ditekan: Motor DIMATIKAN secara manual.")
                break

            if not read_thread.is_alive():
                break

    except KeyboardInterrupt:
        print("Program dihentikan.")
    finally:
        robot.arduino.close()
        csv_file.close()
        print("Koneksi ke Arduino ditutup.")