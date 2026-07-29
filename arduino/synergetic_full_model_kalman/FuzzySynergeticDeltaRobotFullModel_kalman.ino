#include <Arduino.h>
#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.295779513082320877f

// ===== FORWARD DECLARATION (wajib di atas: Arduino auto-prototype) =====
// Tipe Kalman2 harus dikenal sebelum prototipe fungsi dibuat otomatis.
struct Kalman2 {
  float x[2];      // [pos, vel] estimasi
  float P[2][2];   // kovarians estimasi
};
inline void kalmanUpdate(Kalman2 &kf, float z, float dt);


// ======= Binary TX Helpers (Arduino -> Python) =======
static const uint8_t HDR0 = 0xAA;
static const uint8_t HDR1 = 0x55;

uint8_t xorsum(const uint8_t* p, size_t n) {
  uint8_t c = 0;
  while (n--) c ^= *p++;
  return c;
}

// Kirim 15 float (float32 little-endian):
// urutan: e1,e2,e3, edot1,edot2,edot3, angle1_deg,angle2_deg,angle3_deg, calculatedCurr1,calculatedCurr2,calculatedCurr3, current1,current2,current3
void sendBinary(float e1, float e2, float e3,
                float de1, float de2, float de3,
                float a1deg, float a2deg, float a3deg,
                float calculatedCurr1, float calculatedCurr2, float calculatedCurr3,
                float current1, float current2, float current3) {
  const uint8_t COUNT = 15;
  float payload[COUNT] = { e1, e2, e3, de1, de2, de3, a1deg, a2deg, a3deg, calculatedCurr1, calculatedCurr2, calculatedCurr3, current1, current2, current3 };

  SerialUSB.write(HDR0);
  SerialUSB.write(HDR1);
  SerialUSB.write(COUNT);
  SerialUSB.write(reinterpret_cast<const uint8_t*>(payload), COUNT * sizeof(float));

  uint8_t core[1 + COUNT * sizeof(float)];
  core[0] = COUNT;
  memcpy(core + 1, payload, COUNT * sizeof(float));
  uint8_t chk = xorsum(core, sizeof(core));
  SerialUSB.write(chk);
}


// =======================================================
// PIN DEFINISI
// =======================================================
const int safetySwitch1 = 9;
const int safetySwitch2 = 10;
const int safetySwitch3 = 11;

// Motor 1 Pins
const int motorPWM1 = 6;
const int motorDIR1 = 3;
const int encoderChA1 = 48;
const int encoderChB1 = 49;
const int currentSensorPin1 = A0;
float measuredCurrentBuffer1 = 0.0;

// Motor 2 Pins
const int motorPWM2 = 7;
const int motorDIR2 = 4;
const int encoderChA2 = 50;
const int encoderChB2 = 51;
const int currentSensorPin2 = A1;
float measuredCurrentBuffer2 = 0.0;

// Motor 3 Pins
const int motorPWM3 = 8;
const int motorDIR3 = 5;
const int encoderChA3 = 52;
const int encoderChB3 = 53;
const int currentSensorPin3 = A2;
float measuredCurrentBuffer3 = 0.0;

// =======================================================
// PARAMETER ENCODER
// =======================================================
const float encoderPPR = 5.0f;
const float gearRatio = 49.0f;
const float quadratureFactor = 4.0f;
const float countsPerRev = encoderPPR * gearRatio * quadratureFactor;
const float COUNTS_TO_RAD = (2.0f * PI) / countsPerRev;  // pre-computed

// =======================================================
// VARIABEL ENCODER (per motor)
// =======================================================
volatile int en_pulse_count1 = 0;
float countCopyBuffer1 = 0;
float actualAngleBuffer1 = 0.0;
volatile uint8_t prevAB1 = 0;

volatile int en_pulse_count2 = 0;
float countCopyBuffer2 = 0;
float actualAngleBuffer2 = 0.0;
volatile uint8_t prevAB2 = 0;

volatile int en_pulse_count3 = 0;
float countCopyBuffer3 = 0;
float actualAngleBuffer3 = 0.0;
volatile uint8_t prevAB3 = 0;

// =======================================================
// SETPOINT
// =======================================================
float setpoint_rad1 = 0.0, setpoint_rad2 = 0.0, setpoint_rad3 = 0.0;
float setpoint_dq1 = 0.0, setpoint_dq2 = 0.0, setpoint_dq3 = 0.0;     // dq_d
float setpoint_ddq1 = 0.0, setpoint_ddq2 = 0.0, setpoint_ddq3 = 0.0;  // ddq_d

// =======================================================
// PARAMETER FISIK ROBOT DELTA (multivariable)
// Sesuai simulasi Python (simulasiSynergeticFullRobotV4)
// =======================================================
const float GRAVITY = 9.81f;
const float L_A = 0.175f;          // upper arm length (m)
const float L_B = 0.25f;           // lower arm length (m)
const float M_PT = 0.317f;         // moving plate mass (kg)
const float I_A_VAL = 0.0443f;     // upper arm + motor inertia (diagonal)
const float M_A = 0.046f;          // upper arm mass (kg)
const float L_C = 0.08f;           // arm center of mass (m)
const float B_FRICTION = 0.1f;     // friction (only at plant - tidak dipakai di control law)

// Geometri delta (sesuai forward_kinematics Python)
const float ED_M = 0.09f;
const float F_M = 0.07f;
const float R_DIST = F_M - ED_M;

// Konstanta trigonometri
const float TAN30 = 0.5773502691896257f;   // 1/sqrt(3)
const float TAN60 = 1.7320508075688772f;   // sqrt(3)
const float SIN30 = 0.5f;

// Sudut penempatan ketiga lengan (radian)
const float PHI[3] = { 0.0f, 2.0943951023931953f, 4.188790204786391f }; // 0, 120, 240 deg
// Cosine & sine cache untuk rotasi z
const float COS_PHI[3] = { 1.0f, -0.5f, -0.5f };
const float SIN_PHI[3] = { 0.0f, 0.8660254037844386f, -0.8660254037844387f };

// =======================================================
// PARAMETER MOTOR (untuk current PID)
// =======================================================
const float Kt = 1.0f;   // torque constant
const float Ke = 1.0f;   // back EMF coefficient
const float Ra = 3.4f;   // armature resistance

// =======================================================
// PARAMETER SYNERGETIC (multivariable)
// =======================================================
const float c_manifold = 80.0f;   // error decay rate (shared - sesuai simulasi)
float mu1 = 0.001f;
float mu2 = 0.001f;
float mu3 = 0.001f;

// =======================================================
// VARIABEL FILTERED DERIVATIVE
// =======================================================
float dEpos1 = 0, dEpos2 = 0, dEpos3 = 0;
float dot_q1_f = 0.0f, dot_q2_f = 0.0f, dot_q3_f = 0.0f;

// =======================================================
// KALMAN FILTER (constant-velocity, 2-state per joint)
//   state x = [posisi(rad), kecepatan(rad/s)]
//   Tujuan: estimasi KECEPATAN yang bersih dari encoder kasar.
//   CATATAN: posisi mentah TETAP dipakai untuk posError & e1 (jujur).
//   Estimasi posisi Kalman (kf.x[0]) hanya dipakai internal filter.
//   (struct Kalman2 sudah dideklarasikan di atas file)
// =======================================================

// Q = derau proses (seberapa percaya model const-velocity),
// R = derau ukur (varians posisi encoder). Encoder kuantisasi
// 0.367 deg/count -> R disetel dari (q_step)^2.
// Naikkan R = lebih halus tapi lebih lag. Naikkan Q = lebih lincah tapi berisik.
float KF_Q_POS = 1e-6f;   // derau proses pada posisi
float KF_Q_VEL = 0.2f;    // derau proses pada kecepatan (besar = ikuti gerak cepat)
float KF_R     = 2.5e-3f; // (~0.367deg = 0.0064rad)^2 ~ 4.1e-5

Kalman2 kf1 = { {0, 0}, { {1, 0}, {0, 1} } };
Kalman2 kf2 = { {0, 0}, { {1, 0}, {0, 1} } };
Kalman2 kf3 = { {0, 0}, { {1, 0}, {0, 1} } };

// Satu langkah Kalman: prediksi const-velocity lalu koreksi dengan z (posisi ukur).
inline void kalmanUpdate(Kalman2 &kf, float z, float dt) {
  if (dt <= 0.0f) return;
  // --- PREDICT ---
  // x = F x ; F = [[1,dt],[0,1]]
  float px = kf.x[0] + dt * kf.x[1];
  float pv = kf.x[1];
  // P = F P F^T + Q
  float p00 = kf.P[0][0], p01 = kf.P[0][1], p10 = kf.P[1][0], p11 = kf.P[1][1];
  float np00 = p00 + dt * (p10 + p01) + dt * dt * p11 + KF_Q_POS;
  float np01 = p01 + dt * p11;
  float np10 = p10 + dt * p11;
  float np11 = p11 + KF_Q_VEL * dt;
  // --- UPDATE (H = [1,0], ukur posisi) ---
  float S = np00 + KF_R;
  float k0 = np00 / S;
  float k1 = np10 / S;
  float y = z - px;          // inovasi
  kf.x[0] = px + k0 * y;
  kf.x[1] = pv + k1 * y;
  kf.P[0][0] = (1.0f - k0) * np00;
  kf.P[0][1] = (1.0f - k0) * np01;
  kf.P[1][0] = np10 - k1 * np00;
  kf.P[1][1] = np11 - k1 * np01;
}

// =======================================================
// CACHE JACOBIAN (untuk hitung dJ/dt numerik)
// =======================================================
float J_prev[3][3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
float t_prev_J = 0.0f;

// =======================================================
// TARGET ARUS
// =======================================================
float targetCurrentBuffer1 = 0.0f;
float targetCurrentBuffer2 = 0.0f;
float targetCurrentBuffer3 = 0.0f;

// =======================================================
// PID ARUS -> PWM (per motor) - DIPERTAHANKAN
// =======================================================
double curKp1 = 20.0, curKi1 = 10.0, curKd1 = 0.015;
double curI_term1 = 0, prevCurrentError1 = 0;
double pwmOutputBuffer1 = 0;

double curKp2 = 20.0, curKi2 = 10.0, curKd2 = 0.015;
double curI_term2 = 0, prevCurrentError2 = 0;
double pwmOutputBuffer2 = 0;

double curKp3 = 20.0, curKi3 = 10.0, curKd3 = 0.015;
double curI_term3 = 0, prevCurrentError3 = 0;
double pwmOutputBuffer3 = 0;

// =======================================================
// TIMING
// =======================================================
unsigned long prevMicros = 0;
unsigned long lastPIDUpdate = 0;
const unsigned long pidIntervalMicros = 1000;
unsigned long lastPrintTime = 0;
const unsigned long printIntervalMillis = 40;  // 25 Hz monitoring

// =======================================================
// FLAG & IDLE
// =======================================================
bool runMotor = false;
const int minPWM = 255;
int currentIdlePWM = 10;
const int defaultIdlePWM = 200;
bool idlePWM_zeroMode = false;

// =======================================================
// FILTER DERIVATIVE
// =======================================================
inline float filteredDerivative(float currentValue, float previousValue,
                                float dt, float previousFiltered, float alpha) {
  if (dt <= 0.0f) return 0.0f;
  float raw = (currentValue - previousValue) / dt;
  return alpha * raw + (1.0f - alpha) * previousFiltered;
}

// =======================================================
// QUADRATURE DECODER
// =======================================================
inline int8_t quadDelta(uint8_t prevState, uint8_t currState) {
  static const int8_t table[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
  };
  return -table[(prevState << 2) | currState];
}

// =======================================================
// FORWARD KINEMATICS (analitik delta robot)
// Port dari simulasiSynergeticFullRobotV4.py
// =======================================================
void forwardKinematics(float q1, float q2, float q3, float* xOut, float* yOut, float* zOut) {
  const float t_val = (F_M - ED_M) * TAN30 * 0.5f;

  float y1 = -(t_val + L_A * cosf(q1));
  float z1 = -L_A * sinf(q1);

  float y2 = (t_val + L_A * cosf(q2)) * SIN30;
  float x2 = y2 * TAN60;
  float z2 = -L_A * sinf(q2);

  float y3 = (t_val + L_A * cosf(q3)) * SIN30;
  float x3 = -y3 * TAN60;
  float z3 = -L_A * sinf(q3);

  float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;
  if (fabsf(dnm) < 1e-6f) {
    *xOut = 0.0f; *yOut = 0.0f; *zOut = -0.3f;
    return;
  }

  float w1 = y1*y1 + z1*z1;
  float w2 = x2*x2 + y2*y2 + z2*z2;
  float w3 = x3*x3 + y3*y3 + z3*z3;

  float a1c = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  float b1c = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) * 0.5f;
  float a2c = -(z2 - z1) * x3 + (z3 - z1) * x2;
  float b2c = ((w2 - w1) * x3 - (w3 - w1) * x2) * 0.5f;

  float A = a1c*a1c + a2c*a2c + dnm*dnm;
  float B = 2.0f * (a1c * b1c + a2c * (b2c - y1 * dnm) - z1 * dnm * dnm);
  float C = (b2c - y1 * dnm) * (b2c - y1 * dnm) + b1c*b1c + dnm*dnm*(z1*z1 - L_B*L_B);

  float disc = B * B - 4.0f * A * C;
  if (disc < 0.0f) {
    *xOut = 0.0f; *yOut = 0.0f; *zOut = -0.3f;
    return;
  }

  float zActual = -0.5f * (B + sqrtf(disc)) / A;
  float xActual = (a1c * zActual + b1c) / dnm;
  float yActual = (a2c * zActual + b2c) / dnm;

  *xOut = xActual;
  *yOut = yActual;
  *zOut = zActual;
}

// =======================================================
// SOLVE 3x3 LINEAR SYSTEM (untuk Jacobian dan M_inv * rhs)
// Pakai Cramer's rule (paling efisien di Arduino untuk 3x3)
// =======================================================
inline float det3(const float M[3][3]) {
  return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
       - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
       + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

// Solve M * x = b, hasil di x[3]. Return false jika singular.
bool solve3x3(const float M[3][3], const float b[3], float x[3]) {
  float detM = det3(M);
  if (fabsf(detM) < 1e-9f) return false;
  float invDet = 1.0f / detM;

  float Mx[3][3], My[3][3], Mz[3][3];
  for (int i = 0; i < 3; i++) {
    Mx[i][0] = b[i]; Mx[i][1] = M[i][1]; Mx[i][2] = M[i][2];
    My[i][0] = M[i][0]; My[i][1] = b[i]; My[i][2] = M[i][2];
    Mz[i][0] = M[i][0]; Mz[i][1] = M[i][1]; Mz[i][2] = b[i];
  }
  x[0] = det3(Mx) * invDet;
  x[1] = det3(My) * invDet;
  x[2] = det3(Mz) * invDet;
  return true;
}

// =======================================================
// HITUNG JACOBIAN (J dan dJ/dt)
// =======================================================
void computeJacobian(float t_now, float q[3], float J_out[3][3], float dJ_out[3][3]) {
  // Posisi end-effector
  float px, py, pz;
  forwardKinematics(q[0], q[1], q[2], &px, &py, &pz);

  float S_mat[3][3];
  float S_b_diag[3];  // hanya diagonal yang non-zero
  // (S_b_diag asalnya matriks diagonal, jadi simpan sebagai vektor)

  for (int i = 0; i < 3; i++) {
    float ci = COS_PHI[i];
    float si = SIN_PHI[i];

    // v_b = [L_A*sin(q[i]), 0, L_A*cos(q[i])]
    float vb_x = L_A * sinf(q[i]);
    float vb_y = 0.0f;
    float vb_z = L_A * cosf(q[i]);
    // b_i = R_zi @ v_b
    float bi_x = ci * vb_x - si * vb_y;
    float bi_y = si * vb_x + ci * vb_y;
    float bi_z = vb_z;

    // v_s_off = [R_DIST + L_A*cos(q[i]), 0, -L_A*sin(q[i])]
    float vs_x = R_DIST + L_A * cosf(q[i]);
    float vs_y = 0.0f;
    float vs_z = -L_A * sinf(q[i]);
    // s_i = pos - R_zi @ v_s_off
    float Rvs_x = ci * vs_x - si * vs_y;
    float Rvs_y = si * vs_x + ci * vs_y;
    float Rvs_z = vs_z;
    float si_x = px - Rvs_x;
    float si_y = py - Rvs_y;
    float si_z = pz - Rvs_z;

    S_mat[i][0] = si_x;
    S_mat[i][1] = si_y;
    S_mat[i][2] = si_z;

    // Diagonal: dot(s_i, b_i)
    S_b_diag[i] = si_x * bi_x + si_y * bi_y + si_z * bi_z;
  }

  // J = -S^-1 * diag(S_b_diag)
  // Solve 3 sistem linear: S * col_k = -e_k * S_b_diag[k]
  for (int col = 0; col < 3; col++) {
    float rhs[3] = { 0, 0, 0 };
    rhs[col] = -S_b_diag[col];
    float jcol[3];
    if (!solve3x3(S_mat, rhs, jcol)) {
      // Fallback: identitas (proteksi singular)
      for (int r = 0; r < 3; r++) J_out[r][col] = (r == col) ? 1.0f : 0.0f;
    } else {
      for (int r = 0; r < 3; r++) J_out[r][col] = jcol[r];
    }
  }

  // dJ/dt numerik
  float dt_j = t_now - t_prev_J;
  if (dt_j > 1e-6f) {
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        dJ_out[r][c] = (J_out[r][c] - J_prev[r][c]) / dt_j;
  } else {
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++) dJ_out[r][c] = 0.0f;
  }

  // Update cache
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) J_prev[r][c] = J_out[r][c];
  t_prev_J = t_now;
}

// =======================================================
// HITUNG DYNAMICS MATRICES & SYNERGETIC CONTROL LAW
// Sesuai PDF Eq. 10:  tau = M*(ddq_d + c*e_dot + sigma/mu) + C*dq + tau_g
// =======================================================
void computeSynergeticControl(float q[3], float dq[3],
                              float qd[3], float dqd[3], float ddqd[3],
                              float mu[3], float t_now,
                              float tau_out[3]) {
  // === STEP 1: Compute Jacobian J dan dJ ===
  float J[3][3], dJ[3][3];
  computeJacobian(t_now, q, J, dJ);

  // === STEP 2: Compute M_hat = I_a + m_pt * J^T * J ===
  float JtJ[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float s = 0.0f;
      for (int k = 0; k < 3; k++) s += J[k][i] * J[k][j];
      JtJ[i][j] = s;
    }
  }
  float M_hat[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      M_hat[i][j] = M_PT * JtJ[i][j];
      if (i == j) M_hat[i][j] += I_A_VAL;
    }
  }

  // === STEP 3: Compute C_hat = m_pt * J^T * dJ ===
  float JtdJ[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float s = 0.0f;
      for (int k = 0; k < 3; k++) s += J[k][i] * dJ[k][j];
      JtdJ[i][j] = s;
    }
  }
  float C_hat[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) C_hat[i][j] = M_PT * JtdJ[i][j];

  // === STEP 4: Compute tau_g (gravity) ===
  // tau_g_endeff = -J^T * (m_pt * [0, 0, -g])
  // = -J^T * [0, 0, -m_pt*g] = J^T_col3 * m_pt * g
  // i.e. tau_g_endeff[i] = J[2][i] * M_PT * GRAVITY
  // tau_g_arm[i]   = -M_A * L_C * GRAVITY * cos(q[i])
  float tau_g[3];
  for (int i = 0; i < 3; i++) {
    float tau_g_endeff = J[2][i] * M_PT * GRAVITY;
    float tau_g_arm = -(M_A * L_C * GRAVITY * cosf(q[i]));
    tau_g[i] = tau_g_endeff + tau_g_arm;
  }

  // === STEP 5: Synergetic control law (PDF Eq. 10) ===
  // e = qd - q,  e_dot = dqd - dq,  sigma = c*e + e_dot
  // acc_cmd = ddqd + c*e_dot + sigma/mu  (PER JOINT, mu boleh berbeda)
  // tau = M_hat * acc_cmd + C_hat * dq + tau_g
  float acc_cmd[3];
  for (int i = 0; i < 3; i++) {
    float e_i = qd[i] - q[i];
    float ed_i = dqd[i] - dq[i];
    float sigma_i = c_manifold * e_i + ed_i;
    acc_cmd[i] = ddqd[i] + c_manifold * ed_i + sigma_i / mu[i];
  }

  // tau = M_hat * acc_cmd + C_hat * dq + tau_g
  for (int i = 0; i < 3; i++) {
    float t = 0.0f;
    for (int k = 0; k < 3; k++) {
      t += M_hat[i][k] * acc_cmd[k];
      t += C_hat[i][k] * dq[k];
    }
    tau_out[i] = t + tau_g[i];
  }
}

// =======================================================
// SETUP
// =======================================================
void setup() {
  SerialUSB.begin(115200);

  pinMode(safetySwitch1, INPUT_PULLUP);
  pinMode(safetySwitch2, INPUT_PULLUP);
  pinMode(safetySwitch3, INPUT_PULLUP);

  pinMode(encoderChA1, INPUT_PULLUP);
  pinMode(encoderChB1, INPUT_PULLUP);
  pinMode(motorPWM1, OUTPUT);
  pinMode(motorDIR1, OUTPUT);

  pinMode(encoderChA2, INPUT_PULLUP);
  pinMode(encoderChB2, INPUT_PULLUP);
  pinMode(motorPWM2, OUTPUT);
  pinMode(motorDIR2, OUTPUT);

  pinMode(encoderChA3, INPUT_PULLUP);
  pinMode(encoderChB3, INPUT_PULLUP);
  pinMode(motorPWM3, OUTPUT);
  pinMode(motorDIR3, OUTPUT);

  analogReadResolution(12);
  analogWriteResolution(12);

  prevAB1 = (digitalRead(encoderChA1) << 1) | digitalRead(encoderChB1);
  prevAB2 = (digitalRead(encoderChA2) << 1) | digitalRead(encoderChB2);
  prevAB3 = (digitalRead(encoderChA3) << 1) | digitalRead(encoderChB3);

  attachInterrupt(digitalPinToInterrupt(encoderChA1), encoderA_SR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderChB1), encoderB_SR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderChA2), encoderA_SR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderChB2), encoderB_SR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderChA3), encoderA_SR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderChB3), encoderB_SR3, CHANGE);

  prevMicros = micros();
  t_prev_J = 0.0f;
}

// =======================================================
// LOOP UTAMA
// =======================================================
void loop() {
  // --- SERIAL COMMAND ---
  if (SerialUSB.available()) {
    char ch = SerialUSB.peek();
    if (ch == 'S' || ch == 's') {
      SerialUSB.read();
      runMotor = true;
      setpoint_rad1 = setpoint_rad2 = setpoint_rad3 = 0.0f;
      setpoint_dq1 = setpoint_dq2 = setpoint_dq3 = 0.0f;
      setpoint_ddq1 = setpoint_ddq2 = setpoint_ddq3 = 0.0f;
      // Inisialisasi Kalman ke posisi encoder saat ini, kecepatan 0,
      // agar tidak ada lonjakan estimasi di awal gerak.
      kf1.x[0] = actualAngleBuffer1; kf1.x[1] = 0.0f;
      kf2.x[0] = actualAngleBuffer2; kf2.x[1] = 0.0f;
      kf3.x[0] = actualAngleBuffer3; kf3.x[1] = 0.0f;
    } else if (ch == 'X' || ch == 'x') {
      SerialUSB.read();
      runMotor = false;
    } else if (ch == 'R' || ch == 'r') {
      SerialUSB.read();
      // Reserved (Python controls trajectory)
    } else if (ch == 'I' || ch == 'i') {
      SerialUSB.read();
      idlePWM_zeroMode = !idlePWM_zeroMode;
      currentIdlePWM = idlePWM_zeroMode ? 0 : defaultIdlePWM;
    } else {
      // Format CSV: mu1,mu2,mu3,sp1_deg,sp2_deg,sp3_deg[,dq1,dq2,dq3,ddq1,ddq2,ddq3]
      // Backward-compatible: minimum 6 fields (mu + setpoint posisi).
      // Tambahan 6 field opsional: feed-forward dq_d dan ddq_d (rad/s, rad/s^2).
      String receivedData = SerialUSB.readStringUntil('\n');
      float tMu1, tMu2, tMu3, tSp1, tSp2, tSp3;
      float tDq1 = 0, tDq2 = 0, tDq3 = 0;
      float tDdq1 = 0, tDdq2 = 0, tDdq3 = 0;
      int parsed = sscanf(receivedData.c_str(),
                          "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                          &tMu1, &tMu2, &tMu3, &tSp1, &tSp2, &tSp3,
                          &tDq1, &tDq2, &tDq3, &tDdq1, &tDdq2, &tDdq3);
      if (parsed >= 6) {
        mu1 = tMu1; mu2 = tMu2; mu3 = tMu3;
        setpoint_rad1 = tSp1 * DEG_TO_RAD;
        setpoint_rad2 = tSp2 * DEG_TO_RAD;
        setpoint_rad3 = tSp3 * DEG_TO_RAD;
        if (parsed >= 12) {
          setpoint_dq1 = tDq1; setpoint_dq2 = tDq2; setpoint_dq3 = tDq3;
          setpoint_ddq1 = tDdq1; setpoint_ddq2 = tDdq2; setpoint_ddq3 = tDdq3;
        } else {
          // Tanpa feed-forward, anggap nol (controller masih jalan)
          setpoint_dq1 = setpoint_dq2 = setpoint_dq3 = 0.0f;
          setpoint_ddq1 = setpoint_ddq2 = setpoint_ddq3 = 0.0f;
        }
      }
    }
  }

  // --- SAFETY ---
  if (digitalRead(safetySwitch1) == LOW || digitalRead(safetySwitch2) == LOW || digitalRead(safetySwitch3) == LOW) {
    runMotor = false;
  }

  if (!runMotor) {
    stopMotor1();
    stopMotor2();
    stopMotor3();
    return;
  }

  // --- TIMING ---
  unsigned long now = micros();
  float dt = (now - prevMicros) * 1e-6f;
  if (dt <= 0.0f || dt > 0.02f) dt = 0.001f;
  prevMicros = now;
  float t_now_s = now * 1e-6f;

  // --- READ ENCODERS ---
  noInterrupts();
  countCopyBuffer1 = (float)en_pulse_count1;
  interrupts();
  actualAngleBuffer1 = countCopyBuffer1 * COUNTS_TO_RAD;

  noInterrupts();
  countCopyBuffer2 = (float)en_pulse_count2;
  interrupts();
  actualAngleBuffer2 = countCopyBuffer2 * COUNTS_TO_RAD;

  noInterrupts();
  countCopyBuffer3 = (float)en_pulse_count3;
  interrupts();
  actualAngleBuffer3 = countCopyBuffer3 * COUNTS_TO_RAD;

  // --- READ CURRENT SENSORS ---
  int raw1 = analogRead(currentSensorPin1);
  float voltage1 = raw1 * (3.3f / 4095.0f);
  measuredCurrentBuffer1 = (1.51f - voltage1) / 0.1f;

  int raw2 = analogRead(currentSensorPin2);
  float voltage2 = raw2 * (3.3f / 4095.0f);
  measuredCurrentBuffer2 = (1.51f - voltage2) / 0.1f;

  int raw3 = analogRead(currentSensorPin3);
  float voltage3 = raw3 * (3.3f / 4095.0f);
  measuredCurrentBuffer3 = (1.51f - voltage3) / 0.1f;

  // --- POSITION ERROR ---
  float posError1 = setpoint_rad1 - actualAngleBuffer1;
  float posError2 = setpoint_rad2 - actualAngleBuffer2;
  float posError3 = setpoint_rad3 - actualAngleBuffer3;

  // --- KECEPATAN via KALMAN (z = posisi mentah encoder) ---
  // Posisi mentah tetap dipakai untuk posError/e1; Kalman hanya
  // dipakai untuk mendapat kecepatan estimasi yang bersih.
  kalmanUpdate(kf1, actualAngleBuffer1, dt);
  kalmanUpdate(kf2, actualAngleBuffer2, dt);
  kalmanUpdate(kf3, actualAngleBuffer3, dt);

  dot_q1_f = kf1.x[1];   // kecepatan joint dari Kalman
  dot_q2_f = kf2.x[1];
  dot_q3_f = kf3.x[1];

  // --- TURUNAN ERROR ---
  // dEpos = d/dt(setpoint - q) = setpoint_dq - qdot.
  // Pakai kecepatan Kalman, BUKAN diferensiasi posError mentah.
  dEpos1 = setpoint_dq1 - dot_q1_f;
  dEpos2 = setpoint_dq2 - dot_q2_f;
  dEpos3 = setpoint_dq3 - dot_q3_f;

  // =======================================================
  // SYNERGETIC CONTROL LAW MULTIVARIABEL (PDF Eq. 10)
  // =======================================================
  float q_vec[3]  = { actualAngleBuffer1, actualAngleBuffer2, actualAngleBuffer3 };
  float dq_vec[3] = { dot_q1_f, dot_q2_f, dot_q3_f };
  float qd_vec[3] = { setpoint_rad1, setpoint_rad2, setpoint_rad3 };
  float dqd_vec[3]  = { setpoint_dq1, setpoint_dq2, setpoint_dq3 };
  float ddqd_vec[3] = { setpoint_ddq1, setpoint_ddq2, setpoint_ddq3 };
  float mu_vec[3] = { mu1, mu2, mu3 };
  float tau_vec[3];

  computeSynergeticControl(q_vec, dq_vec, qd_vec, dqd_vec, ddqd_vec,
                           mu_vec, t_now_s, tau_vec);

  // tau -> target arus
  targetCurrentBuffer1 = tau_vec[0] / Kt;
  targetCurrentBuffer2 = tau_vec[1] / Kt;
  targetCurrentBuffer3 = tau_vec[2] / Kt;

  // =======================================================
  // CURRENT PID -> PWM (DIPERTAHANKAN)
  // =======================================================
  if (runMotor && now - lastPIDUpdate >= pidIntervalMicros) {
    lastPIDUpdate = now;

    // Motor 1
    double currentError1 = targetCurrentBuffer1 - measuredCurrentBuffer1;
    double curP_term1 = curKp1 * currentError1;
    curI_term1 += curKi1 * currentError1 * dt;
    curI_term1 = constrain(curI_term1, -1500, 1500);
    double curD_term1 = curKd1 * (currentError1 - prevCurrentError1) / dt;
    prevCurrentError1 = currentError1;
    pwmOutputBuffer1 = curP_term1 + curI_term1 + curD_term1;
    pwmOutputBuffer1 = constrain(pwmOutputBuffer1, -4095, 4095);

    // Motor 2
    double currentError2 = targetCurrentBuffer2 - measuredCurrentBuffer2;
    double curP_term2 = curKp2 * currentError2;
    curI_term2 += curKi2 * currentError2 * dt;
    curI_term2 = constrain(curI_term2, -1500, 1500);
    double curD_term2 = curKd2 * (currentError2 - prevCurrentError2) / dt;
    prevCurrentError2 = currentError2;
    pwmOutputBuffer2 = curP_term2 + curI_term2 + curD_term2;
    pwmOutputBuffer2 = constrain(pwmOutputBuffer2, -4095, 4095);

    // Motor 3
    double currentError3 = targetCurrentBuffer3 - measuredCurrentBuffer3;
    double curP_term3 = curKp3 * currentError3;
    curI_term3 += curKi3 * currentError3 * dt;
    curI_term3 = constrain(curI_term3, -1500, 1500);
    double curD_term3 = curKd3 * (currentError3 - prevCurrentError3) / dt;
    prevCurrentError3 = currentError3;
    pwmOutputBuffer3 = curP_term3 + curI_term3 + curD_term3;
    pwmOutputBuffer3 = constrain(pwmOutputBuffer3, -4095, 4095);
  }

  // =======================================================
  // SERIAL MONITORING
  // =======================================================
  unsigned long nowMillis = millis();
  if (runMotor && nowMillis - lastPrintTime >= printIntervalMillis) {
    lastPrintTime = nowMillis;
    float angle1_deg = actualAngleBuffer1 * RAD_TO_DEG;
    float angle2_deg = actualAngleBuffer2 * RAD_TO_DEG;
    float angle3_deg = actualAngleBuffer3 * RAD_TO_DEG;

    float volt1 = (pwmOutputBuffer1 / 4095.0f) * 12.0f;
    float volt2 = (pwmOutputBuffer2 / 4095.0f) * 12.0f;
    float volt3 = (pwmOutputBuffer3 / 4095.0f) * 12.0f;

    float backEmf1 = Ke * dot_q1_f;
    float backEmf2 = Ke * dot_q2_f;
    float backEmf3 = Ke * dot_q3_f;

    float calculatedPidCurrent1 = (volt1 - backEmf1) / Ra;
    float calculatedPidCurrent2 = (volt2 - backEmf2) / Ra;
    float calculatedPidCurrent3 = (volt3 - backEmf3) / Ra;

    sendBinary(
      posError1, posError2, posError3,
      dEpos1, dEpos2, dEpos3,
      angle1_deg, angle2_deg, angle3_deg,
      calculatedPidCurrent1, calculatedPidCurrent2, calculatedPidCurrent3,
      measuredCurrentBuffer1, measuredCurrentBuffer2, measuredCurrentBuffer3);
  }

  // =======================================================
  // OUTPUT PWM
  // =======================================================
  if (runMotor) {
    int pwmVal1 = abs((int)pwmOutputBuffer1);
    if (fabs(targetCurrentBuffer1) > 0.01 && pwmVal1 < minPWM) pwmVal1 = minPWM;
    digitalWrite(motorDIR1, -pwmOutputBuffer1 >= 0 ? HIGH : LOW);
    analogWrite(motorPWM1, pwmVal1);

    int pwmVal2 = abs((int)pwmOutputBuffer2);
    if (fabs(targetCurrentBuffer2) > 0.01 && pwmVal2 < minPWM) pwmVal2 = minPWM;
    digitalWrite(motorDIR2, -pwmOutputBuffer2 >= 0 ? HIGH : LOW);
    analogWrite(motorPWM2, pwmVal2);

    int pwmVal3 = abs((int)pwmOutputBuffer3);
    if (fabs(targetCurrentBuffer3) > 0.01 && pwmVal3 < minPWM) pwmVal3 = minPWM;
    digitalWrite(motorDIR3, -pwmOutputBuffer3 >= 0 ? HIGH : LOW);
    analogWrite(motorPWM3, pwmVal3);
  } else {
    analogWrite(motorPWM1, currentIdlePWM); digitalWrite(motorDIR1, LOW);
    pwmOutputBuffer1 = 0; curI_term1 = 0;

    analogWrite(motorPWM2, currentIdlePWM); digitalWrite(motorDIR2, LOW);
    pwmOutputBuffer2 = 0; curI_term2 = 0;

    analogWrite(motorPWM3, currentIdlePWM); digitalWrite(motorDIR3, LOW);
    pwmOutputBuffer3 = 0; curI_term3 = 0;
  }
}

// =======================================================
// ENCODER ISR
// =======================================================
void updateEncoder1() {
  uint8_t curr = (digitalRead(encoderChA1) << 1) | digitalRead(encoderChB1);
  en_pulse_count1 += quadDelta(prevAB1, curr);
  prevAB1 = curr;
}
void updateEncoder2() {
  uint8_t curr = (digitalRead(encoderChA2) << 1) | digitalRead(encoderChB2);
  en_pulse_count2 += quadDelta(prevAB2, curr);
  prevAB2 = curr;
}
void updateEncoder3() {
  uint8_t curr = (digitalRead(encoderChA3) << 1) | digitalRead(encoderChB3);
  en_pulse_count3 += quadDelta(prevAB3, curr);
  prevAB3 = curr;
}

void encoderA_SR1() { updateEncoder1(); }
void encoderB_SR1() { updateEncoder1(); }
void encoderA_SR2() { updateEncoder2(); }
void encoderB_SR2() { updateEncoder2(); }
void encoderA_SR3() { updateEncoder3(); }
void encoderB_SR3() { updateEncoder3(); }

void stopMotor1() { analogWrite(motorPWM1, 0); digitalWrite(motorDIR1, LOW); }
void stopMotor2() { analogWrite(motorPWM2, 0); digitalWrite(motorDIR2, LOW); }
void stopMotor3() { analogWrite(motorPWM3, 0); digitalWrite(motorDIR3, LOW); }
