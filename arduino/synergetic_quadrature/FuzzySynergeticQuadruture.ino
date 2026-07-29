#include <Arduino.h>
#define DEG_TO_RAD 0.017453292519943295f

// ======= Binary TX Helpers (Arduino -> Python) =======
static const uint8_t HDR0 = 0xAA;
static const uint8_t HDR1 = 0x55;

uint8_t xorsum(const uint8_t* p, size_t n) {
  uint8_t c = 0;
  while (n--) c ^= *p++;
  return c;
}

// Kirim 15 float (float32 little-endian):
// urutan: e1,e2,e3, edot1,edot2,edot3, angle1_deg,angle2_deg,angle3_deg, calculatedCurr,volt2,volt3, current1,current2,current3
void sendBinary(float e1, float e2, float e3,
                float de1, float de2, float de3,
                float a1deg, float a2deg, float a3deg,
                float calculatedCurr1, float calculatedCurr2, float calculatedCurr3,
                float current1, float current2, float current3) {
  const uint8_t COUNT = 15;
  float payload[COUNT] = { e1, e2, e3, de1, de2, de3, a1deg, a2deg, a3deg, calculatedCurr1, calculatedCurr2, calculatedCurr3, current1, current2, current3 };

  // Header
  SerialUSB.write(HDR0);
  SerialUSB.write(HDR1);

  // COUNT
  SerialUSB.write(COUNT);

  // PAYLOAD (COUNT * 4 bytes), float32 LE di Arduino (umumnya little-endian)
  SerialUSB.write(reinterpret_cast<const uint8_t*>(payload), COUNT * sizeof(float));

  // Checksum: XOR atas (COUNT + PAYLOAD)
  uint8_t core[1 + COUNT * sizeof(float)];
  core[0] = COUNT;
  memcpy(core + 1, payload, COUNT * sizeof(float));
  uint8_t chk = xorsum(core, sizeof(core));
  SerialUSB.write(chk);
}


// =======================================================
// PIN DEFINISI
// =======================================================
// Switch Pengaman
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
// PARAMETER ENCODER (sama untuk semua motor)
// =======================================================
const float encoderPPR = 5.0;
const float gearRatio = 49.0;
const float quadratureFactor = 4.0;  // [REVISI]: 4x Quadrature
const float countsPerRev = encoderPPR * gearRatio * quadratureFactor;

// =======================================================
// VARIABEL ENCODER (per motor)
// =======================================================
// Motor 1
volatile int en_pulse_count1 = 0;
float countCopyBuffer1 = 0;
float actualAngleBuffer1 = 0.0;
volatile uint8_t prevAB1 = 0;  // [REVISI]: State sebelumnya

// Motor 2
volatile int en_pulse_count2 = 0;
float countCopyBuffer2 = 0;
float actualAngleBuffer2 = 0.0;
volatile uint8_t prevAB2 = 0;  // [REVISI]: State sebelumnya

// Motor 3
volatile int en_pulse_count3 = 0;
float countCopyBuffer3 = 0;
float actualAngleBuffer3 = 0.0;
volatile uint8_t prevAB3 = 0;  // [REVISI]: State sebelumnya

// =======================================================
// SETPOINT dan PARAMETER SMC per motor
// =======================================================
float setpoint_deg1 = 0.0, setpoint_deg2 = 0.0, setpoint_deg3 = 0.0;
float setpoint_rad1 = 0.0, setpoint_rad2 = 0.0, setpoint_rad3 = 0.0;
float setpoint_counts1 = 0.0, setpoint_counts2 = 0.0, setpoint_counts3 = 0.0;

// Parameter Dinamika Motor
const float J = 0.0443;   // Inertia
const float b = 0.01859;  // Friction
const float Kt = 1.0;     // Torque constant
const float Ke = 1.0;     // Back EMF coefficient
const float Ra = 3.4;     // Armature ressistance

// Parameter Synergetic
// Motor 1
const float c1 = 80.0;  // Error decay rate motor 1
float mu1 = 0.001;      // Time constant manifold

// Motor 1
const float c2 = 80.0;  // Error decay rate motor 1
float mu2 = 0.001;      // Time constant manifold

// Motor 1
const float c3 = 80.0;  // Error decay rate motor 1
float mu3 = 0.001;      // Time constant manifold

float dEpos1 = 0;
float dEpos2 = 0;
float dEpos3 = 0;

// Variabel global untuk filter kecepatan sudut
float dot_q1_f = 0.0;
float dot_q2_f = 0.0;
float dot_q3_f = 0.0;

const float minStartCurrent = 2.5;  // Menjaga agar motor segera bergerak

// Target arus untuk masing-masing motor
float targetCurrentBuffer1 = 0.0;
float targetCurrentBuffer2 = 0.0;
float targetCurrentBuffer3 = 0.0;

// =======================================================
// PID ARUS → OUTPUT PWM (per motor)
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
// WAKTU UNTUK PID & LOOP
// =======================================================
unsigned long prevMicros = 0;
unsigned long lastPIDUpdate = 0;
const unsigned long pidIntervalMicros = 1000;
unsigned long lastPrintTime = 0;
const unsigned long printIntervalMillis = 40;  // 100 Hz monitoring

static unsigned long lastMillis = 0;
static unsigned long lastMicros = 0;

// =======================================================
// TOGGLE RUN FLAG
// =======================================================
bool runMotor = false;

// =======================================================
// PWM Deadzone dan Idle
// =======================================================
const int minPWM = 255;
int currentIdlePWM = 10;         // Aktif digunakan di loop
const int defaultIdlePWM = 200;  // Nilai default untuk reset toggle
bool idlePWM_zeroMode = false;   // Status toggle

// Fungsi filter derivative (Low-pass)
inline float filteredDerivative(float currentValue, float previousValue, float dt, float previousFiltered, float alpha) {
  if (dt <= 0.0f) return 0.0f;
  float raw = (currentValue - previousValue) / dt;
  return alpha * raw + (1.0f - alpha) * previousFiltered;
}

// [REVISI]: Lookup table untuk Quadrature Decoder
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
// SETUP
// =======================================================
void setup() {
  // Ganti Serial menjadi SerialUSB
  SerialUSB.begin(115200);

  // Setup pin safety switch dengan pull-up internal
  pinMode(safetySwitch1, INPUT_PULLUP);
  pinMode(safetySwitch2, INPUT_PULLUP);
  pinMode(safetySwitch3, INPUT_PULLUP);

  // [REVISI]: Setup pin untuk Motor 1
  pinMode(encoderChA1, INPUT_PULLUP);
  pinMode(encoderChB1, INPUT_PULLUP);
  pinMode(motorPWM1, OUTPUT);
  pinMode(motorDIR1, OUTPUT);

  // [REVISI]: Setup pin untuk Motor 2
  pinMode(encoderChA2, INPUT_PULLUP);
  pinMode(encoderChB2, INPUT_PULLUP);
  pinMode(motorPWM2, OUTPUT);
  pinMode(motorDIR2, OUTPUT);

  // [REVISI]: Setup pin untuk Motor 3
  pinMode(encoderChA3, INPUT_PULLUP);
  pinMode(encoderChB3, INPUT_PULLUP);
  pinMode(motorPWM3, OUTPUT);
  pinMode(motorDIR3, OUTPUT);

  analogReadResolution(12);
  analogWriteResolution(12);

  // [REVISI]: Baca state awal encoder
  prevAB1 = (digitalRead(encoderChA1) << 1) | digitalRead(encoderChB1);
  prevAB2 = (digitalRead(encoderChA2) << 1) | digitalRead(encoderChB2);
  prevAB3 = (digitalRead(encoderChA3) << 1) | digitalRead(encoderChB3);

  // [REVISI]: Attach interrupt CHANGE untuk semua channel
  attachInterrupt(digitalPinToInterrupt(encoderChA1), encoderA_SR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderChB1), encoderB_SR1, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encoderChA2), encoderA_SR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderChB2), encoderB_SR2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encoderChA3), encoderA_SR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderChB3), encoderB_SR3, CHANGE);

  prevMicros = micros();
}

// =======================================================
// LOOP UTAMA
// =======================================================
void loop() {
  // --- KOMUNIKASI SERIAL ---
  if (SerialUSB.available()) {
    char ch = SerialUSB.peek();
    if (ch == 'S' || ch == 's') {
      ch = SerialUSB.read();  // Konsumsi karakter
      runMotor = true;        // MOTOR NYALA (Standby)
      setpoint_deg1 = setpoint_deg2 = setpoint_deg3 = 0.0;
      setpoint_counts1 = setpoint_counts2 = setpoint_counts3 = 0.0;
    } else if (ch == 'X' || ch == 'x') {
      ch = SerialUSB.read();  // Konsumsi karakter
      runMotor = false;       // MOTOR MATI SECARA PAKSA
    } else if (ch == 'R' || ch == 'r') {
      ch = SerialUSB.read();  // Konsumsi karakter
      // R sengaja dikosongkan di Arduino karena Trajectory dikendalikan Python
    } else if (ch == 'I' || ch == 'i') {
      ch = SerialUSB.read();  // Konsumsi karakter
      idlePWM_zeroMode = !idlePWM_zeroMode;
      currentIdlePWM = idlePWM_zeroMode ? 0 : defaultIdlePWM;
    } else {
      String receivedData = SerialUSB.readStringUntil('\n');
      float tempMu1, tempMu2, tempMu3, tempSetpoint1, tempSetpoint2, tempSetpoint3;
      int parsed = sscanf(receivedData.c_str(), "%f,%f,%f,%f,%f,%f", &tempMu1, &tempMu2, &tempMu3, &tempSetpoint1, &tempSetpoint2, &tempSetpoint3);
      if (parsed == 6) {
        setpoint_rad1 = tempSetpoint1 * DEG_TO_RAD;
        setpoint_rad2 = tempSetpoint2 * DEG_TO_RAD;
        setpoint_rad3 = tempSetpoint3 * DEG_TO_RAD;
        mu1 = tempMu1;
        mu2 = tempMu2;
        mu3 = tempMu3;
        setpoint_counts1 = (setpoint_rad1 * countsPerRev) / (2.0 * PI);
        setpoint_counts2 = (setpoint_rad2 * countsPerRev) / (2.0 * PI);
        setpoint_counts3 = (setpoint_rad3 * countsPerRev) / (2.0 * PI);
      }
    }
  }

  // --- CEK SWITCH PENGAMAN ---
  // Jika salah satu, dua, atau tiga switch mendeteksi LOW (GND)
  if (digitalRead(safetySwitch1) == LOW || digitalRead(safetySwitch2) == LOW || digitalRead(safetySwitch3) == LOW) {
    runMotor = false;
  }

  if (!runMotor) {
    stopMotor1();
    stopMotor2();
    stopMotor3();
    return;  // Lewati eksekusi PID jika motor mati
  }
  // --- AKHIR KOMUNIKASI SERIAL & SAFETY ---

  unsigned long now = micros();
  float dt = (now - prevMicros) * 1e-6f;
  if (dt <= 0.0f || dt > 0.02f) dt = 0.001f;  // fallback: 1 ms
  prevMicros = now;

  // --- Baca encoder untuk tiap motor ---
  noInterrupts();
  countCopyBuffer1 = (float)en_pulse_count1;
  interrupts();
  actualAngleBuffer1 = (float)countCopyBuffer1 * (2.0 * PI) / countsPerRev;

  noInterrupts();
  countCopyBuffer2 = (float)en_pulse_count2;
  interrupts();
  actualAngleBuffer2 = (float)countCopyBuffer2 * (2.0 * PI) / countsPerRev;

  noInterrupts();
  countCopyBuffer3 = (float)en_pulse_count3;
  interrupts();
  actualAngleBuffer3 = (float)countCopyBuffer3 * (2.0 * PI) / countsPerRev;

  static float prevPosError1 = 0;
  static float prevPosError2 = 0;
  static float prevPosError3 = 0;
  static float prevActualAngle1 = 0;
  static float prevActualAngle2 = 0;
  static float prevActualAngle3 = 0;

  // Motor 1
  int raw1 = analogRead(currentSensorPin1);
  float voltage1 = raw1 * (3.3 / 4095.0);
  measuredCurrentBuffer1 = (1.51 - voltage1) / 0.1;

  // Motor 2
  int raw2 = analogRead(currentSensorPin2);
  float voltage2 = raw2 * (3.3 / 4095.0);
  measuredCurrentBuffer2 = (1.51 - voltage2) / 0.1;

  // Motor 3
  int raw3 = analogRead(currentSensorPin3);
  float voltage3 = raw3 * (3.3 / 4095.0);
  measuredCurrentBuffer3 = (1.51 - voltage3) / 0.1;

  float posError1 = setpoint_rad1 - actualAngleBuffer1;
  float posError2 = setpoint_rad2 - actualAngleBuffer2;
  float posError3 = setpoint_rad3 - actualAngleBuffer3;

  // Cara hitung derivative error dan kecepatan dengan Filter
  const float alphaErr = 0.15f;
  const float alphaVel = 0.15f;

  dEpos1 = filteredDerivative(posError1, prevPosError1, dt, dEpos1, alphaErr);
  dEpos2 = filteredDerivative(posError2, prevPosError2, dt, dEpos2, alphaErr);
  dEpos3 = filteredDerivative(posError3, prevPosError3, dt, dEpos3, alphaErr);

  dot_q1_f = filteredDerivative(actualAngleBuffer1, prevActualAngle1, dt, dot_q1_f, alphaVel);
  dot_q2_f = filteredDerivative(actualAngleBuffer2, prevActualAngle2, dt, dot_q2_f, alphaVel);
  dot_q3_f = filteredDerivative(actualAngleBuffer3, prevActualAngle3, dt, dot_q3_f, alphaVel);

  // --- Hitung Target Arus (berdasarkan error posisi) untuk tiap motor ---
  float sigma1 = c1 * posError1 + dEpos1;
  float q_ddot_d1 = 0.0;
  float tau1 = (J * q_ddot_d1) + (c1 * J * dEpos1) + ((J / mu1) * sigma1) + (b * dot_q1_f);
  float targetCurrent1 = tau1 / Kt;
  targetCurrentBuffer1 = targetCurrent1;

  float sigma2 = c2 * posError2 + dEpos2;
  float q_ddot_d2 = 0.0;
  float tau2 = (J * q_ddot_d2) + (c2 * J * dEpos2) + ((J / mu2) * sigma2) + (b * dot_q2_f);
  float targetCurrent2 = tau2 / Kt;
  targetCurrentBuffer2 = targetCurrent2;

  float sigma3 = c3 * posError3 + dEpos3;
  float q_ddot_d3 = 0.0;
  float tau3 = (J * q_ddot_d3) + (c3 * J * dEpos3) + ((J / mu3) * sigma3) + (b * dot_q3_f);
  float targetCurrent3 = tau3 / Kt;
  targetCurrentBuffer3 = targetCurrent3;

  prevPosError1 = posError1;
  prevPosError2 = posError2;
  prevPosError3 = posError3;
  prevActualAngle1 = actualAngleBuffer1;
  prevActualAngle2 = actualAngleBuffer2;
  prevActualAngle3 = actualAngleBuffer3;

  // --- PID UPDATE untuk kontrol arus tiap motor ---
  if (runMotor && now - lastPIDUpdate >= printIntervalMillis) {
    lastPIDUpdate = now;

    // Motor 1 PID
    double currentError1 = targetCurrentBuffer1 - measuredCurrentBuffer1;
    double curP_term1 = curKp1 * currentError1;
    curI_term1 += curKi1 * currentError1 * dt;
    curI_term1 = constrain(curI_term1, -1500, 1500);
    double curD_term1 = curKd1 * (currentError1 - prevCurrentError1) / dt;
    prevCurrentError1 = currentError1;
    pwmOutputBuffer1 = curP_term1 + curI_term1 + curD_term1;
    pwmOutputBuffer1 = constrain(pwmOutputBuffer1, -4095, 4095);

    // Motor 2 PID
    double currentError2 = targetCurrentBuffer2 - measuredCurrentBuffer2;
    double curP_term2 = curKp2 * currentError2;
    curI_term2 += curKi2 * currentError2 * dt;
    curI_term2 = constrain(curI_term2, -1500, 1500);
    double curD_term2 = curKd2 * (currentError2 - prevCurrentError2) / dt;
    prevCurrentError2 = currentError2;
    pwmOutputBuffer2 = curP_term2 + curI_term2 + curD_term2;
    pwmOutputBuffer2 = constrain(pwmOutputBuffer2, -4095, 4095);

    // Motor 3 PID
    double currentError3 = targetCurrentBuffer3 - measuredCurrentBuffer3;
    double curP_term3 = curKp3 * currentError3;
    curI_term3 += curKi3 * currentError3 * dt;
    curI_term3 = constrain(curI_term3, -1500, 1500);
    double curD_term3 = curKd3 * (currentError3 - prevCurrentError3) / dt;
    prevCurrentError3 = currentError3;
    pwmOutputBuffer3 = curP_term3 + curI_term3 + curD_term3;
    pwmOutputBuffer3 = constrain(pwmOutputBuffer3, -4095, 4095);
  }

  // --- SERIAL MONITORING (jika motor berjalan) ---
  unsigned long nowMillis = millis();
  if (runMotor && nowMillis - lastPrintTime >= printIntervalMillis) {
    lastPrintTime = nowMillis;
    float angle1_deg = actualAngleBuffer1 * (180.0f / PI);
    float angle2_deg = actualAngleBuffer2 * (180.0f / PI);
    float angle3_deg = actualAngleBuffer3 * (180.0f / PI);

    // Konversi hasil PID (PWM) menjadi Voltase untuk dikirim ke Python
    // Asumsi suplai motor adalah 12.0 Volt (Ubah jika kamu menggunakan 24V dll)
    float volt1 = (pwmOutputBuffer1 / 4095.0) * 12.0;
    float volt2 = (pwmOutputBuffer2 / 4095.0) * 12.0;
    float volt3 = (pwmOutputBuffer3 / 4095.0) * 12.0;

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
      calculatedPidCurrent1, calculatedPidCurrent1, calculatedPidCurrent1,
      measuredCurrentBuffer1, measuredCurrentBuffer2, measuredCurrentBuffer3);
  }

  // --- OUTPUT PWM untuk tiap motor ---
  if (runMotor) {
    int pwmVal1 = abs((int)pwmOutputBuffer1);
    if (abs(targetCurrentBuffer1) > 0.01 && pwmVal1 < minPWM) {
      pwmVal1 = minPWM;
    }
    digitalWrite(motorDIR1, -pwmOutputBuffer1 >= 0 ? HIGH : LOW);
    analogWrite(motorPWM1, pwmVal1);

    int pwmVal2 = abs((int)pwmOutputBuffer2);
    if (abs(targetCurrentBuffer2) > 0.01 && pwmVal2 < minPWM) {
      pwmVal2 = minPWM;
    }
    digitalWrite(motorDIR2, -pwmOutputBuffer2 >= 0 ? HIGH : LOW);
    analogWrite(motorPWM2, pwmVal2);

    int pwmVal3 = abs((int)pwmOutputBuffer3);
    if (abs(targetCurrentBuffer3) > 0.01 && pwmVal3 < minPWM) {
      pwmVal3 = minPWM;
    }
    digitalWrite(motorDIR3, -pwmOutputBuffer3 >= 0 ? HIGH : LOW);
    analogWrite(motorPWM3, pwmVal3);
  } else {
    // Jika motor tidak jalan, set semua motor ke idle
    analogWrite(motorPWM1, currentIdlePWM);
    digitalWrite(motorDIR1, LOW);
    pwmOutputBuffer1 = 0;
    curI_term1 = 0;

    analogWrite(motorPWM2, currentIdlePWM);
    digitalWrite(motorDIR2, LOW);
    pwmOutputBuffer2 = 0;
    curI_term2 = 0;

    analogWrite(motorPWM3, currentIdlePWM);
    digitalWrite(motorDIR3, LOW);
    pwmOutputBuffer3 = 0;
    curI_term3 = 0;
  }
}

// =======================================================
// ISR UNTUK ENCODER QUADRATURE (per motor)
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

void encoderA_SR1() {
  updateEncoder1();
}
void encoderB_SR1() {
  updateEncoder1();
}

void encoderA_SR2() {
  updateEncoder2();
}
void encoderB_SR2() {
  updateEncoder2();
}

void encoderA_SR3() {
  updateEncoder3();
}
void encoderB_SR3() {
  updateEncoder3();
}

void stopMotor1() {
  analogWrite(motorPWM1, 0);
  digitalWrite(motorDIR1, LOW);
}

void stopMotor2() {
  analogWrite(motorPWM2, 0);
  digitalWrite(motorDIR2, LOW);
}

void stopMotor3() {
  analogWrite(motorPWM3, 0);
  digitalWrite(motorDIR3, LOW);
}