#include <Arduino.h>
#include <Wire.h>

// ============================================
//            ENCODER CALIBRATION SCRIPT
// ============================================
// Purpose: Empirically determine your Xiaomi S10
// encoder's PPR and calibration constants.
//
// MODES (send via Serial):
//   '1' = Manual Rotation Mode (rotate wheel by hand, count ticks)
//   '2' = Drive Straight Mode  (drive 1m, measure actual ticks)
//   '3' = Spin-In-Place Mode   (360° rotation to calibrate track width)
//   'r' = Reset all counters
//   's' = Stop motors immediately
//   'p' = Print current counts
//
// After calibration, update ENCODER_PPR and
// DISTANCE_PER_TICK in debug_odometry.ino
// ============================================

// --- L298N Motor Driver ---
#define ENA 4
#define IN1 17
#define IN2 16
#define IN3 5
#define IN4 18
#define ENB 23

// --- Encoders ---
#define ENC_R_A 12
#define ENC_R_B 14
#define ENC_L_A 26
#define ENC_L_B 27

// --- MPU6050 ---
#define I2C_SDA 19
#define I2C_SCL 22

// ============================================
//          KNOWN PHYSICAL VALUES
// ============================================
// Measure these with calipers/ruler!
// Current values from your debug_odometry.ino
float WHEEL_DIAMETER_M = 0.067;  // meters — MEASURE AND VERIFY!
float TRACK_WIDTH_M    = 0.235;  // meters (center-to-center of wheels)

// ============================================
//            GLOBAL VARIABLES
// ============================================
volatile long countLeft  = 0;
volatile long countRight = 0;

// For edge detection analysis
volatile unsigned long lastEdgeTimeL = 0;
volatile unsigned long lastEdgeTimeR = 0;
volatile unsigned long minPeriodL = ULONG_MAX;
volatile unsigned long maxPeriodL = 0;
volatile unsigned long minPeriodR = ULONG_MAX;
volatile unsigned long maxPeriodR = 0;

// Snapshot values for delta calculation
long snapCountLeft  = 0;
long snapCountRight = 0;

// MPU6050
const int MPU_ADDR = 0x68;
float yawAngle = 0.0;
unsigned long lastGyroTime = 0;

// Mode tracking
int currentMode = 0;  // 0=idle, 1=manual, 2=drive, 3=spin
bool motorsRunning = false;
unsigned long modeStartTime = 0;
unsigned long lastPrintTime = 0;

// Drive/Spin parameters
const int CALIBRATION_PWM = 120;  // Moderate speed for consistent results
const float PI_VAL = 3.14159265359;

// ============================================
//        ENCODER INTERRUPT ROUTINES
// ============================================
// Using CHANGE on channel A — this gives 2x resolution
// (counts both rising and falling edges of channel A)
// If you also attach CHANGE on channel B, you get 4x.

void IRAM_ATTR isrLeft() {
  unsigned long now = micros();
  unsigned long period = now - lastEdgeTimeL;
  if (period > 100) {  // Debounce: ignore edges < 100µs apart
    if (period < minPeriodL) minPeriodL = period;
    if (period > maxPeriodL && period < 1000000) maxPeriodL = period;
    lastEdgeTimeL = now;
  }

  int a = digitalRead(ENC_L_A);
  int b = digitalRead(ENC_L_B);
  if (a == b)
    countLeft++;
  else
    countLeft--;
}

void IRAM_ATTR isrRight() {
  unsigned long now = micros();
  unsigned long period = now - lastEdgeTimeR;
  if (period > 100) {
    if (period < minPeriodR) minPeriodR = period;
    if (period > maxPeriodR && period < 1000000) maxPeriodR = period;
    lastEdgeTimeR = now;
  }

  int a = digitalRead(ENC_R_A);
  int b = digitalRead(ENC_R_B);
  if (a == b)
    countRight++;
  else
    countRight--;
}

// ============================================
//             MPU6050 HELPER
// ============================================
float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);  // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)2, true);
  if (Wire.available() == 2) {
    int16_t raw = Wire.read() << 8 | Wire.read();
    return raw / 131.0;  // degrees/sec (±250°/s range)
  }
  return 0.0;
}

void updateYaw() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0;
  lastGyroTime = now;
  if (dt > 0 && dt < 0.5) {
    yawAngle += readGyroZ() * dt;
  }
}

// ============================================
//           MOTOR CONTROL HELPERS
// ============================================
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  motorsRunning = false;
}

void driveForward(int pwm) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  motorsRunning = true;
}

void spinInPlace(int pwm) {
  // Left wheel backward, Right wheel forward → counterclockwise rotation
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  motorsRunning = true;
}

void resetCounters() {
  noInterrupts();
  countLeft = 0;
  countRight = 0;
  minPeriodL = ULONG_MAX;
  maxPeriodL = 0;
  minPeriodR = ULONG_MAX;
  maxPeriodR = 0;
  interrupts();

  snapCountLeft = 0;
  snapCountRight = 0;
  yawAngle = 0.0;

  Serial.println(">>> All counters RESET <<<");
}

// ============================================
//                 SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("============================================");
  Serial.println("    ENCODER CALIBRATION TOOL");
  Serial.println("    Xiaomi S10 Odometer Encoder");
  Serial.println("============================================");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  1 = Manual Rotation Mode (rotate wheel by hand)");
  Serial.println("  2 = Drive Straight 1m (measures ticks/meter)");
  Serial.println("  3 = Spin 360° (calibrates track width)");
  Serial.println("  r = Reset counters");
  Serial.println("  s = Stop motors");
  Serial.println("  p = Print current status");
  Serial.println();

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  stopMotors();

  // Encoder pins
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, CHANGE);

  // MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  lastGyroTime = millis();

  Serial.println("Ready. Send a command to begin.\n");
}

// ============================================
//          PRINT STATUS FUNCTION
// ============================================
void printStatus() {
  long cL = countLeft;
  long cR = countRight;

  Serial.println("--------------------------------------------");
  Serial.printf("  Left  Encoder: %7ld ticks\n", cL);
  Serial.printf("  Right Encoder: %7ld ticks\n", cR);
  Serial.printf("  Avg Ticks:     %7.1f\n", (cL + cR) / 2.0);
  Serial.println();

  // Calculate implied PPR if we know wheel circumference
  float wheelCirc = PI_VAL * WHEEL_DIAMETER_M;
  if (abs(cL) > 10) {
    float impliedPPR_L = abs(cL);  // if exactly 1 revolution was made
    Serial.printf("  If LEFT = 1 rev:  PPR = %ld\n", abs(cL));
    Serial.printf("    → dist/tick = %.6f m\n", wheelCirc / abs(cL));
    Serial.printf("    → ticks/m   = %.1f\n", abs(cL) / wheelCirc);
  }
  if (abs(cR) > 10) {
    Serial.printf("  If RIGHT = 1 rev: PPR = %ld\n", abs(cR));
    Serial.printf("    → dist/tick = %.6f m\n", wheelCirc / abs(cR));
    Serial.printf("    → ticks/m   = %.1f\n", abs(cR) / wheelCirc);
  }

  // Edge timing analysis
  if (minPeriodL < ULONG_MAX) {
    Serial.printf("  Left  edge period: min=%luµs  max=%luµs\n", minPeriodL, maxPeriodL);
  }
  if (minPeriodR < ULONG_MAX) {
    Serial.printf("  Right edge period: min=%luµs  max=%luµs\n", minPeriodR, maxPeriodR);
  }

  Serial.printf("  Gyro Yaw: %.1f°\n", yawAngle);
  Serial.println("--------------------------------------------");
}

// ============================================
//                  LOOP
// ============================================
void loop() {
  unsigned long now = millis();

  // Always integrate gyro
  updateYaw();

  // ---- Handle Serial Commands ----
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      case '1':
        stopMotors();
        resetCounters();
        currentMode = 1;
        Serial.println("\n╔══════════════════════════════════════════╗");
        Serial.println("║   MODE 1: MANUAL ROTATION                ║");
        Serial.println("║                                          ║");
        Serial.println("║   Rotate ONE wheel exactly 1 full turn.  ║");
        Serial.println("║   Mark the tire with tape to track.      ║");
        Serial.println("║   Press 'p' to see tick counts.          ║");
        Serial.println("║   The tick count = your ENCODER PPR      ║");
        Serial.println("║   (with CHANGE interrupt on 1 channel).  ║");
        Serial.println("╚══════════════════════════════════════════╝\n");
        break;

      case '2':
        stopMotors();
        resetCounters();
        currentMode = 2;
        Serial.println("\n╔══════════════════════════════════════════╗");
        Serial.println("║   MODE 2: DRIVE STRAIGHT                 ║");
        Serial.println("║                                          ║");
        Serial.println("║   Place robot at start of a 1m track.    ║");
        Serial.println("║   Robot will drive forward at PWM 120.   ║");
        Serial.println("║   Press 's' when it reaches the 1m mark. ║");
        Serial.println("║   Then press 'p' to see ticks/meter.     ║");
        Serial.println("╚══════════════════════════════════════════╝\n");
        Serial.println(">>> Starting drive... Press 's' to stop at 1m <<<");
        delay(500);
        driveForward(CALIBRATION_PWM);
        modeStartTime = now;
        break;

      case '3':
        stopMotors();
        resetCounters();
        currentMode = 3;
        yawAngle = 0.0;
        Serial.println("\n╔══════════════════════════════════════════╗");
        Serial.println("║   MODE 3: SPIN IN PLACE (360°)           ║");
        Serial.println("║                                          ║");
        Serial.println("║   Robot will spin counterclockwise.      ║");
        Serial.println("║   Auto-stops when gyro reads ~360°.      ║");
        Serial.println("║   Or press 's' to stop manually when     ║");
        Serial.println("║   the robot completes a full rotation.   ║");
        Serial.println("║   Use this to verify TRACK_WIDTH.        ║");
        Serial.println("╚══════════════════════════════════════════╝\n");
        Serial.println(">>> Starting spin... <<<");
        delay(500);
        spinInPlace(CALIBRATION_PWM);
        modeStartTime = now;
        break;

      case 'r':
        stopMotors();
        resetCounters();
        currentMode = 0;
        break;

      case 's':
        stopMotors();
        Serial.println(">>> MOTORS STOPPED <<<");
        printStatus();

        if (currentMode == 2) {
          long avgTicks = (abs(countLeft) + abs(countRight)) / 2;
          float wheelCirc = PI_VAL * WHEEL_DIAMETER_M;
          float computedPPR = avgTicks * wheelCirc;  // ticks for 1m → PPR = ticks * circumference / 1.0
          Serial.println("\n  === DRIVE CALIBRATION RESULTS ===");
          Serial.printf("  Avg ticks for 1m: %ld\n", avgTicks);
          Serial.printf("  Ticks/meter:      %ld\n", avgTicks);
          Serial.printf("  Distance/tick:    %.6f m\n", 1.0 / avgTicks);
          Serial.printf("  Implied PPR:      %.1f  (ticks per 1 wheel rev)\n",
                        avgTicks * wheelCirc);
          Serial.printf("    (based on wheel diameter = %.4fm)\n", WHEEL_DIAMETER_M);
          Serial.println("\n  ⚠ If robot didn't travel exactly 1m,");
          Serial.println("    scale the result proportionally!");
          Serial.println("  ====================================\n");
        }

        if (currentMode == 3) {
          long dL = abs(countLeft);
          long dR = abs(countRight);
          float avgTicks = (dL + dR) / 2.0;
          float wheelCirc = PI_VAL * WHEEL_DIAMETER_M;
          // For a 360° spin-in-place: each wheel travels π * TRACK_WIDTH
          // So: avgTicks * (wheelCirc / PPR) = π * TRACK_WIDTH
          // → TRACK_WIDTH = avgTicks * (wheelCirc / PPR) / π
          Serial.println("\n  === SPIN CALIBRATION RESULTS ===");
          Serial.printf("  Left ticks:  %ld\n", dL);
          Serial.printf("  Right ticks: %ld\n", dR);
          Serial.printf("  Gyro angle:  %.1f°\n", yawAngle);
          Serial.printf("  Avg ticks:   %.0f\n", avgTicks);
          Serial.println("\n  Use these values to verify TRACK_WIDTH:");
          Serial.println("    track_width = avg_ticks * DIST_PER_TICK / π");
          Serial.println("  ====================================\n");
        }
        break;

      case 'p':
        printStatus();
        break;
    }
  }

  // ---- Mode 3: Auto-stop at ~360° ----
  if (currentMode == 3 && motorsRunning) {
    if (abs(yawAngle) >= 355.0) {
      stopMotors();
      Serial.println("\n>>> AUTO-STOP: ~360° rotation detected! <<<");
      printStatus();

      long dL = abs(countLeft);
      long dR = abs(countRight);
      float avgTicks = (dL + dR) / 2.0;
      Serial.println("\n  === SPIN CALIBRATION RESULTS ===");
      Serial.printf("  Left ticks:  %ld\n", dL);
      Serial.printf("  Right ticks: %ld\n", dR);
      Serial.printf("  Gyro angle:  %.1f°\n", yawAngle);
      Serial.printf("  Avg ticks:   %.0f\n", avgTicks);
      Serial.println("  ====================================\n");
    }
  }

  // ---- Periodic telemetry while motors are running ----
  if (motorsRunning && (now - lastPrintTime >= 200)) {
    lastPrintTime = now;
    float elapsed = (now - modeStartTime) / 1000.0;
    Serial.printf("  [%.1fs] L:%6ld  R:%6ld  Yaw:%.1f°\n",
                  elapsed, countLeft, countRight, yawAngle);
  }
}
