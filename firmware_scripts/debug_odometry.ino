#include <Arduino.h>
#include <Wire.h>

// ============================================
//                PIN DEFINITIONS
// ============================================

// --- L298N Motor Driver ---
#define ENA 4
#define IN1 17 // UPDATED
#define IN2 16 // UPDATED
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
//          ROBOT PHYSICAL KINEMATICS
// ============================================
const float WHEEL_DIAMETER = 0.070; // meters (70mm, confirmed)
const float TRACK_WIDTH = 0.235;    // meters (center-to-center wheel contact patches)

float ENCODER_PPR = 472.0; // Calibrated: manual wheel rotation measurement

const float PI_VAL = 3.14159265359;
const float WHEEL_CIRCUMFERENCE = PI_VAL * WHEEL_DIAMETER;
float DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / ENCODER_PPR;

// ============================================
//             CLOSED LOOP PID
// ============================================
// These values are starting points. You may need to tune them!
// Our PID operates on 'meters per second' errors.
// Example: If error is 0.1 m/s, Kp * 0.1 = 30 PWM.
float Kp = 300.0; // Proportional
float Ki = 800.0; // Integral
float Kd = 20.0;  // Derivative

float targetSpeedL = 0.0; // in m/s
float targetSpeedR = 0.0; // in m/s

float actualVelL = 0.0;
float actualVelR = 0.0;

float integralL = 0.0;
float integralR = 0.0;
float prevErrorL = 0.0;
float prevErrorR = 0.0;

float currentPwmL = 0;
float currentPwmR = 0;

// Maximum integral windup limit (to prevent the I-term from exploding if wheels
// are stuck)
float maxIntegral = 255.0 / Ki;

// ============================================
//            GLOBAL VARIABLES
// ============================================

// Encoders Counters
volatile long countLeft = 0;
volatile long countRight = 0;

// Variables to calculate deltas
long lastCountLeft = 0;
long lastCountRight = 0;

// Odometry Pose (Global coordinate frame)
float x_pos = 0.0;
float y_pos = 0.0;
float theta_pos = 0.0;

// Loop timing
unsigned long stateStartTime = 0;
unsigned long lastPIDTime = 0;
unsigned long lastPrintTime = 0;
int mainState = 0;

const int MPU_ADDR = 0x68;
int16_t GyZ;

// ============================================
//        ENCODER INTERRUPT ROUTINES
// ============================================
void IRAM_ATTR isrLeft() {
  int a = digitalRead(ENC_L_A);
  int b = digitalRead(ENC_L_B);
  if (a == b)
    countLeft--;   // Inverted: encoder was reading opposite sign
  else
    countLeft++;
}

void IRAM_ATTR isrRight() {
  int a = digitalRead(ENC_R_A);
  int b = digitalRead(ENC_R_B);
  if (a == b)
    countRight++;
  else
    countRight--;
}

// ============================================
//                 SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting Closed Loop PID + Odometry Script...");

  // Select motors
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Stop initially
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // Encoders
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

  stateStartTime = millis();
  lastPIDTime = millis();
}

// ============================================
//         MOTOR APPLICATION FUNCTION
// ============================================
// Converts signed PID output into physical Direction + PWM
void applyMotorSpeeds(float outL, float outR) {
  // Left Motor
  if (outL >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    outL = -outL; // make positive for PWM
  }

  // Right Motor
  if (outR >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    outR = -outR;
  }

  // Constrain to physical limits and write
  currentPwmL = constrain((int)outL, 0, 255);
  currentPwmR = constrain((int)outR, 0, 255);

  analogWrite(ENA, currentPwmL);
  analogWrite(ENB, currentPwmR);
}

// ============================================
//                  LOOP
// ============================================
void loop() {
  unsigned long currentTime = millis();

  // ------------------------------------------
  // 1. STATE MACHINE (Target Velocity Generator)
  // ------------------------------------------
  unsigned long elapsed = currentTime - stateStartTime;

  if (elapsed >= 5000) {
    stateStartTime = currentTime;
    mainState = (mainState + 1) % 4; // Forward -> Left -> Right -> Backward
  }

  // Target Speed Definition (e.g. 0.15 m/s)
  float baseSpeed = 0.15;
  const char *directionStr = "";

  switch (mainState) {
  case 0: // FORWARD
    targetSpeedL = baseSpeed;
    targetSpeedR = baseSpeed;
    directionStr = "FORWARD";
    break;
  case 1: // TURN LEFT (Left wheel back, Right wheel fwd)
    targetSpeedL = -baseSpeed;
    targetSpeedR = baseSpeed;
    directionStr = "LEFT";
    break;
  case 2: // TURN RIGHT
    targetSpeedL = baseSpeed;
    targetSpeedR = -baseSpeed;
    directionStr = "RIGHT";
    break;
  case 3: // REVERSE
    targetSpeedL = -baseSpeed;
    targetSpeedR = -baseSpeed;
    directionStr = "REVERSE";
    break;
  }

  // Fade out behavior during the last 0.5 sec of a state to mimic smooth
  // stopping
  if (elapsed > 4500) {
    targetSpeedL = 0;
    targetSpeedR = 0;
  }

  // ------------------------------------------
  // 2. PID & ODOMETRY LOOP (20Hz = Every 50ms)
  // ------------------------------------------
  if (currentTime - lastPIDTime >= 50) {
    float dt = (currentTime - lastPIDTime) / 1000.0;
    lastPIDTime = currentTime;

    // --- A. Read Encoders ---
    long currentLeft = countLeft;
    long currentRight = countRight;
    long dLeftTicks = currentLeft - lastCountLeft;
    long dRightTicks = currentRight - lastCountRight;

    lastCountLeft = currentLeft;
    lastCountRight = currentRight;

    // --- B. Odometry & Velocity Calculation ---
    float distLeft = dLeftTicks * DISTANCE_PER_TICK;
    float distRight = dRightTicks * DISTANCE_PER_TICK;

    actualVelL = distLeft / dt;
    actualVelR = distRight / dt;

    float dCenter = (distLeft + distRight) / 2.0;
    float dTheta = (distRight - distLeft) / TRACK_WIDTH;

    x_pos += dCenter * cos(theta_pos + (dTheta / 2.0));
    y_pos += dCenter * sin(theta_pos + (dTheta / 2.0));
    theta_pos += dTheta;

    while (theta_pos > PI_VAL)
      theta_pos -= 2.0 * PI_VAL;
    while (theta_pos < -PI_VAL)
      theta_pos += 2.0 * PI_VAL;

    // --- C. PID Core Calculation ---
    float errorL = targetSpeedL - actualVelL;
    float errorR = targetSpeedR - actualVelR;

    // Integrate with Anti-Windup
    integralL += errorL * dt;
    integralR += errorR * dt;
    integralL = constrain(integralL, -maxIntegral, maxIntegral);
    integralR = constrain(integralR, -maxIntegral, maxIntegral);

    // Derivative
    float derivL = (errorL - prevErrorL) / dt;
    float derivR = (errorR - prevErrorR) / dt;
    prevErrorL = errorL;
    prevErrorR = errorR;

    // Final Output (translates target speed + error directly into a PWM offset)
    float outL = (Kp * errorL) + (Ki * integralL) + (Kd * derivL);
    float outR = (Kp * errorR) + (Ki * integralR) + (Kd * derivR);

    // If target is perfectly 0, force stop to prevent micro-jitter
    if (targetSpeedL == 0 && targetSpeedR == 0) {
      outL = 0;
      outR = 0;
      integralL = 0;
      integralR = 0;
    }

    // --- D. Apply to Motors ---
    applyMotorSpeeds(outL, outR);
  }

  // ------------------------------------------
  // 3. SERIAL TELEMETRY (10Hz)
  // ------------------------------------------
  if (currentTime - lastPrintTime >= 100) {
    lastPrintTime = currentTime;

    // Read MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); // GyZ
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)2, true);
    if (Wire.available() == 2)
      GyZ = Wire.read() << 8 | Wire.read();

    float gyroYawSpeed = GyZ / 131.0;
    float thetaDeg = theta_pos * (180.0 / PI_VAL);

    // Dynamic clean printing
    Serial.printf("[%s] tgt: %.2fm/s | actL: %+.2fm/s actR: %+.2fm/s | PWM_L: "
                  "%3.0f PWM_R: %3.0f | X: %.2f Y: %.2f H: %6.1f° | G: %.1f\n",
                  directionStr, targetSpeedL, actualVelL, actualVelR,
                  currentPwmL, currentPwmR, x_pos, y_pos, thetaDeg,
                  gyroYawSpeed);
  }
}
