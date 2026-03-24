/*
 * motors.ino — Differential Drive Motor Controller + MPU6050 IMU
 *
 * Protocol (Pi → Arduino) — 8-byte motor packet:
 *   [0xAA][0x55][left_i8][right_i8][yaw_h][yaw_l][checksum][0x0D]
 *   checksum = (byte0 + byte1 + byte2 + byte3 + byte4 + byte5) & 0xFF
 *   yaw_h/yaw_l: reserved (currently unused, sent as 0x00)
 *
 * Protocol (Arduino → Pi) — 16-byte IMU packet:
 *   [0xBB][0x66][ax_h][ax_l][ay_h][ay_l][az_h][az_l]
 *   [gx_h][gx_l][gy_h][gy_l][gz_h][gz_l][checksum][0x0D]
 *   checksum = sum of bytes 0-13
 *
 * Motor features:
 *   - Dead-band compensation (remaps to PWM_MIN..255 so motors always move)
 *   - Kick-start pulse (overcomes stiction from standstill)
 *   - EMA smoothing on PWM output (prevents jerky jumps)
 *
 * Hardware:
 *   - BTS7960 dual H-bridge (4 PWM pins)
 *   - MPU6050 via I2C (SDA=A4, SCL=A5 on Uno/Nano)
 *   - UART (pins 0/1) connected to Raspberry Pi
 *
 * IMPORTANT: Disconnect Pi TX from Arduino pin 0 before uploading sketches.
 */

#include <Wire.h>

// ═══════════════════════════════════════════════════════════════════════════════
// Pin Definitions
// ═══════════════════════════════════════════════════════════════════════════════
#define LEFT_FORWARD_PIN   9
#define LEFT_BACKWARD_PIN  10
#define RIGHT_FORWARD_PIN  5
#define RIGHT_BACKWARD_PIN 6

// ═══════════════════════════════════════════════════════════════════════════════
// MPU6050
// ═══════════════════════════════════════════════════════════════════════════════
#define MPU6050_ADDR 0x68

// ═══════════════════════════════════════════════════════════════════════════════
// Motor tuning — calibrate per-motor on your robot
// Put the robot on the ground, slowly increase PWM_MIN until wheels just move.
// ═══════════════════════════════════════════════════════════════════════════════
#define PWM_MIN_LEFT     45   // minimum PWM where left motor turns under load
#define PWM_MIN_RIGHT    45   // minimum PWM where right motor turns under load
#define KICK_PWM_EXTRA   35   // extra PWM for kick-start pulse
#define KICK_DURATION_MS 20   // ms — kick-start duration
#define SMOOTH_ALPHA     0.4  // EMA alpha (0.1 = very smooth, 1.0 = instant)

// ═══════════════════════════════════════════════════════════════════════════════
// Timing
// ═══════════════════════════════════════════════════════════════════════════════
#define MOTOR_WATCHDOG_MS  500  // stop motors if no command for this long
#define IMU_SEND_INTERVAL  20   // ms (~50 Hz)

// ═══════════════════════════════════════════════════════════════════════════════
// State
// ═══════════════════════════════════════════════════════════════════════════════
unsigned long lastMotorCmdTime = 0;
unsigned long lastImuSendTime  = 0;
bool motorsActive = false;

// Serial parser
uint8_t rxBuf[8];
uint8_t rxIdx = 0;

// Smoothed PWM (EMA state)
float smoothLeftPWM  = 0.0;
float smoothRightPWM = 0.0;

// Kick-start detection
bool leftWasStopped  = true;
bool rightWasStopped = true;

// ═══════════════════════════════════════════════════════════════════════════════
// Dead-band compensation
// Maps raw 0-255 → PWM_MIN-255 so any non-zero command actually moves the motor
// ═══════════════════════════════════════════════════════════════════════════════
int compensateDeadband(int raw_pwm, int pwm_min) {
  if (raw_pwm == 0) return 0;
  int sign = (raw_pwm > 0) ? 1 : -1;
  int a = abs(raw_pwm);
  int output = pwm_min + (int)((float)a / 255.0 * (255 - pwm_min));
  return output * sign;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Motor drive — safe BTS7960 (never PWM both directions at once)
// ═══════════════════════════════════════════════════════════════════════════════
void setMotorRaw(int fwdPin, int bwdPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(fwdPin, speed);
    analogWrite(bwdPin, 0);
  } else if (speed < 0) {
    analogWrite(fwdPin, 0);
    analogWrite(bwdPin, -speed);
  } else {
    analogWrite(fwdPin, 0);
    analogWrite(bwdPin, 0);
  }
}

void kickStart(int fwdPin, int bwdPin, int target_pwm) {
  int kick = min(abs(target_pwm) + KICK_PWM_EXTRA, 255);
  int sign = (target_pwm > 0) ? 1 : -1;
  setMotorRaw(fwdPin, bwdPin, kick * sign);
  delay(KICK_DURATION_MS);
}

void stopMotors() {
  setMotorRaw(LEFT_FORWARD_PIN,  LEFT_BACKWARD_PIN,  0);
  setMotorRaw(RIGHT_FORWARD_PIN, RIGHT_BACKWARD_PIN, 0);
  motorsActive    = false;
  smoothLeftPWM   = 0.0;
  smoothRightPWM  = 0.0;
  leftWasStopped  = true;
  rightWasStopped = true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MPU6050 Setup
// ═══════════════════════════════════════════════════════════════════════════════
void setupMPU6050() {
  Wire.begin();
  Wire.setClock(400000);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); Wire.write(0x00); // wake
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); Wire.write(0x00); // accel ±2g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); Wire.write(0x00); // gyro ±250°/s
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A); Wire.write(0x03); // DLPF ~44Hz
  Wire.endTransmission(true);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Read IMU + send 16-byte packet to Pi
// ═══════════════════════════════════════════════════════════════════════════════
void readAndSendIMU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire.available() < 14) return;

  uint8_t packet[16];
  packet[0] = 0xBB;
  packet[1] = 0x66;

  // Accel X, Y, Z (6 bytes)
  for (int i = 2; i < 8; i++) packet[i] = Wire.read();

  // Skip temperature (2 bytes)
  Wire.read(); Wire.read();

  // Gyro X, Y, Z (6 bytes)
  for (int i = 8; i < 14; i++) packet[i] = Wire.read();

  // Checksum over bytes 0-13
  uint8_t cksum = 0;
  for (int i = 0; i < 14; i++) cksum += packet[i];
  packet[14] = cksum;
  packet[15] = 0x0D;

  Serial.write(packet, 16);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Process 8-byte motor packet from Pi
// [0xAA][0x55][left][right][yaw_h][yaw_l][checksum][0x0D]
// ═══════════════════════════════════════════════════════════════════════════════
void processMotorPacket(uint8_t* pkt) {
  // Validate header + tail
  if (pkt[0] != 0xAA || pkt[1] != 0x55 || pkt[7] != 0x0D) return;

  // Validate checksum (sum of bytes 0-5)
  uint8_t cksum = 0;
  for (int i = 0; i < 6; i++) cksum += pkt[i];
  if (cksum != pkt[6]) return;

  // Extract signed motor commands (-127 to +127)
  int8_t leftCmd  = (int8_t)pkt[2];
  int8_t rightCmd = (int8_t)pkt[3];
  // pkt[4], pkt[5] = yaw millirad/s (reserved, not used yet)

  // Scale [-127, +127] → [-255, +255]
  int leftRaw  = map(leftCmd,  -127, 127, -255, 255);
  int rightRaw = map(rightCmd, -127, 127, -255, 255);

  // Dead-band compensation
  int leftComp  = compensateDeadband(leftRaw,  PWM_MIN_LEFT);
  int rightComp = compensateDeadband(rightRaw, PWM_MIN_RIGHT);

  // EMA smoothing
  smoothLeftPWM  = smoothLeftPWM  * (1.0 - SMOOTH_ALPHA) + leftComp  * SMOOTH_ALPHA;
  smoothRightPWM = smoothRightPWM * (1.0 - SMOOTH_ALPHA) + rightComp * SMOOTH_ALPHA;

  int leftPWM  = (int)smoothLeftPWM;
  int rightPWM = (int)smoothRightPWM;

  // Final deadband (avoid motor whine at very low PWM)
  if (abs(leftPWM)  < PWM_MIN_LEFT  / 2) leftPWM  = 0;
  if (abs(rightPWM) < PWM_MIN_RIGHT / 2) rightPWM = 0;

  // Drive motors (no kick-start — the EMA ramp + deadband compensation handles it)
  setMotorRaw(LEFT_FORWARD_PIN,  LEFT_BACKWARD_PIN,  leftPWM);
  setMotorRaw(RIGHT_FORWARD_PIN, RIGHT_BACKWARD_PIN, rightPWM);

  leftWasStopped  = (leftPWM  == 0);
  rightWasStopped = (rightPWM == 0);

  motorsActive = true;
  lastMotorCmdTime = millis();
}

// ═══════════════════════════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  pinMode(LEFT_FORWARD_PIN,   OUTPUT);
  pinMode(LEFT_BACKWARD_PIN,  OUTPUT);
  pinMode(RIGHT_FORWARD_PIN,  OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);
  stopMotors();

  setupMPU6050();

  lastMotorCmdTime = millis();
  lastImuSendTime  = millis();
}

// ═══════════════════════════════════════════════════════════════════════════════
// Main Loop
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // 1. Parse incoming 8-byte motor packets
  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    if (rxIdx == 0 && b != 0xAA) continue;       // wait for header byte 1
    if (rxIdx == 1 && b != 0x55) { rxIdx = 0; continue; } // header byte 2

    rxBuf[rxIdx++] = b;

    if (rxIdx >= 8) {
      processMotorPacket(rxBuf);
      rxIdx = 0;
    }
  }

  // 2. Motor watchdog — stop if no command received
  if (motorsActive && (now - lastMotorCmdTime > MOTOR_WATCHDOG_MS)) {
    stopMotors();
  }

  // 3. Send IMU data at ~50 Hz
  if (now - lastImuSendTime >= IMU_SEND_INTERVAL) {
    readAndSendIMU();
    lastImuSendTime = now;
  }
}
