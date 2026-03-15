/*
 * motors.ino — Differential Drive Motor Controller + MPU6050 IMU
 *
 * Receives 6-byte motor command packets from Raspberry Pi via UART:
 *   [0xAA] [0x55] [left_i8] [right_i8] [checksum] [0x0D]
 *
 * Sends 16-byte IMU data packets upstream at ~50 Hz:
 *   [0xBB] [0x66] [ax_h] [ax_l] [ay_h] [ay_l] [az_h] [az_l]
 *   [gx_h] [gx_l] [gy_h] [gy_l] [gz_h] [gz_l] [checksum] [0x0D]
 *
 * [FUTURE] Encoder packet (0xCC 0x77) — reserved, see bottom of file.
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
// Pin Definitions — swap FORWARD/BACKWARD to reverse a motor's direction
// ═══════════════════════════════════════════════════════════════════════════════
#define LEFT_FORWARD_PIN   9
#define LEFT_BACKWARD_PIN  10
#define RIGHT_FORWARD_PIN  5
#define RIGHT_BACKWARD_PIN 6

// ═══════════════════════════════════════════════════════════════════════════════
// MPU6050 I2C Address
// ═══════════════════════════════════════════════════════════════════════════════
#define MPU6050_ADDR 0x68

// ═══════════════════════════════════════════════════════════════════════════════
// Timing
// ═══════════════════════════════════════════════════════════════════════════════
#define MOTOR_WATCHDOG_MS   500   // Stop motors if no command for this long
#define IMU_SEND_INTERVAL   20    // ms (~50 Hz)

// ═══════════════════════════════════════════════════════════════════════════════
// State
// ═══════════════════════════════════════════════════════════════════════════════
unsigned long lastMotorCmdTime = 0;
unsigned long lastImuSendTime  = 0;
bool motorsActive = false;

// Serial receive buffer
uint8_t rxBuf[6];
uint8_t rxIdx = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// Motor Control — safe BTS7960 drive (never PWM both directions at once)
// ═══════════════════════════════════════════════════════════════════════════════
void setMotor(int fwdPin, int bwdPin, int speed) {
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

void stopMotors() {
  setMotor(LEFT_FORWARD_PIN,  LEFT_BACKWARD_PIN,  0);
  setMotor(RIGHT_FORWARD_PIN, RIGHT_BACKWARD_PIN, 0);
  motorsActive = false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MPU6050 Setup
// ═══════════════════════════════════════════════════════════════════════════════
void setupMPU6050() {
  Wire.begin();
  Wire.setClock(400000); // 400 kHz fast I2C

  // Wake up MPU6050 (exit sleep mode)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Clear sleep bit
  Wire.endTransmission(true);

  // Configure accelerometer: ±2g (default, sensitivity = 16384 LSB/g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00); // ±2g
  Wire.endTransmission(true);

  // Configure gyroscope: ±250°/s (default, sensitivity = 131 LSB/°/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00); // ±250°/s
  Wire.endTransmission(true);

  // Set DLPF (Digital Low Pass Filter) to ~44 Hz bandwidth
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A); // CONFIG
  Wire.write(0x03); // DLPF_CFG = 3 (44 Hz accel, 42 Hz gyro)
  Wire.endTransmission(true);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Read 14 bytes of raw sensor data from MPU6050 and send as packet
// ═══════════════════════════════════════════════════════════════════════════════
void readAndSendIMU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Start at ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire.available() < 14) return; // Incomplete read, skip

  uint8_t packet[16];
  packet[0] = 0xBB; // Header
  packet[1] = 0x66; // Header

  // Accel X, Y, Z (6 bytes) — registers 0x3B–0x40
  packet[2] = Wire.read();  // ax_h
  packet[3] = Wire.read();  // ax_l
  packet[4] = Wire.read();  // ay_h
  packet[5] = Wire.read();  // ay_l
  packet[6] = Wire.read();  // az_h
  packet[7] = Wire.read();  // az_l

  // Skip temperature (2 bytes) — registers 0x41–0x42
  Wire.read();
  Wire.read();

  // Gyro X, Y, Z (6 bytes) — registers 0x43–0x48
  packet[8]  = Wire.read(); // gx_h
  packet[9]  = Wire.read(); // gx_l
  packet[10] = Wire.read(); // gy_h
  packet[11] = Wire.read(); // gy_l
  packet[12] = Wire.read(); // gz_h
  packet[13] = Wire.read(); // gz_l

  // Checksum: sum of bytes 0–13
  uint8_t cksum = 0;
  for (int i = 0; i < 14; i++) {
    cksum += packet[i];
  }
  packet[14] = cksum;
  packet[15] = 0x0D; // Tail

  Serial.write(packet, 16);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Parse incoming motor command packet
// ═══════════════════════════════════════════════════════════════════════════════
void processMotorPacket(uint8_t* pkt) {
  // Validate header and tail
  if (pkt[0] != 0xAA || pkt[1] != 0x55 || pkt[5] != 0x0D) return;

  // Validate checksum
  uint8_t expected_cksum = (pkt[0] + pkt[1] + pkt[2] + pkt[3]) & 0xFF;
  if (pkt[4] != expected_cksum) return;

  // Extract signed speeds (-127 to +127)
  int8_t leftCmd  = (int8_t)pkt[2];
  int8_t rightCmd = (int8_t)pkt[3];

  // Scale from [-127, +127] to [-255, +255] PWM
  int leftPWM  = map(leftCmd,  -127, 127, -255, 255);
  int rightPWM = map(rightCmd, -127, 127, -255, 255);

  // Apply deadband (ignore tiny commands that cause motor whine)
  if (abs(leftPWM)  < 5) leftPWM  = 0;
  if (abs(rightPWM) < 5) rightPWM = 0;

  setMotor(LEFT_FORWARD_PIN,  LEFT_BACKWARD_PIN,  leftPWM);
  setMotor(RIGHT_FORWARD_PIN, RIGHT_BACKWARD_PIN, rightPWM);

  motorsActive = true;
  lastMotorCmdTime = millis();
}

// ═══════════════════════════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(LEFT_FORWARD_PIN,   OUTPUT);
  pinMode(LEFT_BACKWARD_PIN,  OUTPUT);
  pinMode(RIGHT_FORWARD_PIN,  OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);
  stopMotors();

  // MPU6050
  setupMPU6050();

  lastMotorCmdTime = millis();
  lastImuSendTime  = millis();
}

// ═══════════════════════════════════════════════════════════════════════════════
// Main Loop
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── 1. Read serial for motor commands ──────────────────────────────────────
  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    // Synchronize: if we see header byte 0xAA at position 0, start buffering
    if (rxIdx == 0 && b != 0xAA) continue; // Wait for header
    if (rxIdx == 1 && b != 0x55) { rxIdx = 0; continue; } // Bad second header

    rxBuf[rxIdx++] = b;

    if (rxIdx >= 6) {
      processMotorPacket(rxBuf);
      rxIdx = 0;
    }
  }

  // ── 2. Motor watchdog ──────────────────────────────────────────────────────
  if (motorsActive && (now - lastMotorCmdTime > MOTOR_WATCHDOG_MS)) {
    stopMotors();
  }

  // ── 3. Send IMU data at ~50 Hz ────────────────────────────────────────────
  if (now - lastImuSendTime >= IMU_SEND_INTERVAL) {
    readAndSendIMU();
    lastImuSendTime = now;
  }

  // ── 4. [FUTURE] Read encoders and send encoder packet ─────────────────────
  // When you add quadrature encoders:
  //   - Attach interrupt pins for encoder A/B channels
  //   - Count ticks in ISR
  //   - At ~50 Hz, send packet: [0xCC] [0x77] [leftTicks_4bytes] [rightTicks_4bytes] [cksum] [0x0D]
  //   - Reset tick counters after sending
  // The diff_drive_controller on the Pi already has a stub to parse 0xCC 0x77 packets.
}