#include <Wire.h>

// ═══════════════════════════════════════════════════════════════════════════════
// Pin Definitions
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
#define IMU_SEND_INTERVAL   100   // ms (10 Hz for readability in serial monitor)
#define MOTOR_STATE_INTERVAL 2000 // 2 seconds between maneuvers

unsigned long lastImuSendTime = 0;
unsigned long lastStateChange = 0;
int currentState = -1; 
int testSpeed = 150; // PWM speed (0 to 255) for testing

// ═══════════════════════════════════════════════════════════════════════════════
// Motor Control
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

// ═══════════════════════════════════════════════════════════════════════════════
// MPU6050 Setup
// ═══════════════════════════════════════════════════════════════════════════════
void setupMPU6050() {
  Wire.begin();
  Wire.setClock(400000); 

  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Accel config (±2g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); 
  Wire.write(0x00); 
  Wire.endTransmission(true);

  // Gyro config (±250°/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); 
  Wire.write(0x00); 
  Wire.endTransmission(true);

  // DLPF to ~44 Hz
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A); 
  Wire.write(0x03); 
  Wire.endTransmission(true);
}

// ═══════════════════════════════════════════════════════════════════════════════
// IMU Read and Print
// ═══════════════════════════════════════════════════════════════════════════════
void readAndPrintIMU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire.available() < 14) return;

  // Read high and low bytes and combine them into 16-bit integers
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  Wire.read(); // Skip temp high
  Wire.read(); // Skip temp low

  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  // Print readable values to Serial Monitor
  Serial.print("Accel (X,Y,Z): ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.print(az);
  
  Serial.print("  |  Gyro (X,Y,Z): ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.println(gz);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("Starting Motor and IMU Debug...");

  // Motor pins
  pinMode(LEFT_FORWARD_PIN,   OUTPUT);
  pinMode(LEFT_BACKWARD_PIN,  OUTPUT);
  pinMode(RIGHT_FORWARD_PIN,  OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);
  
  setMotor(LEFT_FORWARD_PIN, LEFT_BACKWARD_PIN, 0);
  setMotor(RIGHT_FORWARD_PIN, RIGHT_BACKWARD_PIN, 0);

  setupMPU6050();

  lastImuSendTime = millis();
  // Force a state change immediately
  lastStateChange = millis() - MOTOR_STATE_INTERVAL;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Loop
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // 1. Process IMU reading independently
  if (now - lastImuSendTime >= IMU_SEND_INTERVAL) {
    readAndPrintIMU();
    lastImuSendTime = now;
  }

  // 2. State machine for motors checking every 2000ms
  if (now - lastStateChange >= MOTOR_STATE_INTERVAL) {
    currentState = (currentState + 1) % 4; // Cycle 0 -> 1 -> 2 -> 3
    lastStateChange = now;
    
    switch(currentState) {
      case 0: // Forward
        Serial.println("\n--- MOTOR STATE: FORWARD ---");
        setMotor(LEFT_FORWARD_PIN, LEFT_BACKWARD_PIN, testSpeed);
        setMotor(RIGHT_FORWARD_PIN, RIGHT_BACKWARD_PIN, testSpeed);
        break;
        
      case 1: // Backward
        Serial.println("\n--- MOTOR STATE: BACKWARD ---");
        setMotor(LEFT_FORWARD_PIN, LEFT_BACKWARD_PIN, -testSpeed);
        setMotor(RIGHT_FORWARD_PIN, RIGHT_BACKWARD_PIN, -testSpeed);
        break;
        
      case 2: // Left turn
        Serial.println("\n--- MOTOR STATE: LEFT TURN ---");
        setMotor(LEFT_FORWARD_PIN, LEFT_BACKWARD_PIN, -testSpeed);
        setMotor(RIGHT_FORWARD_PIN, RIGHT_BACKWARD_PIN, testSpeed);
        break;
        
      case 3: // Right turn
        Serial.println("\n--- MOTOR STATE: RIGHT TURN ---");
        setMotor(LEFT_FORWARD_PIN, LEFT_BACKWARD_PIN, testSpeed);
        setMotor(RIGHT_FORWARD_PIN, RIGHT_BACKWARD_PIN, -testSpeed);
        break;
    }
  }
}
