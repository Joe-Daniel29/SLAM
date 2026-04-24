#include <Arduino.h>
#include <Wire.h>

// ============================================
//                PIN DEFINITIONS
// ============================================

// --- L298N Motor Driver ---
#define ENA 4
#define IN1 17
#define IN2 16
#define IN3 5
#define IN4 18
#define ENB 23

// --- Encoders ---
// Right Encoder 
#define ENC_R_A 12
#define ENC_R_B 14
// Left Encoder
#define ENC_L_A 26
#define ENC_L_B 27

// --- MPU6050 ---
#define I2C_SDA 19
#define I2C_SCL 22

// ============================================
//            GLOBAL VARIABLES
// ============================================

// Encoders Counters
volatile long countLeft = 0;
volatile long countRight = 0;

volatile bool dirLeftClockwise = true;
volatile bool dirRightClockwise = true;

// MPU6050 (Basic implementation without external heavy libraries)
const int MPU_ADDR = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// Non-blocking loop timing variables
unsigned long stateStartTime = 0;
unsigned long lastSerialPrint = 0;
int mainState = 0; // 0=Forward, 1=Left, 2=Right, 3=Backward

// ============================================
//        ENCODER INTERRUPT ROUTINES
// ============================================
void IRAM_ATTR isrLeft() {
  int a = digitalRead(ENC_L_A);
  int b = digitalRead(ENC_L_B);
  // Simple quadrature decoding logic
  if (a == b) {
    countLeft++;
    dirLeftClockwise = true;
  } else {
    countLeft--;
    dirLeftClockwise = false;
  }
}

void IRAM_ATTR isrRight() {
  int a = digitalRead(ENC_R_A);
  int b = digitalRead(ENC_R_B);
  if (a == b) {
    countRight++;
    dirRightClockwise = true;
  } else {
    countRight--;
    dirRightClockwise = false;
  }
}

// ============================================
//                 SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to connect

  Serial.println("Starting Hardware Debug Script...");

  // -- Setup Motor Pins --
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Stop motors initially
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // -- Setup Encoder Pins & Interrupts --
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  // We attach interrupt to Phase A of both encoders
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, CHANGE);

  // -- Initialize MPU6050 --
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero to wake up the MPU-6050
  Wire.endTransmission(true);

  Serial.println("Setup complete. Starting main loop...");
  stateStartTime = millis();
}

// ============================================
//             HELPER FUNCTIONS
// ============================================
void setMotorDirections(bool leftFwd, bool rightFwd) {
  digitalWrite(IN1, leftFwd ? HIGH : LOW);
  digitalWrite(IN2, leftFwd ? LOW : HIGH);
  
  digitalWrite(IN3, rightFwd ? HIGH : LOW);
  digitalWrite(IN4, rightFwd ? LOW : HIGH);
}

// ============================================
//                  LOOP
// ============================================
void loop() {
  unsigned long currentTime = millis();
  
  // ------------------------------------------
  // 1. NON-BLOCKING MOTOR STATE MACHINE
  // ------------------------------------------
  // The robot moves every 5000ms.
  // We use this elapsed time to determine fade-in / hold / fade-out
  unsigned long elapsed = currentTime - stateStartTime;
  
  if (elapsed >= 5000) {
    stateStartTime = currentTime;
    elapsed = 0;
    mainState = (mainState + 1) % 4; // Loop through Forward -> Left -> Right -> Backward
  }

  // Calculate PWM fade logic
  int currentSpeed = 0;
  int maxSpeed = 200; // Target maximum PWM (0-255 range by default)

  // Smooth Speed profile over the 5 seconds interval
  if (elapsed < 1000) {
    // Fade IN for the first 1 second (0 -> maxSpeed)
    currentSpeed = map(elapsed, 0, 1000, 0, maxSpeed);
  } else if (elapsed < 4000) {
    // Hold constant speed for 3 seconds
    currentSpeed = maxSpeed;
  } else if (elapsed < 5000) {
    // Fade OUT for the last 1 second (maxSpeed -> 0)
    currentSpeed = map(elapsed, 4000, 5000, maxSpeed, 0);
  }

  // Ensure values don't go out of bounds mathematically
  currentSpeed = constrain(currentSpeed, 0, 255);

  const char* directionStr = "";

  // Apply motor direction based on state
  switch (mainState) {
    case 0: 
      setMotorDirections(true, true); 
      directionStr = "FORWARD";
      break;
    case 1: 
      setMotorDirections(false, true); 
      directionStr = "LEFT";
      break;
    case 2: 
      setMotorDirections(true, false); 
      directionStr = "RIGHT";
      break;
    case 3: 
      setMotorDirections(false, false); 
      directionStr = "REVERSE";
      break;
  }

  // Apply PWM to ENA and ENB (gradual acceleration / deceleration)
  analogWrite(ENA, currentSpeed);
  analogWrite(ENB, currentSpeed);

  // ------------------------------------------
  // 2. READ MPU-6050 & PRINT SERIAL DATA
  // ------------------------------------------
  // Print 4 times a second (every 250ms), without blocking the loop!
  if (currentTime - lastSerialPrint >= 250) {
    lastSerialPrint = currentTime;

    // Read MPU Data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true);  
    
    if (Wire.available() == 14) {
      AcX = Wire.read() << 8 | Wire.read();  
      AcY = Wire.read() << 8 | Wire.read();  
      AcZ = Wire.read() << 8 | Wire.read();  
      Tmp = Wire.read() << 8 | Wire.read();  
      GyX = Wire.read() << 8 | Wire.read();  
      GyY = Wire.read() << 8 | Wire.read();  
      GyZ = Wire.read() << 8 | Wire.read();  
    }

    // Print telemetry data intelligently in a clean one-liner
    Serial.printf("[%7s | PWM:%3d] ENC L:%-13s(%6ld) R:%-13s(%6ld) | ACC: %6d,%6d,%6d | GYR: %6d,%6d,%6d\n",
                  directionStr, currentSpeed,
                  dirLeftClockwise ? "Clockwise" : "Anticlockwise", countLeft,
                  dirRightClockwise ? "Clockwise" : "Anticlockwise", countRight,
                  AcX, AcY, AcZ, GyX, GyY, GyZ);
  }
}
