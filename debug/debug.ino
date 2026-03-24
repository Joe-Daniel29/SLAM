#define LEFT_FORWARD_PIN   9
#define LEFT_BACKWARD_PIN  10
#define RIGHT_FORWARD_PIN  5
#define RIGHT_BACKWARD_PIN 6

// ═══════════════════════════════════════════════════════════════════════════════
// TUNE THESE THREE VALUES
// ═══════════════════════════════════════════════════════════════════════════════
const int MAX_SPEED     = 65;  // Lowered from 45. (Warning: if < 25, motors may stall)
const int RAMP_STEP     = 1;   // Smallest possible increment for maximum smoothness
const int RAMP_DELAY_MS = 30;  // Increased delay to make the acceleration "lazy"
// ═══════════════════════════════════════════════════════════════════════════════

int currentLeft = 0;
int currentRight = 0;

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

void rampedDrive(int targetLeft, int targetRight, unsigned long holdTime) {
  bool leftDone = false;
  bool rightDone = false;

  while (!leftDone || !rightDone) {
    // Ramp Left
    if (abs(targetLeft - currentLeft) <= RAMP_STEP) {
      currentLeft = targetLeft;
      leftDone = true;
    } else {
      currentLeft += (targetLeft > currentLeft) ? RAMP_STEP : -RAMP_STEP;
    }

    // Ramp Right
    if (abs(targetRight - currentRight) <= RAMP_STEP) {
      currentRight = targetRight;
      rightDone = true;
    } else {
      currentRight += (targetRight > currentRight) ? RAMP_STEP : -RAMP_STEP;
    }

    setMotor(LEFT_FORWARD_PIN, LEFT_BACKWARD_PIN, currentLeft);
    setMotor(RIGHT_FORWARD_PIN, RIGHT_BACKWARD_PIN, currentRight);
    delay(RAMP_DELAY_MS);
  }
  delay(holdTime);
}

void setup() {
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);
}

void loop() {
  // Slow Forward
  rampedDrive(MAX_SPEED, MAX_SPEED, 2000);
  
  // Slow Stop
  rampedDrive(0, 0, 1000);
  
  // Slow Backward
  rampedDrive(-MAX_SPEED, -MAX_SPEED, 2000);
  
  // Slow Stop
  rampedDrive(0, 0, 5000);

   rampedDrive(MAX_SPEED, -MAX_SPEED, 1000);
  
  // Slow Stop
  rampedDrive(0, 0, 5000);

  
   rampedDrive(-MAX_SPEED, MAX_SPEED, 1000);
  
  // Slow Stop
  rampedDrive(0, 0, 5000);
}