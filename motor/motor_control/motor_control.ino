#include <Arduino.h>

// ================= MOTOR PINS =================
#define ENA_LEFT 4
#define IN1_LEFT 16
#define IN2_LEFT 17

#define ENB_RIGHT 19
#define IN3_RIGHT 18
#define IN4_RIGHT 5

// ================= ENCODER PINS =================
#define LEFT_ENC_A 13
#define LEFT_ENC_B 14
#define RIGHT_ENC_A 26
#define RIGHT_ENC_B 27

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// ================= ROBOT PARAMETERS =================
const float  WHEEL_DIAMETER_M  = 0.065;     // 6.5 cm
const int PULSES_PER_REV = 990;        // encoder counts per wheel revolution
const float WHEEL_CIRC_M = PI * WHEEL_DIAMETER_M ;  // meters per revolution
// Wheel Distance?


// ================= CONTROL PARAMETERS =================
const unsigned long SAMPLE_INTERVAL_MS = 100;  // control period

// const float KP = 120.0;
// const float KI = 10.0;
// const float KD = 5.0;

// Left motor PID
float KpL = 1100.0, 
      KiL = 0.0, 
      KdL = 0.0; 

// Right motor PID  
float KpR = 0.0, 
      KiR = 0.0, 
      KdR = 0.0;   

const int MAX_PWM = 255;
const int MIN_PWM = 50;

// ================= SPEED INPUT (edit here) =================
float desiredSpeedLeftMps = 0.35;   // Left wheel speed in m/s
float desiredSpeedRightMps = 0.25;  // Right wheel speed in m/s

// ================= INTERNAL VARIABLES =================
float integralLeft = 0, integralRight = 0;
float prevErrorLeft = 0, prevErrorRight = 0;
unsigned long lastControlTime = 0;

// ================= ENCODER INTERRUPT FUNCTIONS =================
void IRAM_ATTR updateLeftEncoder() {
  if (digitalRead(LEFT_ENC_B) != digitalRead(LEFT_ENC_A))
    leftEncoderCount--;
  else
    leftEncoderCount++;
}

void IRAM_ATTR updateRightEncoder() {
  if (digitalRead(RIGHT_ENC_B) != digitalRead(RIGHT_ENC_A))
    rightEncoderCount++;
  else
    rightEncoderCount--;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(ENA_LEFT, OUTPUT);
  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(ENB_RIGHT, OUTPUT);
  pinMode(IN3_RIGHT, OUTPUT);
  pinMode(IN4_RIGHT, OUTPUT);

  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN3_RIGHT, LOW);
  digitalWrite(IN4_RIGHT, LOW);
  analogWrite(ENA_LEFT, 0);
  analogWrite(ENB_RIGHT, 0);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), updateRightEncoder, CHANGE);

  Serial.println("Velocity Control Initialized");
  lastControlTime = millis();
}

// ================= LOOP =================
void loop() {
  if (millis() - lastControlTime >= SAMPLE_INTERVAL_MS) {
    lastControlTime = millis();

    long leftCount, rightCount;
    noInterrupts();
    leftCount = leftEncoderCount;
    rightCount = rightEncoderCount;
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    interrupts();

    // --- Compute measured speed ---
    float leftRevs = (float)leftCount / PULSES_PER_REV;
    float rightRevs = (float)rightCount / PULSES_PER_REV;

    float leftSpeedMps = (leftRevs * WHEEL_CIRC_M) / (SAMPLE_INTERVAL_MS / 1000.0);
    float rightSpeedMps = (rightRevs * WHEEL_CIRC_M) / (SAMPLE_INTERVAL_MS / 1000.0);

    // --- PID ---
    // float pwmLeft = computePID(desiredSpeedLeftMps, leftSpeedMps, integralLeft, prevErrorLeft);
    // float pwmRight = computePID(desiredSpeedRightMps, rightSpeedMps, integralRight, prevErrorRight);
    float pwmLeft = computePID_Left(desiredSpeedLeftMps, leftSpeedMps);
    float pwmRight = computePID_Right(desiredSpeedRightMps, rightSpeedMps);

    // --- Apply PWM to motors ---
    setMotorLeft(255);
    setMotorRight(255);


    // --- Telemetry ---
    Serial.print("L_des: "); Serial.print(desiredSpeedLeftMps, 3);
    Serial.print(" m/s | L_meas: "); Serial.print(leftSpeedMps, 3);
    // Serial.print(" | PWM_L: "); Serial.print(pwmLeft, 1);
    Serial.print(" || R_des: "); Serial.print(desiredSpeedRightMps, 3);
    Serial.print(" m/s | R_meas: "); Serial.print(rightSpeedMps, 3);
    // Serial.print(" | PWM_R: "); Serial.println(pwmRight, 1);
    
  }
}


// ================= HELPER FUNCTIONS =================
// float computePID(float desired, float measured, float &integral, float &prevError) {
//   float error = desired - measured;
//   integral += error * (SAMPLE_INTERVAL_MS / 1000.0);
//   float derivative = (error - prevError) / (SAMPLE_INTERVAL_MS / 1000.0);
//   prevError = error;
//   float output = KP * error + KI * integral + KD * derivative;
//   return output;
// }

float computePID_Left(float desired, float measured) {
  float error = desired - measured;
  integralLeft += error * (SAMPLE_INTERVAL_MS / 1000.0);
  float derivative = (error - prevErrorLeft) / (SAMPLE_INTERVAL_MS / 1000.0);
  prevErrorLeft = error;
  return KpL * error + KiL * integralLeft + KdL * derivative;
}

float computePID_Right(float desired, float measured) {
  float error = desired - measured;
  integralRight += error * (SAMPLE_INTERVAL_MS / 1000.0);
  float derivative = (error - prevErrorRight) / (SAMPLE_INTERVAL_MS / 1000.0);
  prevErrorRight = error;
  return KpR * error + KiR * integralRight + KdR * derivative;
}

void setMotorLeft(float pwmValue) { 
  pwmValue = (pwmValue > 0 ? MIN_PWM + pwmValue: -MIN_PWM + pwmValue );
  pwmValue = constrain(pwmValue, -MAX_PWM, MAX_PWM);

  // Apply minimum PWM threshold (deadband compensation)
  // if (abs(pwmValue) < MIN_PWM && pwmValue != 0) {
  //   pwmValue = (pwmValue > 0 ? MIN_PWM : -MIN_PWM);
  // }
  // 
  if (pwmValue > 0) {
    digitalWrite(IN1_LEFT, HIGH);
    digitalWrite(IN2_LEFT, LOW);
    analogWrite(ENA_LEFT, pwmValue);
  } 
  else if (pwmValue < 0) {
    digitalWrite(IN1_LEFT, LOW);
    digitalWrite(IN2_LEFT, HIGH);
    analogWrite(ENA_LEFT, -pwmValue);
  } 
  else {
    digitalWrite(IN1_LEFT, LOW);
    digitalWrite(IN2_LEFT, LOW);
    analogWrite(ENA_LEFT, 0);
  }
  
  Serial.print(" | PWM_L: "); Serial.println(pwmValue, 1);

}

void setMotorRight(float pwmValue) {
  pwmValue = constrain(pwmValue, -MAX_PWM, MAX_PWM);

  // Apply minimum PWM threshold (deadband compensation)
  if (abs(pwmValue) < MIN_PWM && pwmValue != 0) {
    pwmValue = (pwmValue > 0 ? MIN_PWM : -MIN_PWM);
  }

  if (pwmValue > 0) {
    digitalWrite(IN3_RIGHT, LOW);
    digitalWrite(IN4_RIGHT, HIGH);
    analogWrite(ENB_RIGHT, pwmValue);
  } 
  else if (pwmValue < 0) {
    digitalWrite(IN3_RIGHT, HIGH);
    digitalWrite(IN4_RIGHT, LOW);
    analogWrite(ENB_RIGHT, -pwmValue);
  } 
  else {
    digitalWrite(IN3_RIGHT, LOW);
    digitalWrite(IN4_RIGHT, LOW);
    analogWrite(ENB_RIGHT, 0);
  }

  Serial.print(" | PWM_R: "); Serial.println(pwmValue, 1);

}


void goForward(int speed) {
  // Set speed for both motors (0-255)
  analogWrite(ENA_LEFT, speed); // Full speed
  analogWrite(ENB_RIGHT, speed);
  
  Serial.println("Moving FORWARD...");
  
  // Left Motor Forward
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);
  // Right Motor Forward
  digitalWrite(IN3_RIGHT, LOW);
  digitalWrite(IN4_RIGHT, HIGH);
  
  // delay(2000); // Run for 2 seconds
}

void stop() {
  // --- ACTION: STOP ---
  Serial.println("Stopping...");
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN3_RIGHT, LOW);
  digitalWrite(IN4_RIGHT, LOW);
  analogWrite(ENA_LEFT, 0);
  analogWrite(ENB_RIGHT, 0);
  // delay(1000);
}

void encoderCount() {
  
  long leftCount;
  long rightCount;

  noInterrupts(); // Disable interrupts
  leftCount = leftEncoderCount;
  rightCount = rightEncoderCount;
  interrupts();   // Re-enable interrupts

  // --- Print the values ---
  Serial.print("Left Encoder: ");
  Serial.print(leftCount);
  Serial.print("  |  Right Encoder: ");
  Serial.println(rightCount);
  
  delay(100);
}
