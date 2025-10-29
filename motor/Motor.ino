#include <Arduino.h>

// Left Motor Pins
const int ENA_LEFT = 4 ; // Speed control (PWM)
const int IN1_LEFT = 16;
const int IN2_LEFT = 17;

// Right Motor Pins
const int ENB_RIGHT = 19; // Speed control (PWM)
const int IN3_RIGHT = 5;
const int IN4_RIGHT = 18;


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

  Serial.println("Program starting. Motors are stopped.");
}

void loop() {

  // --- ACTION 1: MOVE FORWARD ---
  Serial.println("Moving FORWARD...");
  // Left Motor Forward
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);
  // Right Motor Forward
  digitalWrite(IN3_RIGHT, LOW);
  digitalWrite(IN4_RIGHT, HIGH);
  // Set speed for both motors (0-255)
  analogWrite(ENA_LEFT, 255); // Full speed
  analogWrite(ENB_RIGHT, 255);
  delay(2000); // Run for 2 seconds

  // --- ACTION 2: STOP ---
  Serial.println("Stopping...");
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN3_RIGHT, LOW);
  digitalWrite(IN4_RIGHT, LOW);
  analogWrite(ENA_LEFT, 0);
  analogWrite(ENB_RIGHT, 0);
  delay(1000); // Wait for 1 second

  // --- ACTION 3: MOVE BACKWARD ---
  // Serial.println("Moving BACKWARD...");
  // // Left Motor Backward
  // digitalWrite(IN1_LEFT, LOW);
  // digitalWrite(IN2_LEFT, HIGH);
  // // Right Motor Backward
  // digitalWrite(IN3_RIGHT, LOW);
  // digitalWrite(IN4_RIGHT, HIGH);
  // // Set speed for both motors
  // analogWrite(ENA_LEFT, 128); // Half speed
  // analogWrite(ENB_RIGHT, 128);
  // delay(2000); // Run for 2 seconds

  // // --- ACTION 4: STOP ---
  // Serial.println("Stopping...");
  // digitalWrite(IN1_LEFT, LOW);
  // digitalWrite(IN2_LEFT, LOW);
  // digitalWrite(IN3_RIGHT, LOW);
  // digitalWrite(IN4_RIGHT, LOW);
  // analogWrite(ENA_LEFT, 0);
  // analogWrite(ENB_RIGHT, 0);
  // delay(1000); // Wait for 1 second

  // // --- ACTION 5: TURN LEFT ---
  // Serial.println("Turning LEFT...");
  // // Left Motor Backward
  // digitalWrite(IN1_LEFT, LOW);
  // digitalWrite(IN2_LEFT, HIGH);
  // // Right Motor Forward
  // digitalWrite(IN3_RIGHT, HIGH);
  // digitalWrite(IN4_RIGHT, LOW);
  // // Set speed for both motors
  // analogWrite(ENA_LEFT, 200);
  // analogWrite(ENB_RIGHT, 200);
  // delay(1500); // Turn for 1.5 seconds

  // // --- ACTION 6: STOP ---
  // Serial.println("Stopping...");
  // digitalWrite(IN1_LEFT, LOW);
  // digitalWrite(IN2_LEFT, LOW);
  // digitalWrite(IN3_RIGHT, LOW);
  // digitalWrite(IN4_RIGHT, LOW);
  // analogWrite(ENA_LEFT, 0);
  // analogWrite(ENB_RIGHT, 0);
  // delay(1000); // Wait for 1 second

  // // --- ACTION 7: TURN RIGHT ---
  // Serial.println("Turning RIGHT...");
  // // Left Motor Forward
  // digitalWrite(IN1_LEFT, HIGH);
  // digitalWrite(IN2_LEFT, LOW);
  // // Right Motor Backward
  // digitalWrite(IN3_RIGHT, LOW);
  // digitalWrite(IN4_RIGHT, HIGH);
  // // Set speed for both motors
  // analogWrite(ENA_LEFT, 200);
  // analogWrite(ENB_RIGHT, 200);
  // delay(1500); // Turn for 1.5 seconds

  // // --- ACTION 8: STOP ---
  // Serial.println("Stopping...");
  // digitalWrite(IN1_LEFT, LOW);
  // digitalWrite(IN2_LEFT, LOW);
  // digitalWrite(IN3_RIGHT, LOW);
  // digitalWrite(IN4_RIGHT, LOW);
  // analogWrite(ENA_LEFT, 0);
  // analogWrite(ENB_RIGHT, 0);
  // delay(3000); // Wait for 3 seconds before repeating the entire loop
}
