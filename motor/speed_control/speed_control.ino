#include <Arduino.h>

// Left Motor Pins
const int ENA_LEFT = 4; // PWM
const int IN1_LEFT = 16;
const int IN2_LEFT = 17;

// Right Motor Pins
const int ENB_RIGHT = 19; // PWM
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

  // Stop motors initially
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN3_RIGHT, LOW);
  digitalWrite(IN4_RIGHT, LOW);
  analogWrite(ENA_LEFT, 0);
  analogWrite(ENB_RIGHT, 0);

  Serial.println("ESP32 ready for PWM input");
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n'); // Expect "pwm\n" or "pwmL,pwmR\n"
    data.trim();
    
    int commaIndex = data.indexOf(',');
    int pwmL, pwmR;

    if (commaIndex == -1) {
      // Only one value sent → use same for both motors
      pwmL = pwmR = data.toInt();
    } else {
      pwmL = data.substring(0, commaIndex).toInt();
      pwmR = data.substring(commaIndex + 1).toInt();
    }

    // Constrain PWM to 0-255
    pwmL = constrain(pwmL, 0, 255);
    pwmR = constrain(pwmR, 0, 255);

    // Set motors forward
    digitalWrite(IN1_LEFT, HIGH);
    digitalWrite(IN2_LEFT, LOW);
    digitalWrite(IN3_RIGHT, LOW);
    digitalWrite(IN4_RIGHT, HIGH);
    analogWrite(ENA_LEFT, pwmL);
    analogWrite(ENB_RIGHT, pwmR);

    // Serial.print("PWM set -> Left: ");
    // Serial.print(pwmL);
    // Serial.print(" Right: ");
    // Serial.println(pwmR);
  }
}
