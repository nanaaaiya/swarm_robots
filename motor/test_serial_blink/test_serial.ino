// ESP32 LED blink on serial receive
const int LED_PIN = 2; // Onboard LED, usually GPIO 2

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n'); // Read line from Jetson
    digitalWrite(LED_PIN, HIGH); // Turn LED on
    delay(200);                  // Keep it on 200ms
    digitalWrite(LED_PIN, LOW);  // Turn LED off
    delay(50);
  }
}
