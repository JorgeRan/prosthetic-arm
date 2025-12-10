#include <Arduino.h>
#include <ESP32Servo.h>

Servo servo;
String incoming = "";

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  servo.attach(23);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    incoming = Serial.readStringUntil('\n');
    incoming.trim();

    // Serial.print("Got: ");
    // Serial.println(incoming);

    float x = incoming.toFloat();
    float angle = mapFloat(x, 0.0, 0.19, 0, 180);

    Serial.println(angle);

    angle = constrain(angle, 0, 180);

    servo.write(angle);
  }
}
