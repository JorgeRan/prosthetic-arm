#include <Arduino.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>


const size_t CAPACITY = 512;
StaticJsonDocument<CAPACITY> doc;

Servo thumb;
Servo indexf;
Servo middle;
Servo ring;
Servo pinky;

String incoming = "";

int sensorPin = 26;

int latchPin = 12;
int clockPin = 13;
int dataPin = 14;

// int threshold[8] = {511, 1023, 1534, 2045, 2556, 3067, 3578, 4089};
float threshold[8] = { 0.02375, 0.0475, 0.07125, 0.095, 0.11875, 0.1425, 0.16625, 0.19 };


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float smoothThumb  = 0;
float smoothIndex  = 0;
float smoothMiddle = 0;
float smoothRing   = 0;
float smoothPinky  = 0;

const float alpha = 0.15;  


void setup() {
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  thumb.setPeriodHertz(50);
  indexf.setPeriodHertz(50);
  middle.setPeriodHertz(50);
  ring.setPeriodHertz(50);
  pinky.setPeriodHertz(50);

  thumb.attach(22);
  indexf.attach(25);
  middle.attach(19);
  ring.attach(18);
  pinky.attach(23);

  thumb.write(0);
  indexf.write(0);
  middle.write(0);
  ring.write(0);
  pinky.write(0);

  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {

 
    DeserializationError error = deserializeJson(doc, Serial);

    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    serializeJson(doc, Serial);
    Serial.println();

    float thumbValue = doc["thumb"];
    float indexValue = doc["index"];
    float middleValue = doc["middle"];
    float ringValue = doc["ring"];
    float pinkyValue = doc["pinky"];

 
    smoothThumb  = alpha * thumbValue  + (1 - alpha) * smoothThumb;
    smoothIndex  = alpha * indexValue  + (1 - alpha) * smoothIndex;
    smoothMiddle = alpha * middleValue + (1 - alpha) * smoothMiddle;
    smoothRing   = alpha * ringValue   + (1 - alpha) * smoothRing;
    smoothPinky  = alpha * pinkyValue  + (1 - alpha) * smoothPinky;


    float thumbAngle = mapFloat(smoothThumb, 0.0, 0.19, 0, 180);

    float indexAngle = mapFloat(smoothIndex, 0.0, 0.19, 0, 180);

    float middleAngle = mapFloat(smoothMiddle, 0.0, 0.2, 0, 180);

    float ringAngle = mapFloat(smoothRing, 0.0, 0.19, 0, 180);

    float pinkyAngle = mapFloat(smoothPinky, 0.0, 0.19, 0, 180);

    //   Serial.println(angle);

    //   angle = constrain(angle, 0, 180);

    //   servo.write(angle);
    // }

    //int sensorValue = analogRead(sensorPin);
    //Serial.println(sensorValue);

    //  float angle = mapFloat(sensorValue, 0, 4095, 0, 180);

    //angle = constrain(angle, 0, 180);

    thumb.write(constrain(thumbAngle, 0, 180));
    indexf.write(constrain(indexAngle, 0, 180));
    middle.write(constrain(middleAngle, 0, 180));
    ring.write(constrain(ringAngle, 0, 180));
    pinky.write(constrain(pinkyAngle, 0, 180));


    // byte ledData = 0b00000000;

    // for (int j = 0; j < 8; j++) {
    //   if (indexValue >= threshold[j]) {
    //     ledData |= (1 << j);
    //   }
    // }

    // writeTo595(LSBFIRST, ledData);

    delay(10);   
  }
}

void writeTo595(int order, byte _data) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, order, _data);
  digitalWrite(latchPin, HIGH);
}
