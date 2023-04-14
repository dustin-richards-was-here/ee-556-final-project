#include <Arduino.h>
#include "Encoder.h"
#include "Filters.h"
#include "L289N.h"

L289N motor(8, 9, 10);

//// counts/rev of the output shaft
//// this is not super precise, but neither is this project
constexpr int ENCODER_SCALE = 1900 * 2;

constexpr int ENCODER_PIN_A = 2;
constexpr int ENCODER_PIN_B = 3;

Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

constexpr float filterCutoffFreq = 50; // Hz, I think
FilterOnePole filter(LOWPASS, filterCutoffFreq);

int32_t lastPos = 0;
uint32_t lastEncoderTime = 0;

uint32_t lastPrintTime = 0;
constexpr int printPeriod = 5; // ms

uint32_t lastToggleTime = 0;
constexpr int togglePeriod = 5000; // ms
int speed = 255;

void setup() {
  motor.init();

  Serial.begin(115200);
  Serial.println("BEGIN");
  lastEncoderTime = micros();
}

void loop() {
  int32_t pos = encoder.read();
  int32_t encoderTime = micros();
  float vel = float(pos - lastPos) / float(encoderTime - lastEncoderTime) * 1000000;
  lastPos = pos;
  lastEncoderTime = encoderTime;

  filter.input(vel);

  if (millis() - lastPrintTime > printPeriod) {
    Serial.print(millis());
    Serial.print(",");
    Serial.print(filter.Y / ENCODER_SCALE);
    Serial.print(",");
    Serial.println(speed);
    lastPrintTime = millis();
  }

  if (millis() - lastToggleTime > togglePeriod) {
    motor.setSpeedDirection(speed);
    speed *= -1;
    lastToggleTime = millis();
  }
}