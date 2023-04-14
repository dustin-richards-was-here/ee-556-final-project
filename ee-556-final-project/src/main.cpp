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

// filtering for the encoder values
constexpr float filterCutoffFreq = 50; // Hz, I think
FilterOnePole filter(LOWPASS, filterCutoffFreq);

// timing for velocity calculation from encoder data
int32_t lastPos = 0;
uint32_t lastEncoderTime = 0;

// serial timing for data logging
uint32_t lastPrintTime = 0;
constexpr int printPeriod = 5; // ms

// parameters for stepping through different power outputs
//#define STEP_INCREMENT
#ifdef STEP_INCREMENT
  uint32_t lastToggleTime = 0;
  constexpr int togglePeriod = 2000; // ms
  constexpr int NUM_POWERS = 4;
  constexpr int MAX_POWER = 255;
  constexpr int MIN_POWER = 0;
  constexpr float POWER_INCREMENT = float(MAX_POWER - MIN_POWER) / NUM_POWERS;
  float power = MIN_POWER;
#else
  uint32_t lastToggleTime = 0;
  constexpr int togglePeriod = 2000; // ms
  constexpr int NUM_POWERS = 6;
  constexpr int POWERS[NUM_POWERS] = {0, 100, 0, 180, 0, 255};
  float power = POWERS[0];
  int powerIndex = 0;
#endif

void setup() {
  motor.init();
  motor.setSpeed(power);

  Serial.begin(115200);
  Serial.println("BEGIN");
  lastEncoderTime = micros();
}

void loop() {
  // read the current rotational position of the encoder
  int32_t pos = encoder.read();

  // calculate the velocity
  int32_t encoderTime = micros();
  float vel = float(pos - lastPos) / float(encoderTime - lastEncoderTime) * 1000000;
  lastPos = pos;
  lastEncoderTime = encoderTime;

  // filter the velocity
  filter.input(vel);

  // log data to the serial port every printPeriod milliseconds
  if (millis() - lastPrintTime > printPeriod) {
    // time in ms
    Serial.print(millis());
    Serial.print(",");
    // output shaft velocity in revs/sec
    Serial.print(filter.Y / ENCODER_SCALE);
    Serial.print(",");
    // current output power [-255,255]
    Serial.println(int(power));
    lastPrintTime = millis();
  }

  // adjust motor power every togglePeriod milliseoncds
  if (millis() - lastToggleTime > togglePeriod) {
    #ifdef STEP_INCREMENT
      power += POWER_INCREMENT;
      if (power > MAX_POWER)
        power = MIN_POWER;
    #else
      powerIndex++;
      if (powerIndex >= NUM_POWERS)
        powerIndex = 0;
      power = POWERS[powerIndex];
    #endif
    motor.setSpeedDirection(int(power));
    lastToggleTime = millis();
  }
}