#include <Arduino.h>
#include "BodenSensor.h"
#include "config.h"
#include <chrono>

void setup() {
  Serial.begin(115200);
  // Setup all the Pins with correct modes
  BodenSensor::setupPins();
  BodenSensor::initSensorPositions();

  // some random blinking to show that the program is running
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);
  digitalWrite(2, HIGH);

}

void loop() {
  // const auto startTime = std::chrono::high_resolution_clock::now();

  BodenSensor::updateLine();

  /*
  Serial.print(BodenSensor::line.progress);
  Serial.print(",");
  Serial.print(BodenSensor::line.rot);
  Serial.print(",");
  Serial.print(BodenSensor::line.crossedMid);
  Serial.println();
  */

  // const auto endTime = std::chrono::high_resolution_clock::now();
  // const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  // Serial.println(duration.count());

  delay(1);
}
