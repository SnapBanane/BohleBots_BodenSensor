#include <Arduino.h>
#include "BodenSensor.h"
#include <chrono>
#include <config.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADRESS);
  Wire.onRequest(requestEvent);
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

void requestEvent() {
  // Sende progress (4 Bytes als float)
  Wire.write((byte*)&BodenSensor::line.progress, sizeof(float));
  // Sende rot (4 Bytes als float)
  Wire.write((byte*)&BodenSensor::line.rot, sizeof(float));
}

void loop() {
  // const auto startTime = std::chrono::high_resolution_clock::now();

  BodenSensor::updateLine();

  Wire.beginTransmission(TARGET_ADDRESS);
  Wire.write((byte*)&BodenSensor::line.progress, sizeof(float));
  Wire.write((byte*)&BodenSensor::line.rot, sizeof(float));
  Wire.endTransmission();

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
