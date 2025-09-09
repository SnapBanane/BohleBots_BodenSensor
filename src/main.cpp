#include <Arduino.h>
#include "BodenSensor.h"
#include "config.h"

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

  digitalWrite(2, HIGH);
  const std::array<int, 32> sensorData = BodenSensor::getSensorDataArr();
  delay(10);
  digitalWrite(2, LOW);

  const std::vector<int> activeIndices = BodenSensor::getActiveIndicesArr(sensorData);
  BodenSensor::computeClosestLineToCenter(activeIndices);

  for (const auto i : activeIndices) {
    Serial.print(i);
    Serial.print(",");
  }
  Serial.print("|");
  Serial.print(BodenSensor::line.dist);
  Serial.print(",");
  Serial.print(BodenSensor::line.rot);
  Serial.println();

  delay(1);
}
