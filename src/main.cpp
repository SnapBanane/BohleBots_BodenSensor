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

  BodenSensor::updateLine();

  const std::array<int, 32> sensorData = BodenSensor::getSensorDataArr(1);

  /*
  for (const std::vector<int> activeIndices = BodenSensor::getActiveIndicesArr(sensorData); const auto i : activeIndices) {
    Serial.print(i);
    Serial.print(",");
  }
  Serial.println();

  Serial.print(BodenSensor::line.progress);
  Serial.print(",");
  Serial.print(BodenSensor::line.rot);
  Serial.print(",");
  */
  Serial.print(BodenSensor::line.progress);
  Serial.print(",");
  Serial.print(BodenSensor::line.rot);
  Serial.print(",");
  Serial.print(BodenSensor::line.crossedMid);
  Serial.println();
  /*
  for (int i = 0; i < sensorData.size(); i++)
  {
    Serial.print(sensorData[i]);
    Serial.print(",");
  }
  Serial.println();
  */

  delay(1);
}
