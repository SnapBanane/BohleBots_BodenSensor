//
// Created by julius on 08.09.2025.
//

#include <Arduino.h>
#include "../include/BodenSensor.h"
#include "../include/config.h"
#include "../include/movingAverage.h"
#include <array>
#include <vector>
#include <cmath>

std::array<BodenSensor::point, 32> BodenSensor::sensorPositions;
BodenSensor::Line BodenSensor::line = {};

MovingAverage<10> rotAvg;

void BodenSensor::setupPins()
{
  for (const unsigned char addressPin : addressPins) {
    pinMode(addressPin, OUTPUT);
  }

  for (const unsigned char outputPin : outputPins) {
    pinMode(outputPin, OUTPUT);
  }

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // enable led ring
}

void BodenSensor::initSensorPositions()
{
  for (int i = 0; i < 32; ++i)
  {
    const double angle = 2 * M_PI * i / 32;
    sensorPositions[i].x = std::cos(angle);
    sensorPositions[i].y = std::sin(angle);
  }
}

void BodenSensor::setMuxChannel(const byte channel)
{
  for (byte i = 0; i < addressPins.size(); i++) {
    digitalWrite(addressPins[i], (channel >> i) & 0x01);
  }
}

std::array<int, 32> BodenSensor::getSensorDataArr(const int _delay)
{
  // digitalWrite(2, HIGH);

  std::array<int, 32> sensorData = {};
  sensorData.fill(0);

  for (int i = 0; i < 8; i++) {
    const int channel = CHANNEL_ORDER[i];
    setMuxChannel(channel);
    for (int j = 0; j < 4; j++) {
      sensorData[j*8+i] = analogRead(outputPins[j]);
    }
  }
  delay(_delay);
  // digitalWrite(2, LOW);

  return sensorData;
}

std::vector<int> BodenSensor::getActiveIndicesArr(const std::array<int, 32>& sensorData)
{
  std::vector<int> activeIndices;
  for (size_t i = 0; i < sensorData.size(); ++i) {
    if (sensorData[i] > THRESHOLD) {
      activeIndices.push_back(i);
    }
  }
  return activeIndices;
}

// helper
double unwrapAngle(double current, double previous) {
  double diff = current - previous;
  while (diff > M_PI)  diff -= 2 * M_PI;
  while (diff < -M_PI) diff += 2 * M_PI;
  return previous + diff;
}

void BodenSensor::computeClosestLineToCenter() {
  constexpr int _delay = 0; // delay in ms
  const std::array<int, 32> sensorData = BodenSensor::getSensorDataArr(_delay);

  const std::vector<int> activeSensorIndices = BodenSensor::getActiveIndicesArr(sensorData);

  int maxIndexDist = 0;

  if (activeSensorIndices.size() < 1) { // if no sensors active return -1 we can later detect that
    line.progress = -1.0;
    line.rot = -1.0;
    line.crossedMid = false;
    rotAvg.reset();
    return;
  }

  if (activeSensorIndices.size() == 1) {
    line.progress = 1.0;
    const double x = sensorPositions[activeSensorIndices[0]].x;
    const double y = sensorPositions[activeSensorIndices[0]].y;
    line.rot = static_cast<float>(std::atan2(y, x) + M_PI_2) *180.0f / static_cast<float>(M_PI);
    if (line.rot < 0) line.rot += 360.0f; // untested
    return;
  }

  int bestP1 = -1, bestP2 = -1; // init empty

  for (auto i = 0; i < activeSensorIndices.size(); ++i) {
    for (auto j = i + 1; j < activeSensorIndices.size(); ++j) {
      const int p1 = activeSensorIndices[i];
      const int p2 = activeSensorIndices[j];

      int indexDist = std::abs(p2 - p1);
      indexDist = std::min(indexDist, 32 - indexDist);

      if (indexDist > 16) continue;

      if (indexDist > maxIndexDist) {
        maxIndexDist = indexDist;
        line.progress = indexDist;
        bestP1 = p1;
        bestP2 = p2;
      }
    }
  }

  if (bestP1 != -1 && bestP2 != -1) {
    double angle1 = atan2(sensorPositions[bestP1].x, sensorPositions[bestP1].y) * 180.0 / M_PI;
    double angle2 = atan2(sensorPositions[bestP2].x, sensorPositions[bestP2].y) * 180.0 / M_PI;

    if (angle1 < 0) angle1 += 360.0;
    if (angle2 < 0) angle2 += 360.0;

    // angle rotation diff
    double diff = angle1 - angle2;

    if (diff > 180.0) diff -= 360.0;
    if (diff < -180.0) diff += 360.0;

    line.rot = static_cast<float>(static_cast<float>(angle2) + diff / 2.0);

    line.rot -= 45.0;

    if (line.rot < 0) line.rot += 360.0;
    if (line.rot >= 360.0) line.rot -= 360.0;
  }
}

void BodenSensor::updateLine() {
  const double lastRotSave = line.rot;

  computeClosestLineToCenter();

  if (lastRotSave == -1) { // catch line not detected to prevent insta jump
    line.crossedMid = false;
    return;
  }
  static boolean state = false;

  double diff = lastRotSave - line.rot;

  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;

  if (std::abs(diff) > 120) { state = !state;  line.crossedMid = state;}

  if (line.crossedMid) {
    line.progress = 32 - line.progress;
  }

  //rotAvg.add(line.rot);

}
