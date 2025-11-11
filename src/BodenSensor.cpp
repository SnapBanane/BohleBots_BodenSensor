//
// Created by julius on 08.09.2025.
//

#include <Arduino.h>
#include "../include/BodenSensor.h"
#include "../include/config.h"
#include <array>
#include <vector>
#include <cmath>

std::array<BodenSensor::Point, 32> BodenSensor::_sensorPositions;
BodenSensor::Line BodenSensor::line = {};

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
    const auto angle = static_cast<float>(2 * M_PI * i / 32);
    _sensorPositions[i].x = std::cos(angle);
    _sensorPositions[i].y = std::sin(angle);

    const auto sensorAngle = static_cast<float>(angle * 180.0 / M_PI);
    _sensorPositions[i].angle = sensorAngle;
  }
}

void BodenSensor::setMuxChannel(const byte channel)
{
  for (std::size_t i = 0; i < addressPins.size(); i++) {
    digitalWrite(addressPins[i], channel >> i & 0x01);
  }
}

std::array<int, 32> BodenSensor::getSensorDataArr()
{
  std::array<int, 32> sensorData = {};
  sensorData.fill(0);

  for (int i = 0; i < 8; i++) {
    const int channel = CHANNEL_ORDER[i];
    setMuxChannel(channel);
    for (int j = 0; j < 4; j++) {
      sensorData[j*8+i] = analogRead(outputPins[j]);
    }
  }

  return sensorData;
}

std::vector<int> BodenSensor::getActiveIndicesArr(const std::array<int, 32>& sensorData)
{
  std::vector<int> activeIndices;
  for (size_t i = 0; i < sensorData.size(); ++i) {
    if (sensorData[i] > THRESHOLD) {
      activeIndices.push_back(static_cast<int>(i));
    }
  }
  return activeIndices;
}

void BodenSensor::initSensor() {
  setupPins();
  initSensorPositions();
}

// helper
double unwrapAngle(const float current, const float previous) {
  float diff = current - previous;
  while (diff > M_PI)  diff -= 2 * M_PI;
  while (diff < -M_PI) diff += 2 * M_PI;
  return previous + diff;
}

void BodenSensor::computeClosestLineToCenter() {
  const std::array<int, 32> sensorData = getSensorDataArr();

  const std::vector<int> activeSensorIndices = getActiveIndicesArr(sensorData);

  int maxIndexDist = 0;

  if (activeSensorIndices.empty()) {
    line.progress = -1.0;
    line.rot = -1.0;
    line.crossedMid = false;
    return;
  }

  if (activeSensorIndices.size() == 1) {
    line.progress = 1.0;
    const double x = _sensorPositions[activeSensorIndices[0]].x;
    const double y = _sensorPositions[activeSensorIndices[0]].y;
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
    float angle1 = _sensorPositions[bestP1].angle;
    float angle2 = _sensorPositions[bestP2].angle;

    if (angle1 < 0) angle1 += 360.0;
    if (angle2 < 0) angle2 += 360.0;

    // angle rotation diff
    float diff = angle1 - angle2;

    if (diff > 180.0) diff -= 360.0;
    if (diff < -180.0) diff += 360.0;

    line.rot = static_cast<float>(angle2 + diff / 2.0);

    line.rot -= 45.0;

    if (line.rot < 0) line.rot += 360.0;
    if (line.rot >= 360.0) line.rot -= 360.0;
  }
}

void BodenSensor::updateLine() {
  const float lastRotSave = line.rot;

  computeClosestLineToCenter();

  if (lastRotSave == -1) { // catch line not detected to prevent insta jump
    line.crossedMid = false;
    return;
  }
  static boolean state = false;

  float diff = lastRotSave - line.rot;

  if (diff > 180) diff -= 360.0f;
  else if (diff < -180) diff += 360.0f;

  if (std::abs(diff) > 120) { state = !state;  line.crossedMid = state;}

  if (line.crossedMid) {
    line.progress = 32 - line.progress;
  }
}
