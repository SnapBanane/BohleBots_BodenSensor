//
// Created by julius on 08.09.2025.
//

#include <Arduino.h>
#include "../include/BodenSensor.h"
#include "../include/config.h"
#include <array>
#include <vector>
#include <cmath>

// Define variables
std::array<BodenSensor::point, 32> BodenSensor::sensorPositions;
BodenSensor::Line BodenSensor::line = {};

static double lastDist = -1.0;
static double lastRot = 0.0;
static bool crossedMid = false;
static bool wasBelowMid = false;

void BodenSensor::setupPins()
{
  for (const unsigned char addressPin : addressPins) {
    pinMode(addressPin, OUTPUT);
  }

  for (const unsigned char outputPin : outputPins) {
    pinMode(outputPin, OUTPUT);
  }

  pinMode(2, OUTPUT);
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
  digitalWrite(2, HIGH);

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
  digitalWrite(2, LOW);

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
  constexpr int _delay = 50; // delay in ms
  const std::array<int, 32> sensorData = BodenSensor::getSensorDataArr(_delay);

  const std::vector<int> activeSensorIndices = BodenSensor::getActiveIndicesArr(sensorData);

  std::array<point, 2> bestPair = {};
  double minDist = std::numeric_limits<double>::max();

  if (activeSensorIndices.size() < 1) { // if no sensors active return -1 we can later detect that
    line.progress = -1.0;
    line.rot = 0.0;
    lastDist = -1.0;
    lastRot = 0.0;
    return;
  }

  if (activeSensorIndices.size() == 1) {
    line.progress = 1.0;
    const double x = sensorPositions[activeSensorIndices[0]].x;
    const double y = sensorPositions[activeSensorIndices[0]].y;
    line.rot = std::atan2(y, x) + M_PI_2;
    return;
  }

  for (auto i = 0; i < activeSensorIndices.size(); ++i) {
    for (auto j = i + 1; j < activeSensorIndices.size(); ++j) {
      const int p1 = activeSensorIndices[i];
      const int p2 = activeSensorIndices[j];
      const double mx = (sensorPositions[p1].x + sensorPositions[p2].x) / 2.0;
      const double my = (sensorPositions[p1].y + sensorPositions[p2].y) / 2.0;

      if (const double distance = std::sqrt(mx * mx + my * my); distance < minDist) {
        minDist = distance;
        bestPair[0] = sensorPositions[p1];
        bestPair[1] = sensorPositions[p2];

        // set line properties
        line.progress = minDist; // will get remapped later
        double angle = std::atan2(my, mx);
        line.rot = angle;
        // line.rot = angle;
      }
    }
  }

  lastDist = line.progress;
  lastRot = line.rot;
}

void BodenSensor::updateLine() {
  computeClosestLineToCenter();
}
