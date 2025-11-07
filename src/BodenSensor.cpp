//
// Created by julius on 08.09.2025.
//

#include <Arduino.h>
#include "../include/BodenSensor.h"
#include "../include/config.h"
#include "../include/movingAverage.h"
#include <array>
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
    // Pre-calculate angle in degrees for later use (avoid atan2 in hot path)
    double angleDeg = angle * 180.0 / M_PI;
    sensorPositions[i].angle = angleDeg;
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
  std::array<int, 32> sensorData = {};

  for (int i = 0; i < 8; i++) {
    setMuxChannel(CHANNEL_ORDER[i]);
    for (int j = 0; j < 4; j++) {
      sensorData[j*8+i] = analogRead(outputPins[j]);
    }
  }
  
  if (_delay > 0) {
    delay(_delay);
  }

  return sensorData;
}

int BodenSensor::getActiveIndicesArr(const std::array<int, 32>& sensorData, int* activeIndices)
{
  int count = 0;
  for (int i = 0; i < 32; ++i) {
    if (sensorData[i] > THRESHOLD) {
      activeIndices[count++] = i;
      if (count >= 32) break;  // Safety check to prevent buffer overflow
    }
  }
  return count;
}

void BodenSensor::computeClosestLineToCenter() {
  constexpr int _delay = 0; // delay in ms
  const std::array<int, 32> sensorData = BodenSensor::getSensorDataArr(_delay);

  int activeIndices[32];  // Fixed-size array instead of vector
  const int activeCount = BodenSensor::getActiveIndicesArr(sensorData, activeIndices);

  if (activeCount < 1) { // if no sensors active return -1 we can later detect that
    line.progress = -1.0;
    line.rot = -1.0;
    line.crossedMid = false;
    rotAvg.reset();
    return;
  }

  if (activeCount == 1) {
    line.progress = 1.0;
    // Use pre-calculated angle
    line.rot = static_cast<float>(sensorPositions[activeIndices[0]].angle + 90.0);
    if (line.rot >= 360.0f) line.rot -= 360.0f;
    return;
  }

  int maxIndexDist = 0;
  int bestP1 = -1, bestP2 = -1; // init empty

  // Optimize nested loops
  for (int i = 0; i < activeCount; ++i) {
    const int p1 = activeIndices[i];
    for (int j = i + 1; j < activeCount; ++j) {
      const int p2 = activeIndices[j];

      int indexDist = std::abs(p2 - p1);
      if (indexDist > 16) {
        indexDist = 32 - indexDist;
      }

      if (indexDist > maxIndexDist) {
        maxIndexDist = indexDist;
        line.progress = indexDist;
        bestP1 = p1;
        bestP2 = p2;
      }
    }
  }

  if (bestP1 != -1 && bestP2 != -1) {
    // Use pre-calculated angles
    double angle1 = sensorPositions[bestP1].angle;
    double angle2 = sensorPositions[bestP2].angle;

    // angle rotation diff
    double diff = angle1 - angle2;

    if (diff > 180.0) diff -= 360.0;
    if (diff < -180.0) diff += 360.0;

    line.rot = static_cast<float>(angle2 + diff / 2.0);

    line.rot -= 45.0f;

    if (line.rot < 0.0f) line.rot += 360.0f;
    if (line.rot >= 360.0f) line.rot -= 360.0f;
  }
}

void BodenSensor::updateLine() {
  const float lastRotSave = line.rot;

  computeClosestLineToCenter();

  if (lastRotSave == -1.0f) { // catch line not detected to prevent insta jump
    line.crossedMid = false;
    return;
  }
  static bool state = false;

  float diff = lastRotSave - line.rot;

  // Faster angle normalization
  if (diff > 180.0f) diff -= 360.0f;
  else if (diff < -180.0f) diff += 360.0f;

  if (std::abs(diff) > 120.0f) { 
    state = !state;  
    line.crossedMid = state;
  }

  if (line.crossedMid) {
    line.progress = 32 - line.progress;
  }
}
