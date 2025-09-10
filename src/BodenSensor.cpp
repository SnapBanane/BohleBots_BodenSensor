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
    if (sensorData[i] > 500) {
      activeIndices.push_back(i);
    }
  }
  return activeIndices;
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
        line.rot = std::atan2(my, mx);
      }
    }
  }

  lastDist = line.progress;
  lastRot = line.rot;
}

void BodenSensor::updateLine() {
  static bool firstRun = true;
  static int runCount = 0;

  computeClosestLineToCenter();

  // Reset wenn keine Linie erkannt
  if (line.progress < 0) {
    crossedMid = false;
    lastRot = 0.0;
    firstRun = true;
    line.percent = -1.0;
    if (runCount > 5) {
      line.crossedMid = false;
    }
    runCount++;
    return;
  }
  runCount = 0;

  // Check für plötzliche Rotation (Linienübergang)
  if (!firstRun) {
    double deltaRot = std::fabs(line.rot - lastRot);
    if (deltaRot > M_PI) deltaRot = 2 * M_PI - deltaRot; // Normalisiere auf [0, PI]

    if (deltaRot > 2.61799) { // >150° = Linienübergang erkannt
      crossedMid = true;
    }
  }
  firstRun = false;

  // Wenn Linie gekreuzt wurde, rotiere alle zukünftigen Rotationen um 180°
  if (crossedMid) {
    line.rot += M_PI;
    if (line.rot > M_PI) line.rot -= 2 * M_PI;
    if (line.rot < -M_PI) line.rot += 2 * M_PI;
  }

  lastRot = line.rot;

  // Mappe progress: vor Kreuzung 0-0.5, nach Kreuzung 0.5-1.0
  double mappedProgress;
  if (!crossedMid) {
    // Vor Kreuzung: 1.0 -> 0.0, 0.0 -> 0.5
    mappedProgress = (1.0 - line.progress) * 0.5;
  } else {
    // Nach Kreuzung: 0.0 -> 0.5, 1.0 -> 1.0
    mappedProgress = 0.5 + line.progress * 0.5;
  }

  // Berechne Prozent (0-100)
  line.percent = mappedProgress * 100.0;
  if (line.percent > 100.0) line.percent = 100.0;
  if (line.percent < 0.0) line.percent = 0.0;

  // Speichere Status
  line.crossedMid = crossedMid;
}
