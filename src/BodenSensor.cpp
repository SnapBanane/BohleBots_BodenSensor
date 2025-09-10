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
  static double smoothedRot = 0.0;
  static int stableCount = 0;
  static bool pendingCross = false;
  static double crossCheckRot = 0.0;

  const double SMOOTHING_FACTOR = 0.3; // Smoothing für Rotation
  const double CROSS_THRESHOLD = 2.61799; // 150° für Kreuzungserkennung
  const int STABILITY_REQUIRED = 3; // Anzahl stabile Messungen vor Kreuzung

  computeClosestLineToCenter();

  if (line.progress < 0) {
    // Reset nur bei längerer Pause
    static int noLineCount = 0;
    noLineCount++;
    if (noLineCount > 5) { // Reset erst nach 5 Zyklen ohne Linie
      crossedMid = false;
      lastRot = 0.0;
      firstRun = true;
      smoothedRot = 0.0;
      stableCount = 0;
      pendingCross = false;
      noLineCount = 0;
    }
    line.percent = -1.0;
    line.crossedMid = crossedMid;
    return;
  }

  // Smoothing der Rotation
  if (firstRun) {
    smoothedRot = line.rot;
    firstRun = false;
  } else {
    // Smooth nur wenn Änderung < 90° (normale Bewegung)
    double deltaRot = std::fabs(line.rot - smoothedRot);
    if (deltaRot > M_PI) deltaRot = 2 * M_PI - deltaRot;

    if (deltaRot < M_PI_2) { // < 90° = normale Bewegung
      smoothedRot = smoothedRot * (1.0 - SMOOTHING_FACTOR) + line.rot * SMOOTHING_FACTOR;
      stableCount++;
    } else {
      // Große Änderung erkannt - prüfe auf Kreuzung
      if (deltaRot > CROSS_THRESHOLD && stableCount >= STABILITY_REQUIRED) {
        if (!pendingCross) {
          pendingCross = true;
          crossCheckRot = line.rot;
        } else {
          // Bestätige Kreuzung wenn weiterhin große Differenz
          double confirmDelta = std::fabs(line.rot - crossCheckRot);
          if (confirmDelta < M_PI_4) { // < 45° = bestätigt
            crossedMid = !crossedMid;
            smoothedRot = line.rot;
            pendingCross = false;
            stableCount = 0;
          }
        }
      }
    }
  }

  // Verwende geglättete Rotation für Ausgabe
  double outputRot = smoothedRot;
  if (crossedMid) {
    outputRot += M_PI;
    if (outputRot > M_PI) outputRot -= 2 * M_PI;
    if (outputRot < -M_PI) outputRot += 2 * M_PI;
  }

  line.rot = outputRot;
  lastRot = outputRot;

  // Progress Mapping wie vorher
  double mappedProgress;
  if (!crossedMid) {
    mappedProgress = (1.0 - line.progress) * 0.5;
  } else {
    mappedProgress = 0.5 + line.progress * 0.5;
  }

  line.percent = mappedProgress * 100.0;
  if (line.percent > 100.0) line.percent = 100.0;
  if (line.percent < 0.0) line.percent = 0.0;

  line.crossedMid = crossedMid;
}
