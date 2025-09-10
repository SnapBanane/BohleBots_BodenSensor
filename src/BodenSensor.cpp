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

  if (activeSensorIndices.size() < 1) {
    line.progress = -1.0;
    line.rot = 0.0;
    lastDist = -1.0;
    lastRot = 0.0;
    return;
  }

  if (activeSensorIndices.size() == 1) {
    const int idx = activeSensorIndices[0];
    const double x = sensorPositions[idx].x;
    const double y = sensorPositions[idx].y;

    // Compute normal perpendicular to the radius vector
    double mag = std::sqrt(x*x + y*y);
    if (mag == 0.0) mag = 1.0; // prevent divide by zero
    double normalX = -y / mag;
    double normalY = x / mag;

    // Signed distance from center (0,0) to the line perpendicular to radius through sensor
    double signedDist = 0.0 * normalX + 0.0 * normalY; // still 0 for center, but keeps sign logic consistent
    // Here signedDist will be used for mapping percent later

    // Map -1..1 -> 0..1 (for percent)
    double mappedDist = (-signedDist + 1.0) / 2.0;
    mappedDist = std::max(0.0, std::min(1.0, mappedDist));

    line.progress = mappedDist;
    line.rot = std::atan2(y, x); // vector from center to sensor
    return;
  }


  // Multiple sensors - find best pair with maximum separation
  std::array<point, 2> bestPair = {};
  double bestSignedDist = 0.0;
  double bestRot = 0.0;
  int maxIndexDistance = 0;

  for (size_t i = 0; i < activeSensorIndices.size(); ++i) {
    for (size_t j = i + 1; j < activeSensorIndices.size(); ++j) {
      const int p1 = activeSensorIndices[i];
      const int p2 = activeSensorIndices[j];

      // Calculate index distance (considering circular array)
      int indexDist = std::abs(p2 - p1);
      if (indexDist > 16) indexDist = 32 - indexDist; // wrap around for circular array

      // Choose pair with maximum index distance
      if (indexDist > maxIndexDistance) {
        maxIndexDistance = indexDist;

        // Line endpoints
        point lineP1 = sensorPositions[p1];
        point lineP2 = sensorPositions[p2];

        // Calculate signed distance from robot center (0,0) to the line between p1 and p2
        double dx = lineP2.x - lineP1.x;
        double dy = lineP2.y - lineP1.y;

        // Project robot center onto line
        double t = (0 - lineP1.x) * dx + (0 - lineP1.y) * dy;
        t /= (dx * dx + dy * dy);
        point closest = {lineP1.x + t * dx, lineP1.y + t * dy};

        // Calculate signed distance using line normal
        double mag = std::sqrt(dx*dx + dy*dy);
        double signedDist = 0.0;
        if (mag > 0) {
          double normalX = -dy / mag;  // perpendicular to line direction
          double normalY = dx / mag;
          signedDist = (0 - lineP1.x) * normalX + (0 - lineP1.y) * normalY;
        }

        // Map signed distance to [0, 1] range
        // left side (-1) → 1.0, center (0) → 0.5, right side (+1) → 0.0
        bestSignedDist = (-signedDist + 1.0) / 2.0;
        bestSignedDist = std::max(0.0, std::min(1.0, bestSignedDist));

        // Vector from center to closest point on line
        bestRot = std::atan2(closest.y, closest.x);
      }
    }
  }

  line.progress = bestSignedDist;
  line.rot = bestRot;
  lastDist = line.progress;
  lastRot = line.rot;
}

void BodenSensor::updateLine() {
  static bool firstRun = true;
  static int runCount = 0;

  computeClosestLineToCenter();

  // Reset wenn keine Linie erkannt
  if (line.progress < 0) {
    lastRot = 0.0;
    firstRun = true;
    line.percent = -1.0;
    if (runCount > 5) {
      line.crossedMid = false;
      crossedMid = false;
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
