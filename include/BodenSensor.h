//
// Created by julius on 08.09.2025.

#ifndef BODENSENSOR_BODENSENSOR_H
#define BODENSENSOR_BODENSENSOR_H
#include <Arduino.h>

class BodenSensor
{
public:

  struct point {
    double x;
    double y;
  };

  struct Line
  {
    double rot;
    double dist;
  };

  // variables
  static std::array<point, 32> sensorPositions;
  static Line line;

  // functions
  static void initSensorPositions();
  static void setupPins();
  static void setMuxChannel(byte channel);
  static std::array<int, 32> getSensorDataArr(int _delay);
  static std::vector<int> getActiveIndicesArr(const std::array<int, 32>& sensorData);
  static void computeClosestLineToCenter(const std::vector<int>& activeIndices);
};


#endif //BODENSENSOR_BODENSENSOR_H