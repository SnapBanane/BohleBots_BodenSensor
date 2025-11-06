//
// Created by julius on 05.11.2025.
//

#ifndef BODENSENSOR_MOVINGAVERAGE_H
#define BODENSENSOR_MOVINGAVERAGE_H
#include <cstddef>

template<size_t N>
class MovingAverage {
private:
  double buffer[N];
  size_t index;
  size_t count;
  double sum;

public:
  MovingAverage();
  void add(double value);
  double get() const;
  void reset();
};

#include "movingAverage.cpp"
#endif //BODENSENSOR_MOVINGAVERAGE_H