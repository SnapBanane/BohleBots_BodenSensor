//
// Created by julius on 05.11.2025.
//

#ifndef BODENSENSOR_MOVINGAVERAGE_TPP
#define BODENSENSOR_MOVINGAVERAGE_TPP

#include "movingAverage.h"

template<size_t N>
MovingAverage<N>::MovingAverage() : index(0), count(0), sum(0.0) {
    for (size_t i = 0; i < N; ++i) {
        buffer[i] = 0.0;
    }
}

template<size_t N>
void MovingAverage<N>::add(double value) {
    sum -= buffer[index];
    buffer[index] = value;
    sum += value;
    index = (index + 1) % N;
    if (count < N) count++;
}

template<size_t N>
double MovingAverage<N>::get() const {
    if (count == 0) return 0.0;
    return sum / count;
}

template<size_t N>
void MovingAverage<N>::reset() {
    for (size_t i = 0; i < N; ++i) {
        buffer[i] = 0.0;
    }
    index = 0;
    count = 0;
    sum = 0.0;
}

#endif //BODENSENSOR_MOVINGAVERAGE_TPP
//