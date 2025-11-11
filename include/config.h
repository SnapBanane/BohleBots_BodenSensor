//
// Created by julius on 08.09.2025.
//

#ifndef BODENSENSOR_CONFIG_H
#define BODENSENSOR_CONFIG_H
#include <Arduino.h>

constexpr std::array<byte, 3> addressPins = {14, 27, 13}; // mux pins
constexpr std::array<byte, 4> outputPins = {26, 33, 32, 25};

constexpr byte ledPin = 2; // led pins
constexpr std::array CHANNEL_ORDER = {4, 6, 7, 5, 3, 0, 1, 2}; // order to read the channels cause the physical layout is cursed

constexpr int THRESHOLD = 100; // sensor threshold for active state

constexpr int TARGET_ADDRESS = 0x20; // i2c address to send data to

constexpr int I2C_ADRESS = 0x10;

#endif //BODENSENSOR_CONFIG_H