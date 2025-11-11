//
// Created by julius on 08.09.2025.

#ifndef BODENSENSOR_BODENSENSOR_H
#define BODENSENSOR_BODENSENSOR_H

class BodenSensor {
public:
    struct Line {
        float rot;
        int progress;
        bool crossedMid;
    };

    static Line line;

    static void initSensor();

    static void updateLine();

private:
    struct Point { // point = Point
        float x;
        float y;
        float angle;
    };

    static std::array<Point, 32> _sensorPositions;

    static void initSensorPositions();

    static void setupPins();

    static void setMuxChannel(byte channel);

    static std::array<int, 32> getSensorDataArr();

    static std::vector<int> getActiveIndicesArr(const std::array<int, 32> &sensorData);

    static void computeClosestLineToCenter();
};

#endif //BODENSENSOR_BODENSENSOR_H