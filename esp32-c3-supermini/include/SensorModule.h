#pragma once
#include <Arduino.h>

struct IMUData {
    uint32_t timestamp;
    float ax, ay, az;
    float gx, gy, gz;
    float temp;
};

class SensorModule {
public:
    void begin();
    IMUData getIMUData();
};