#pragma once
#include <Arduino.h>

class SensorModule {
public:
    void begin();
    String getIMUData();
};
