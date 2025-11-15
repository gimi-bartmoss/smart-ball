// SensorModule.h
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

class SensorModule {
public:
    void begin();

    // Called each loop to sample IMU and perform impact detection
    void update();

    // Returns true when an impact event has been detected
    bool hasImpact() const { return impactDetected; }

    // Clear impact flag after data has been processed
    void clearImpact() { impactDetected = false; }

    // Export buffered IMU data as a text block for TCP transmission
    String exportBufferedData();

    // Set impact threshold (from WiFi command)
    void setThreshold(float t) { impactThreshold = t; }

    float getThreshold() const { return impactThreshold; }

    String getLatestData() const;

private:
    // Structure to store one IMU sample in float form
    struct IMUSample {
        float ax, ay, az;
        float gx, gy, gz;
        float temp;
    };

    static const int BUFFER_SIZE = 4000; 
    // Ring buffer to store the last N IMU samples
    IMUSample buffer[BUFFER_SIZE];
    int head = 0;   // Index of the oldest sample in the ring buffer
    int count = 0;  // Number of valid samples currently stored

    Adafruit_MPU6050 mpu;

    // Impact detection threshold (m/s^2)
    float impactThreshold = 35.0f;

    // Set to true when |a| exceeds impactThreshold
    bool impactDetected = false;

    // Push one IMU sample into the ring buffer
    void pushSample(const IMUSample& s);

    // Compute magnitude of acceleration vector |a|
    float computeAccelNorm(const IMUSample& s) const;
};
