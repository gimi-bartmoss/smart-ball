#include "SensorModule.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void SensorModule::begin() {
    Wire.begin(9, 8);
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip!");
        while (1) { delay(10); }
    }
    Serial.println("MPU6050 initialized.");

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    delay(100);
}

String SensorModule::getIMUData() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    String data = "AX:" + String(a.acceleration.x, 2) +
                  ", AY:" + String(a.acceleration.y, 2) +
                  ", AZ:" + String(a.acceleration.z, 2) +
                  ", GX:" + String(g.gyro.x, 2) +
                  ", GY:" + String(g.gyro.y, 2) +
                  ", GZ:" + String(g.gyro.z, 2) +
                  ", T:" + String(temp.temperature, 2)
                  + "\n";
    return data;
}
