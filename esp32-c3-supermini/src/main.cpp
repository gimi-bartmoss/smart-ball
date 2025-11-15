// main.cpp
// TODO: create config.h to handle constants like SSID, password, port, initial threshold

#include <Arduino.h>
#include "WiFiServerModule.h"
#include "SensorModule.h"

const char* ssid = "SmartBall_AP";
const char* password = "12345678";

WiFiServerModule wifiServer(8080);
SensorModule sensor;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("SmartBall system starting...");

    wifiServer.begin(ssid, password);
    sensor.begin();

    // Sync sensor threshold with WiFi module initial value
    sensor.setThreshold(wifiServer.getImpactThreshold());
}

void loop() {
    // Handle incoming WiFi commands
    wifiServer.update();

    // If TCP command has updated threshold, apply it to sensor
    if (wifiServer.hasNewThreshold()) {
        float th = wifiServer.getImpactThreshold();
        sensor.setThreshold(th);
        Serial.print("Applied new threshold to sensor: ");
        Serial.println(th);
    }

    // Sample IMU and perform impact detection
    sensor.update();

    // Only send IMU data after an impact event has been detected
    if (sensor.hasImpact()) {
        Serial.println("Impact detected! Exporting IMU buffer...");

        String payload = sensor.exportBufferedData();
        wifiServer.sendData(payload);

        sensor.clearImpact();
        Serial.println("Impact event handled, sensor ready for next throw.");
    }

    delay(5);
}