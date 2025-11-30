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
    Serial.println("=== BOOT ===");
    wifiServer.begin(ssid, password);
    sensor.begin();
}

void loop() {
    IMUData data = sensor.getIMUData();
    wifiServer.handleClient(&data);
    Serial.print("Sent data with timestamp: ");
    Serial.println(data.timestamp);
    delay(40);  // Interval
}
