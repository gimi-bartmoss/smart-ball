#include <Arduino.h>
#include "WiFiServerModule.h"
#include "SensorModule.h"

const char* ssid = "SmartBall_AP";
const char* password = "12345678";

WiFiServerModule wifiServer(8080);
SensorModule sensor;

void setup() {
    Serial.begin(115200);
    wifiServer.begin(ssid, password);
    sensor.begin();
}

void loop() {
    String data = sensor.getIMUData();
    wifiServer.handleClient(data);
    Serial.print("Sent: ");
    Serial.print(data);
    delay(500);
}
