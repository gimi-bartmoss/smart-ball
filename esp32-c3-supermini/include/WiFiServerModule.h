#pragma once
#include <WiFi.h>
#include "SensorModule.h"

class WiFiServerModule {
public:
    WiFiServerModule(int port);
    void begin(const char* ssid, const char* password);
    void handleClient(const IMUData* data);

private:
    WiFiServer server;
    WiFiClient client;
};
