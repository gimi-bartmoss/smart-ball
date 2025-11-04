#pragma once
#include <WiFi.h>

class WiFiServerModule {
public:
    WiFiServerModule(int port);
    void begin(const char* ssid, const char* password);
    void handleClient(String data);

private:
    WiFiServer server;
};
