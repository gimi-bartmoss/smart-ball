#pragma once
#include <Arduino.h>
#include <WiFi.h>

class WiFiServerModule {
public:
    explicit WiFiServerModule(uint16_t port);

    void begin(const char* ssid, const char* password);

    // Handle incoming TCP connections and commands
    void update();

    // Send a large IMU data block to the connected client
    void sendData(const String& data);

    // Current impact threshold value as known by the server
    float getImpactThreshold() const { return impactThreshold; }

    // Returns true if a new threshold was set via TCP command and clears the flag
    bool hasNewThreshold();

private:
    WiFiServer server;
    WiFiClient client;

    float impactThreshold = 35.0f;

    // Flag indicating that impactThreshold has been changed via TCP
    bool thresholdUpdated = false;

    void handleNewClient();
    void handleClientData();
    void handleCommand(const String& cmd);
};