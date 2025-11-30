#include "WiFiServerModule.h"

WiFiServerModule::WiFiServerModule(int port) : server(port) {}

void WiFiServerModule::begin(const char* ssid, const char* password) {
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Access Point started, IP address: ");
    Serial.println(IP);

    server.begin();
    server.setNoDelay(true);
    Serial.println("TCP server started.");
}

void WiFiServerModule::handleClient(const IMUData* data) {
    if (client && client.connected()) {
            client.write((const uint8_t*)data, sizeof(IMUData));
    } else {
        WiFiClient newClient = server.available();    
        if (newClient) {
            if (client) {
                client.stop();
            }
            client = newClient;
            client.write((const uint8_t*)data, sizeof(IMUData));
            return;
            }
            if (client) {
                client.stop();
                client = WiFiClient();
            }
    }
}
