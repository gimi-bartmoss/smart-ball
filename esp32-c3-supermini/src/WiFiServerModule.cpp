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

void WiFiServerModule::handleClient(String data) {
    static WiFiClient client = server.available();

    if (!client || !client.connected()) {
        WiFiClient newClient = server.available();
        if (newClient) {
            client = newClient;
            Serial.println("Client connected!");
        }
        return;
    }

    client.print(data);

    if (!client.connected()) {
        client.stop();
        Serial.println("Client disconnected.");
    }
}
