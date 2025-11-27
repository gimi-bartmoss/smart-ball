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
    if (client && client.connected()) {
            client.print(data);
    } else {
        WiFiClient newClient = server.available();    
        if (newClient) {
            if (client) {
                client.stop();
            }
            client = newClient;
            client.print(data);
            return;
            }
            if (client) {
                client.stop();
                client = WiFiClient();
            }
    }
}
