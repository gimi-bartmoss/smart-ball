#include "WiFiServerModule.h"

WiFiServerModule::WiFiServerModule(uint16_t port)
    : server(port) {}

void WiFiServerModule::begin(const char* ssid, const char* password) {
    // Start Wi-Fi in Access Point mode
    WiFi.softAP(ssid, password);
    IPAddress ip = WiFi.softAPIP();
    Serial.print("Access Point started, IP address: ");
    Serial.println(ip);

    server.begin();
    server.setNoDelay(true);
    Serial.println("TCP server started.");
}

void WiFiServerModule::handleNewClient() {
    WiFiClient newClient = server.available();
    if (newClient) {
        // Optionally drop previous client and accept the new one
        client.stop();
        client = newClient;
        Serial.println("Client connected.");
        client.println("SmartBall TCP server ready.");
    }
}

void WiFiServerModule::handleClientData() {
    if (!client || !client.connected()) return;
    while (client.available()) {
        // Read one line per command (newline-terminated)
        String cmd = client.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) {
            handleCommand(cmd);
        }
    }
}

void WiFiServerModule::handleCommand(const String& cmd) {
    // Command: SET THRESH=<value>
    if (cmd.startsWith("SET THRESH=")) {
        String valStr = cmd.substring(11);  // After "SET THRESH"
        valStr.trim();
        float val = valStr.toFloat();
        if (val > 0.0f && val < 300.0f) {  // Accelerometer range of MPU6050 is ±16g per axis
            impactThreshold = val;
            thresholdUpdated = true;  // Mark that we have a new threshold
            client.println("OK THRESH=" + String(impactThreshold, 2));
            Serial.print("Impact threshold updated via TCP: ");
            Serial.println(impactThreshold);
        } else {
            client.println("ERR invalid threshold");
        }
        return;
    }

    // Command: GET THRESH
    if (cmd == "GET THRESH") {
        client.println("THRESH=" + String(impactThreshold, 2));
        return;
    }

    // Command: PING
    if (cmd == "PING") {
        client.println("PONG");
        return;
    }

    // Unknown command
    client.println("ERR unknown command");
}

bool WiFiServerModule::hasNewThreshold() {
    if (thresholdUpdated) {
        thresholdUpdated = false;
        return true;
    }
    return false;
}

void WiFiServerModule::update() {
    if (!client || !client.connected()) {
        handleNewClient();
    } else {
        handleClientData();
    }
}

void WiFiServerModule::sendData(const String& data) {
    if (client && client.connected()) {
        // Send the entire IMU data block to the client
        client.print(data);
        client.flush();
        Serial.print("Sent IMU data block, size = ");
        Serial.println(data.length());
    } else {
        Serial.println("No client connected, cannot send data.");
    }
}