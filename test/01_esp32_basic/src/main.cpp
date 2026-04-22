// ============================================
// Test 01: ESP32-S3 Basic + WiFi
// Just the board, nothing else connected.
// Verifies: Serial, WiFi, GPIO, free heap
// ============================================

#include <Arduino.h>
#include <WiFi.h>

// Change these to your hotspot
#define WIFI_SSID     "YourSSID"
#define WIFI_PASSWORD "YourPassword"

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n##############################################");
    Serial.println("#  Test 01: ESP32-S3 Basic                   #");
    Serial.println("##############################################\n");

    // Board info
    Serial.printf("  Chip: %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("  Cores: %d\n", ESP.getChipCores());
    Serial.printf("  CPU freq: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("  Free heap: %d bytes (%d KB)\n", ESP.getFreeHeap(), ESP.getFreeHeap() / 1024);
    Serial.printf("  Flash: %d MB\n", ESP.getFlashChipSize() / 1024 / 1024);
    Serial.println();

    // GPIO test - toggle a pin
    Serial.println("  GPIO test: toggling all project pins LOW...");
    int pins[] = {1, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18};
    for (int i = 0; i < 15; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
    }
    Serial.println("  GPIO: OK\n");

    // WiFi test
    Serial.printf("  Connecting to WiFi '%s'...", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n  WiFi: CONNECTED\n");
        Serial.printf("  IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());
    } else {
        Serial.println("\n  WiFi: FAILED - check SSID/password");
    }

    Serial.println("\n##############################################");
    Serial.println("#  RESULT: If you see this, ESP32-S3 works!  #");
    Serial.println("##############################################\n");
}

void loop() {
    static unsigned long last = 0;
    if (millis() - last >= 2000) {
        last = millis();
        Serial.printf("Uptime: %lus | Heap: %d KB | WiFi: %s\n",
            millis() / 1000,
            ESP.getFreeHeap() / 1024,
            WiFi.status() == WL_CONNECTED ? "OK" : "DISCONNECTED");
    }
}
