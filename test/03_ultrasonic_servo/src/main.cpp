// ============================================
// Test 03: HC-SR04 Ultrasonic + SG90 Servo x
// Wire: TRIG=GPIO18, ECHO=GPIO8
//        Servo=GPIO9, VCC=5V, GND
// Tests: distance reading, servo sweep, full scan
// ============================================

#include <Arduino.h>
#include <ESP32Servo.h>

#define US_TRIG       18
#define US_ECHO       3
#define SERVO_PIN 9

Servo servo;

float readDistance() {
    digitalWrite(US_TRIG, LOW);  delayMicroseconds(2);
    digitalWrite(US_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(US_TRIG, LOW);
    unsigned long dur = pulseIn(US_ECHO, HIGH, 30000);
    if (dur == 0) return -1;
    return dur / 58.0;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n##############################################");
    Serial.println("#  Test 03: HC-SR04 + SG90 Servo             #");
    Serial.println("#  Wire: TRIG=GPIO18, ECHO=GPIO8             #");
    Serial.println("#        Servo=GPIO9                         #");
    Serial.println("#        VCC=5V (both), GND                  #");
    Serial.println("##############################################\n");

    pinMode(US_TRIG, OUTPUT);
    pinMode(US_ECHO, INPUT);

    // Servo init (ESP32 style)
    servo.setPeriodHertz(50);
    servo.attach(SERVO_PIN, 500, 2400);
    servo.write(90);
    delay(500);

    // === Ultrasonic test ===
    Serial.println("=== Ultrasonic Test ===");
    Serial.println("  Place hand ~20cm in front of sensor\n");

    for (int i = 0; i < 5; i++) {
        float d = readDistance();
        if (d < 0) Serial.printf("  [%d] ERROR: No echo!\n", i + 1);
        else       Serial.printf("  [%d] %.1f cm\n", i + 1, d);
        delay(200);
    }

    // === Servo test ===
    Serial.println("\n=== Servo Test ===");

    Serial.println("  Moving to CENTER (90)...");
    servo.write(90);
    delay(700);
    float center = readDistance();
    Serial.printf("  Center distance: %.1f cm\n", center);

    Serial.println("  Moving to LEFT (160)...");
    servo.write(160);
    delay(700);
    float left = readDistance();
    Serial.printf("  Left distance: %.1f cm\n", left);

    Serial.println("  Moving to RIGHT (20)...");
    servo.write(20);
    delay(700);
    float right = readDistance();
    Serial.printf("  Right distance: %.1f cm\n", right);

    servo.write(90);
    delay(300);

    // Check results
    Serial.println("\n=== Results ===");
    if (center < 0 && left < 0 && right < 0) {
        Serial.println("  Ultrasonic: FAIL - no readings at all");
        Serial.println("  Check: TRIG→GPIO18, ECHO→GPIO8, VCC→5V");
    } else {
        Serial.println("  Ultrasonic: OK");
    }

    if (abs(left - center) > 5 || abs(right - center) > 5) {
        Serial.println("  Servo: OK (distances change at different angles)");
    } else {
        Serial.println("  Servo: might not be moving (all readings similar)");
        Serial.println("  Check: servo signal→GPIO9, servo VCC→5V");
    }

    Serial.println("\n##############################################");
    Serial.println("#  Continuous sweep mode                     #");
    Serial.println("##############################################\n");
}

void loop() {
    // Continuous sweep: L → C → R → C, print distances
    int angles[] = {160, 90, 20, 90};
    const char* labels[] = {"LEFT ", "CENTER", "RIGHT", "CENTER"};

    for (int i = 0; i < 4; i++) {
        servo.write(angles[i]);
        delay(300);
        float d = readDistance();
        Serial.printf("[%s] %3d deg → %6.1f cm\n", labels[i], angles[i], d);
    }
    Serial.println();
    delay(500);
}
