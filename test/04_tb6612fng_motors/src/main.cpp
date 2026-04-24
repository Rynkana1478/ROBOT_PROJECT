// ============================================
// Test 04: TB6612FNG Motor Driver
// Wire: AIN1=GPIO4, AIN2=GPIO5, PWMA=GPIO6
//        BIN1=GPIO7, BIN2=GPIO15, PWMB=GPIO16
//        STBY=GPIO17, VM=7.4V, VCC=3.3V, GND
// Tests: each motor direction, speed ramp, brake
// ============================================

#include <Arduino.h>

#define AIN1  6
#define AIN2  5
#define PWMA  4
#define BIN1  15
#define BIN2  16
#define PWMB  17
#define STBY  7

// LEDC channels
#define CH_L  0
#define CH_R  1

void stopAll() {
    ledcWrite(CH_L, 0);
    ledcWrite(CH_R, 0);
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

void leftFwd(int speed) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    ledcWrite(CH_L, speed);
}

void leftBwd(int speed) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    ledcWrite(CH_L, speed);
}

void rightFwd(int speed) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    ledcWrite(CH_R, speed);
}

void rightBwd(int speed) {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    ledcWrite(CH_R, speed);
}

void brakeAll() {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
    ledcWrite(CH_L, 1023);
    ledcWrite(CH_R, 1023);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n##############################################");
    Serial.println("#  Test 04: TB6612FNG Motor Driver            #");
    Serial.println("#  Wire: AIN1=4, AIN2=5, PWMA=6              #");
    Serial.println("#        BIN1=7, BIN2=15, PWMB=16            #");
    Serial.println("#        STBY=17, VM=7.4V, VCC=3.3V          #");
    Serial.println("##############################################\n");

    // Pin setup
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);

    ledcSetup(CH_L, 5000, 10);  // 5kHz, 10-bit
    ledcAttachPin(PWMA, CH_L);
    ledcSetup(CH_R, 5000, 10);
    ledcAttachPin(PWMB, CH_R);

    // Enable driver
    digitalWrite(STBY, HIGH);
    stopAll();

    Serial.println("  TB6612FNG STBY=HIGH (active)\n");

    // === Test 1: Individual motors ===
    Serial.println("=== Test 1: Individual Motors (1.5 sec each) ===\n");

    Serial.println("  LEFT wheels FORWARD...");
    leftFwd(500); delay(1500); stopAll(); delay(500);

    Serial.println("  LEFT wheels BACKWARD...");
    leftBwd(500); delay(1500); stopAll(); delay(500);

    Serial.println("  RIGHT wheels FORWARD...");
    rightFwd(500); delay(1500); stopAll(); delay(500);

    Serial.println("  RIGHT wheels BACKWARD...");
    rightBwd(500); delay(1500); stopAll(); delay(500);

    Serial.println("  CHECKLIST:");
    Serial.println("  [ ] Left wheels moved forward then backward?");
    Serial.println("  [ ] Right wheels moved forward then backward?");
    Serial.println("  [ ] All 4 wheels spun (2 per side)?");
    Serial.println("  If swapped: swap AIN/BIN wires at TB6612\n");

    // === Test 2: Combined ===
    Serial.println("=== Test 2: Combined Movements ===\n");

    Serial.println("  FORWARD (both)...");
    leftFwd(500); rightFwd(500); delay(2000); stopAll(); delay(500);

    Serial.println("  BACKWARD (both)...");
    leftBwd(500); rightBwd(500); delay(2000); stopAll(); delay(500);

    Serial.println("  SPIN LEFT...");
    leftBwd(500); rightFwd(500); delay(1000); stopAll(); delay(500);

    Serial.println("  SPIN RIGHT...");
    leftFwd(500); rightBwd(500); delay(1000); stopAll(); delay(500);

    Serial.println("  [ ] Robot drove straight forward?");
    Serial.println("  [ ] Robot drove straight backward?");
    Serial.println("  [ ] Robot spun left then right?\n");

    // === Test 3: Speed ramp ===
    Serial.println("=== Test 3: Speed Ramp ===\n");
    Serial.println("  Ramping 0 → max → 0...");

    for (int s = 0; s <= 1023; s += 100) {
        leftFwd(s); rightFwd(s);
        Serial.printf("    PWM: %4d\n", s);
        delay(250);
    }
    for (int s = 1023; s >= 0; s -= 100) {
        leftFwd(s); rightFwd(s);
        Serial.printf("    PWM: %4d\n", s);
        delay(250);
    }
    stopAll();
    Serial.println("  Smooth acceleration? If jerky, check connections.\n");

    // === Test 4: Brake ===
    Serial.println("=== Test 4: Brake vs Coast ===\n");

    Serial.println("  Full speed → COAST (stop)...");
    leftFwd(800); rightFwd(800); delay(1000);
    stopAll();  // coast
    Serial.println("  (wheels should spin down slowly)");
    delay(2000);

    Serial.println("  Full speed → BRAKE...");
    leftFwd(800); rightFwd(800); delay(1000);
    brakeAll();  // active brake
    Serial.println("  (wheels should stop immediately)");
    delay(1000);
    stopAll();

    // === Test 5: Standby ===
    Serial.println("\n=== Test 5: Standby ===\n");
    Serial.println("  Running motors, then STBY=LOW...");
    leftFwd(500); rightFwd(500); delay(1000);
    digitalWrite(STBY, LOW);
    Serial.println("  STBY=LOW: motors should stop (driver sleeping)");
    delay(2000);
    digitalWrite(STBY, HIGH);
    Serial.println("  STBY=HIGH: driver awake again");
    stopAll();

    Serial.println("\n##############################################");
    Serial.println("#  Motor test complete!                       #");
    Serial.println("#  Type 'f','b','l','r','s' in Serial Monitor#");
    Serial.println("#  for manual control                        #");
    Serial.println("##############################################\n");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case 'f': leftFwd(500);  rightFwd(500);  Serial.println("FORWARD");  break;
            case 'b': leftBwd(500);  rightBwd(500);  Serial.println("BACKWARD"); break;
            case 'l': leftBwd(500);  rightFwd(500);  Serial.println("SPIN LEFT"); break;
            case 'r': leftFwd(500);  rightBwd(500);  Serial.println("SPIN RIGHT"); break;
            case 's': stopAll();     Serial.println("STOP"); break;
            case 'x': brakeAll();    Serial.println("BRAKE"); break;
        }
    }
}
