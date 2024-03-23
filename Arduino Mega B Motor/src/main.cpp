#include <Arduino.h>
const int LED_PIN = 13;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    if (Serial.available() > 0) {
        String message = Serial.readStringUntil('\n');
        if (message.indexOf("hallo") != -1) {
            digitalWrite(LED_PIN, HIGH);
            delay(1000); // LED für 1 Sekunde einschalten
            digitalWrite(LED_PIN, LOW);
        }
    }
}
