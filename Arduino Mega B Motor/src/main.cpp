#include <Arduino.h>
// Deklaration der Float-Variable zur Speicherung des empfangenen Werts
float receivedValue;

void setup() {
  // Beginnen der seriellen Kommunikation mit einer Baudrate von 9600
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
  // Überprüfen, ob Daten auf der seriellen Schnittstelle verfügbar sind
  if (Serial.available() > 0) {
    // Lesen der empfangenen Daten und Speichern in der Float-Variablen
    receivedValue = Serial.parseFloat();
    // if(receivedValue >= 5) {
    //   digitalWrite(13, HIGH);
    //   delay(200);
    //   digitalWrite(13, LOW);
    // }
    
    // Ausgabe des empfangenen Werts zur Bestätigung
    Serial.print("Received value: ");
    Serial.println(receivedValue);
  }
}
