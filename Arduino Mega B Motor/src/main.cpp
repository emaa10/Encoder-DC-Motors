// nur pwm getten und auf motor packen
#include <Arduino.h>

// left dc motor pins
#define LEFT_DCM_LEFT_PWM 5
#define LEFT_DCM_RIGHT_PWM 4

// right dc motor pins
#define RIGHT_DCM_LEFT_PWM 2
#define RIGHT_DCM_RIGHT_PWM 3

float pwmLeft=0;
float pwmRight=0;

float receivedValue1; // pwm value left
float receivedValue2; // pwm value right

void setup() {
  // Beginnen der seriellen Kommunikation mit einer Baudrate von 9600
  Serial.begin(9600);
  // pinmodes
  pinMode(LEFT_DCM_LEFT_PWM, OUTPUT);
  pinMode(LEFT_DCM_RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DCM_LEFT_PWM, OUTPUT);
  pinMode(RIGHT_DCM_RIGHT_PWM, OUTPUT);
}

void loop() {
  // Überprüfen, ob Daten auf der seriellen Schnittstelle verfügbar sind
  if (Serial.available() > 0) {
    // Lesen der empfangenen Daten bis zum Komma und Speichern in der ersten Float-Variablen
    receivedValue1 = Serial.parseFloat();
    
    // Warten auf das Komma
    while (Serial.read() != ',');
    
    // Lesen der empfangenen Daten nach dem Komma und Speichern in der zweiten Float-Variablen
    receivedValue2 = Serial.parseFloat();
    
    // Warten auf das Ende der aktuellen Zeile, um sicherzustellen, dass keine weiteren Zeichen im Puffer sind
    while (Serial.read() != '\n');
    
    pwmLeft = receivedValue1;
    pwmRight = receivedValue2;
    analogWrite(RIGHT_DCM_LEFT_PWM, pwmRight);
    analogWrite(RIGHT_DCM_RIGHT_PWM, 0);
    analogWrite(LEFT_DCM_LEFT_PWM, pwmLeft);
    analogWrite(LEFT_DCM_RIGHT_PWM, 0);
    Serial.print("Received value 1: ");
    Serial.println(receivedValue1);
    Serial.print("Received value 2: ");
    Serial.println(receivedValue2);
  }
  delay(10);
}