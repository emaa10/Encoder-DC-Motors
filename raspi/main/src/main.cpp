#include <iostream>
#include <wiringPi.h>

// Definieren Sie die Pins, an denen die Encoder angeschlossen sind
const int encoderPin1A = 21; // Beispiel-Pin für Encoder 1, Phase A
const int encoderPin1B = 22; // Beispiel-Pin für Encoder 1, Phase B
const int encoderPin2A = 4; // Beispiel-Pin für Encoder 2, Phase A
const int encoderPin2B = 5; // Beispiel-Pin für Encoder 2, Phase B

// Variablen zur Verfolgung der Encoder-Positionen
volatile int encoderPos1 = 0;
volatile int encoderPos2 = 0;

// Interrupt-Handler für Encoder 1, Phase A
void handleEncoder1A() {
    if (digitalRead(encoderPin1B) == LOW) {
        encoderPos1++;
    } else {
        encoderPos1--;
    }
}

// Interrupt-Handler für Encoder 2, Phase A
void handleEncoder2A() {
    if (digitalRead(encoderPin2B) == LOW) {
        encoderPos2++;
    } else {
        encoderPos2--;
    }
}

int main() {
    // Initialisierung von WiringPi
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize WiringPi." << std::endl;
        return 1;
    }

    // Konfigurieren Sie die Encoder-Pins als Eingänge
    pinMode(encoderPin1A, INPUT);
    pinMode(encoderPin1B, INPUT);
    pinMode(encoderPin2A, INPUT);
    pinMode(encoderPin2B, INPUT);

    // Registrieren Sie die Interrupt-Handler für die Encoder-Pins
    wiringPiISR(encoderPin1A, INT_EDGE_BOTH, &handleEncoder1A);
    wiringPiISR(encoderPin2A, INT_EDGE_BOTH, &handleEncoder2A);

    // Hauptprogrammschleife
    while (true) {
        // Drucken Sie die aktuellen Positionen der Encoder
        // std::cout << digitalRead(encoderPin1A) << std::endl;
        // std::cout << digitalRead(encoderPin1B) << std::endl;
        // std::cout << digitalRead(encoderPin2A) << std::endl;
        // std::cout << digitalRead(encoderPin2B) << std::endl;
        std::cout << "Encoder 1 Position: " << encoderPos1 << std::endl;
        std::cout << "Encoder 2 Position: " << encoderPos2 << std::endl;

        delay(5);
    }

    return 0;
}