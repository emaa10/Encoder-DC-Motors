#include <iostream>
#include <wiringPi.h>

// Definieren Sie die Pins, an denen die Encoder angeschlossen sind
const int encoderPin1A = 21; // Beispiel-Pin für Encoder 1, Phase A
const int encoderPin1B = 22; // Beispiel-Pin für Encoder 1, Phase B
const int encoderPin2A = 28; // Beispiel-Pin für Encoder 2, Phase A
const int encoderPin2B = 29; // Beispiel-Pin für Encoder 2, Phase B

// Variablen zur Verfolgung der Encoder-Positionen
volatile int encoderPos1 = 0;
volatile int encoderPos2 = 0;

void ai0()
{
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(encoderPin1A) == LOW)
  {
    encoderPos1++;
    std::cout << "hoch 1" << std::endl;
  }
  else
  {
    encoderPos1--;
    std::cout << "runter 1" << std::endl;
  }
}

// LEFT_2
void ai1()
{
  if (digitalRead(encoderPin1B) == LOW)
  {
    encoderPos1--;
    std::cout << "hoch 2" << std::endl;
  }
  else
  {
    encoderPos1++;
    std::cout << "runter 2" << std::endl;
  }
}

// RIGHT_1
void bi0()
{
  if (digitalRead(encoderPin2A) == LOW)
  {
    encoderPos2--;
    std::cout << "runter 3" << std::endl;
  }
  else
  {
    encoderPos2++;
    std::cout << "hoch 3" << std::endl;
  }
}

// RIGHT_2
void bi1()
{
  if (digitalRead(encoderPin2B) == LOW)
  {
    encoderPos2++;
    std::cout << "hoch 4" << std::endl;
  }
  else
  {
    encoderPos2--;
    std::cout << "runter 4" << std::endl;
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
    wiringPiISR(encoderPin1A, INT_EDGE_RISING, &ai0);
    wiringPiISR(encoderPin1B, INT_EDGE_RISING, &ai1);
    wiringPiISR(encoderPin2A, INT_EDGE_RISING, &bi0);
    wiringPiISR(encoderPin2B, INT_EDGE_RISING, &bi1);

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