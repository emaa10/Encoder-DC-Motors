#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <chrono>
#include "wiringPi.h"
#include "wiringSerial.h"


const std::string serialMegaA = "/dev/ttyACM0"; // needs change
const std::string serialMegaB = "/dev/ttyACM0";
std::ifstream serial(serialMegaA.c_str());
int sPortB = serialOpen(serialMegaB.c_str(), 9600);

void getEncoderData() {
    //
}

void sendPWMValues(float pwmLeft, float pwmRight) {
    std::string message = std::to_string(pwmLeft) + "," + std::to_string(pwmRight);
    serialPrintf(sPortB, "%s\n", message.c_str());
}

void setup() {
    // initialize stream
    if (!serial.is_open()) {
        std::cerr << "Port error on Mega A Encoder, Port" << serialMegaA << std::endl;
    }

    // initialize wiringpi
    if (wiringPiSetup() == -1) {
        std::cerr << "Fehler beim Initialisieren von WiringPi." << std::endl;
    }
    if (sPortB < 0) {
        std::cerr << "Port error on Mega B Motor, Port " << serialMegaB << std::endl;
    }
}

void loop() {
    //
}



// ---

int main() {
    setup();
    while(true) {loop();}
}
