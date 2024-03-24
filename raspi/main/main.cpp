#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>

const std::string serialMegaA = "/dev/ttyACM0";

void getEncoderData() {
    //
}

void setup() {
    // initialize wiringpi
    if (wiringPiSetup() == -1) {
        std::cerr << "Fehler beim Initialisieren von WiringPi." << std::endl;
    }
    int serial_port = serialOpen(serialMegaA.c_str(), 9600);
    if (serial_port < 0) {
        std::cerr << "Port error on Mega A Encoder, Port " << serialMegaA << std::endl;
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
