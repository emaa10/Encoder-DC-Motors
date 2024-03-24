#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>

const std::string serialMegaA = "/dev/ttyACM0";
const std::string serialMegaB = "/dev/ttyACM0";
int serial_port = serialOpen(serialMegaB.c_str(), 9600);

void getEncoderData() {
    //
}

void sendPWMValues(float pwmLeft, float pwmRight) {
    std::string message = std::to_string(pwmLeft) + "," + std::to_string(pwmRight);
    serialPrintf(serial_port, "%s\n", message.c_str());
}

void setup() {
    // initialize wiringpi
    if (wiringPiSetup() == -1) {
        std::cerr << "Fehler beim Initialisieren von WiringPi." << std::endl;
    }
    if (serial_port < 0) {
        std::cerr << "Port error on Mega A Encoder, Port " << serialMegaB << std::endl;
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
