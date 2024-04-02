#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>

int main() {
    if (wiringPiSetup() == -1) {
        std::cerr << "Fehler beim Initialisieren von WiringPi." << std::endl;
        return 1;
    }

    int serial_port = serialOpen("/dev/ttyACM0", 115200);
    if (serial_port < 0) {
        std::cerr << "Fehler beim Öffnen des seriellen Ports." << std::endl;
        return 1;
    }

    std::string message;
    while (true) {
        std::cout << "Nachricht eingeben (exit zum Beenden): ";
        std::getline(std::cin, message);
        
        if (message == "exit")
            break;

        serialPrintf(serial_port, "%s\n", message.c_str());
    }

    serialClose(serial_port); 
    return 0;
}

