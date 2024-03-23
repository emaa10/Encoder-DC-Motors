#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <chrono>

int main() {
    std::string serial_port = "/dev/ttyACM0"; // Pfad zum Serial Port des Arduinos

    std::ifstream serial(serial_port.c_str()); // Datei-Stream für den Serial Port

    if (!serial.is_open()) {
        std::cerr << "Fehler beim Öffnen des Serial Ports." << std::endl;
        return 1;
    }

    std::string line;
    while (true) {
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(100)) {
            if (std::getline(serial, line)) {
                std::cout << line << std::endl; // Ausgabe der gelesenen Zeile
            }
        }
        usleep(10000); // kurze Pause, um die CPU nicht zu belasten
    }

    return 0;
}

