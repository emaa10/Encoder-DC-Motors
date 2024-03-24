#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
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
    long int encLeft, encRight;
    char comma;
    while (true) {
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(100)) {
            if (std::getline(serial, line)) {
                std::istringstream iss(line);
                if (iss >> encLeft >> comma >> encRight) {
                    std::cout << "Encoder Left: " << encLeft << ", Encoder Right: " << encRight << std::endl;
                }
            }
        }
        usleep(10000); // kurze Pause, um die CPU nicht zu belasten
    }

    return 0;
}

