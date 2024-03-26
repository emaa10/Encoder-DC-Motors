#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include "wiringPi.h"
#include "wiringSerial.h"
#include <math.h>
#include <thread>


const std::string serialMegaA = "/dev/ttyACM0"; // enc
const std::string serialMegaB = "/dev/ttyACM1"; // dc
std::ifstream serial(serialMegaA.c_str());
int sPortB = serialOpen(serialMegaB.c_str(), 9600);

// encoder stuff
const float pulsesPerEncRev = 1200;
const float encWheelDiameterCM = 5;
const float motorWheelDiameterCM = 7;
const float encWheelScope = encWheelDiameterCM * M_PI; 
const float motorWheelScope = motorWheelDiameterCM * M_PI; // distance travelled per rev
const float pulsesPerRev = pulsesPerEncRev * (motorWheelScope / encWheelScope);
const float pulsesPerMM = pulsesPerRev / motorWheelScope / 10;
const float pulsesPerCM = pulsesPerRev / motorWheelScope;

long int encoderLeft; // enc count left
long int encoderRight;// enc count right

float currentPwmLeft;
float currentPwmRight;

int counter = 0;

void getEncoderData() {
    std::string line;
    char comma;
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(100)) {
        if (std::getline(serial, line)) {
            std::istringstream iss(line);
            if (iss >> encoderLeft >> comma >> encoderRight) {
                // std::cout << "Encoder Left: " << encoderLeft << ", Encoder Right: " << encoderRight << std::endl;
            }
        }
    }
}

// thread to get enc data 24/7
void getEncoderDataThread() {
    while(true) {
        getEncoderData();
    }
}

long int getEncoderLeft() {
    // getEncoderData();
    return encoderLeft;
}

long int getEncoderRight() {
    // getEncoderData();
    return encoderRight;
}

void sendPWMValues(float pwmLeft, float pwmRight) {
    currentPwmLeft = pwmLeft;
    currentPwmRight = pwmRight;
    std::string message = std::to_string(pwmLeft) + "," + std::to_string(pwmRight);
    serialPrintf(sPortB, "%s\n", message.c_str());
}

void driveDistanceMM(int distance) { // distance in mm
    float pulses = distance * pulsesPerMM;
    float distanceDrivenMM = 0;
    while(distanceDrivenMM <= distance) {
        // drive...
        sendPWMValues(100, 100); // start with 100 speed on both motors
    }
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

    std::thread t(getEncoderDataThread);
    t.detach();
}

/**
* @description: Drive a specific distance in MM while syncing with encoders
* @param distance: distance in mm
*/
void driveDistance(int distance) {
    const int startEncLeft = getEncoderLeft();
    const int startEncRight = getEncoderRight();
    // need these 2 lines to recalculate current enc values
    long int currentEncoderLeft = getEncoderLeft() - startEncLeft;
    long int currentEncoderRight = getEncoderRight() - startEncRight;

    
}

void loop() {
    // for loop through tactics
    // for each coord: generate path, for each coord of path: calc angle and distance, turn, drive
    std::cout << getEncoderLeft() << "," << getEncoderRight() << std::endl;
    delay(20);
    counter++;
    std::cout << counter << std::endl;
    if(counter >= 500) { // nach 10 sek
        const int startEncLeft = getEncoderLeft();
        const int startEncRight = getEncoderRight();
        while(true) {
            long int currentEncoderLeft = getEncoderLeft() - startEncLeft;
            long int currentEncoderRight = getEncoderRight() - startEncRight;
            std::cout << getEncoderLeft() << "," << getEncoderRight() << std::endl; 
            std::cout << currentEncoderLeft << ";" << currentEncoderRight << std::endl;
            delay(20);
        }
    }
}



// ---

int main() {
    setup();
    while(true) {loop();}
}
