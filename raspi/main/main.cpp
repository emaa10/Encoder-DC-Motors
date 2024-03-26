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
const float pwmSpeed = 100; //default pwm speed
const unsigned int pulsesPerSec = pulsesPerRev; //goal pulses per sec 1680, 1 round per second
const float wheelDistance = 121; //abstand der encoderräder in mm

const int syncInterval = 1; // sync motors with encoders every second
const int syncCounter = syncInterval * 1000 / 20;

//odom
int x=0; // curent bot x
int y=0; // current bot y
int theta=0; // current bot theta
long int lastEncLeft=0;   // last enc position left
long int lastEncRight=0;
long int leftEncoderChange;
long int rightEncoderChange;


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

// here tracking encoder data for odometry and sending it to the megas
void drive(float drivePwmLeft, float drivePwmRight) {
    if(drivePwmLeft > 150) {
        drivePwmLeft = 150;
    }
    if(drivePwmRight > 150) {
        drivePwmRight = 150;
    }
    sendPWMValues(drivePwmLeft, drivePwmRight);
    std::cout << "drive pwm left: " << drivePwmLeft << " " << drivePwmRight << std::endl;
    // here odometry
};

// updates the position, based on the last time this func was ran
void updatePosition() {
    // long int leftEncoderChange = getEncoderLeft() - lastEncLeft;
    // long int rightEncoderChange = getEncoderRight() - lastEncRight;

    float leftDistance = (leftEncoderChange / pulsesPerEncRev) * (M_PI * encWheelDiameterCM);
    float rightDistance = (rightEncoderChange / pulsesPerEncRev) * (M_PI * encWheelDiameterCM);
    float deltaDistance = (leftDistance + rightDistance) / 2;
    float deltaTheta = (rightDistance - leftDistance) / wheelDistance / 10;

    x += deltaDistance * cos(theta + deltaTheta / 2);
    y += deltaDistance * sin(theta + deltaTheta / 2);
    theta += deltaTheta;

    theta = fmod(theta, 2 * M_PI);
    if (theta < 0)
        theta += 2 * M_PI;


    // lastEncLeft = getEncoderLeft();
    // lastEncRight = getEncoderRight();
}

/**
* @description: Drive a specific distance in MM while syncing with encoders
* @param distance: distance in mm
*/
void driveDistance(int distance) {
    // RUN BEFORE DRIVING!!
    int startEncLeft = getEncoderLeft();
    int startEncRight = getEncoderRight();
    float distancePulses = distance * pulsesPerMM;

    // need these 2 lines to recalculate current enc values. 
    long int currentEncoderLeft = 0;
    long int currentEncoderRight = 0;

    drive(pwmSpeed, pwmSpeed); // start with 100 pwm
    counter = 0;
    while(distancePulses > (currentEncoderLeft + currentEncoderRight)/2) {
        // solange wir noch nicht da sind
        currentEncoderLeft = getEncoderLeft() - startEncLeft;
        currentEncoderRight = getEncoderRight() - startEncRight;
        // hier check ob gegner auf strecke
        counter++;
        if(counter >= syncCounter) { //wenn bestimmte zeit vergangen
            // neue pwm werte basierend auf encoder daten berechnen
            if(currentEncoderLeft != 0 && currentEncoderRight != 0) {
                float newPwmLeft = pulsesPerSec / abs(currentEncoderLeft) * currentPwmLeft;
                float newPwmRight = pulsesPerSec / abs(currentEncoderRight) * currentPwmRight;
                std::cout << "currentecnoderleft: " << currentEncoderLeft << std::endl;
                std::cout << "currentpwm: " << currentPwmLeft << std::endl;
                drive(newPwmLeft, newPwmRight);
                startEncLeft = getEncoderLeft();
                startEncRight = getEncoderRight();
            }
            counter = 0;
        }
        delay(20);
    }
    leftEncoderChange = currentEncoderLeft;
    rightEncoderChange = currentEncoderRight;
    updatePosition();
}

void loop() {
    // updatePosition();
    // std::cout << x << ", " << y << ", " << theta << std::endl;
    // delay(1000);
    driveDistance(5000);
    delay(200);
}



// ---

int main() {
    setup();
    while(true) {loop();}
}
