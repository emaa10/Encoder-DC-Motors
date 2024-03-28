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
#include "./pathplanning.h"
#include "./structs.h"


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
const float pulsesPerSec = pulsesPerRev; //goal pulses per sec 1680, 1 round per second
const float wheelDistance = 121; //abstand der encoderräder in mm

const int syncInterval = 1; // sync motors with encoders every second
const int syncCounter = syncInterval * 1000 / 20;

const bool yellow = true;
Pathplanner p(-20, 0, 0, 200, yellow);

//odom
float x=0; // curent bot x
float y=0; // current bot y
float theta=0; // current bot theta
long int lastEncLeft=0;   // last enc position left
long int lastEncRight=0;
long int leftEncoderChange;
long int rightEncoderChange;


long int encoderLeft; // enc count left
long int encoderRight;// enc count right

float currentPwmLeft;
float currentPwmRight;

int counter = 0;
long int oldEncoderLeft = 0;
long int oldEncoderRight = 0;

template<typename T>
void print(const T& input) {
    std::cout << input;
}
void print(const char* input) {
    std::cout << input;
}
template<typename T>
void println(const T& input) {
    std::cout << input << std::endl;
}
void println(const char* input) {
    std::cout << input << std::endl;
}


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

/**
 * @description: Return the current angle in degrees.
 * @param input: Input to calculate degrees from radians. Default: theta (current value of bot)
*/
float getAngle(float input = theta) {
    return theta*180/M_PI;
}

void sendPWMValues(float pwmLeft, float pwmRight) {
    currentPwmLeft = pwmLeft;
    currentPwmRight = pwmRight;
    std::string message = std::to_string(pwmLeft) + "," + std::to_string(pwmRight);
    serialPrintf(sPortB, "%s\n", message.c_str());
}

std::vector<Vector> generatePath(int from_x, int from_y, double angle, int to_x, int to_y) {
    return p.getPath({{from_x, from_y}, angle}, {to_x, to_y});
    // example: std::vector<Vector> path = generatePath(1, 1, 1, 1, 1);
}

void getNextCoord(int current_x, int current_y, std::vector<Vector> npath) {
    // here get next coord code
    // return ...
    // USE NPATH INSTEAD OF PATH
}

bool isPathFree(int current_x, int current_y, double current_angle, std::vector<Vector> npath) {
    // return true if free
    return p.freePath({{current_x, current_y}, current_angle}, npath);
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
    // std::cout << "in drive func: " << drivePwmLeft << " " << drivePwmRight << std::endl;
    // here odometry
};

// updates the position, based on the last time this func was ran
void updatePosition(float leftEncChange, float rightEncChange) {
    float leftDistance = leftEncChange / pulsesPerMM;
    float rightDistance = rightEncChange / pulsesPerMM;
    float distance = (leftDistance + rightDistance) / 2;
    float dTheta = (rightDistance - leftDistance) / wheelDistance;
    x += distance * cos(theta + dTheta / 2);
    y += distance * sin(theta + dTheta / 2);
    theta += dTheta;
    std::cout << "X: " << x << " Y: " << y << " Theta: " << getAngle() << std::endl;
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
                // std::cout << "before drive func: " << pulsesPerSec / abs(currentEncoderLeft) * currentPwmLeft << ", " << pulsesPerSec << ", " << abs(currentEncoderLeft) << ", " << currentPwmLeft << std::endl;
                drive(newPwmLeft, newPwmRight);
                startEncLeft = getEncoderLeft();
                startEncRight = getEncoderRight();
                leftEncoderChange = currentEncoderLeft;
                rightEncoderChange = currentEncoderRight;
                updatePosition(leftEncoderChange, rightEncoderChange);
            }
            counter = 0;
        }
        delay(20);
    }
}


void printPath(const vector<Vector>& path) {
    cout << "Path Coordinates:" << endl;
    for (const auto& point : path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
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

void loop() {

}



// ---

int main() {
    setup();
    while(true) {loop();}
}
