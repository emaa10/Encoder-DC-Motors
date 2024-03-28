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
using namespace std;

const std::string serialMegaA = "/dev/ttyACM0"; // enc
const std::string serialMegaB = "/dev/ttyACM1"; // dc
std::ifstream serial(serialMegaA.c_str());
int sPortB = serialOpen(serialMegaB.c_str(), 9600);
int sPortA = serialOpen(serialMegaA.c_str(), 9600);

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
const float wheelDistance = 121; //abstand der encoderräder in mm, muss vllt geändert werden
const float wheelDistanceBig = 184; // in mm, muss vllt geändert werden
const float turnValue = wheelDistanceBig * M_PI / 360; // abstand beider räder um 1° zu fahren

const int pullCord = 8;

const int syncInterval = 1; // sync motors with encoders every second
const int syncCounter = syncInterval * 1000 / 20;
const int syncCounterTurn = 200; // check alle 200ms, wenn ich das änder auch das /5 beim turn ändern!
const int startDelay = 5000; // 5 secods after raspi start -> need pullcord

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

bool pullCordConnected() {
    return (digitalRead(pullCord) == 0);
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

// SETS ENCODER DATA TO 0 PERMANENTLY, will be set on the arduino!!!
void setEncoderZero() {
    serialPrintf(sPortA, "%s\n", std::string("0").c_str());
}

/**
 * @description: Return the current angle in degrees.
 * @param input: Input to calculate degrees from radians. Default: theta (current value of bot)
*/
float getAngle(float input = theta) {
    float result = theta*180/M_PI;
    // result = fmod((result + 360.0), 360.0);
    // now in updatepos with theta
    return result;
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
    if(drivePwmLeft > 150) {drivePwmLeft = 150;}
    if(drivePwmRight > 150) {drivePwmRight = 150;}
    if(drivePwmLeft < -150) {drivePwmLeft = -150;}
    if(drivePwmRight < -150) {drivePwmRight = -150;}
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
    theta = fmod((theta + 2 * M_PI), (2 * M_PI)); // test in radian
    std::cout << "X: " << x << " Y: " << y << " Theta: " << getAngle() << std::endl;
}

void turn(float degrees) {
    float distance = turnValue * degrees;
    float pulsesLeft = -1.0f * (distance * pulsesPerMM); // links rückwärts... sollte passen ig
    float pulsesRight = distance * pulsesPerMM;

    int startEncLeft = getEncoderLeft();
    int startEncRight = getEncoderRight();
    long int currentEncoderLeft = 0;
    long int currentEncoderRight = 0;

    drive(-pwmSpeed, pwmSpeed); // links rückwrts
    counter = 0;
    while(currentEncoderLeft > pulsesLeft || currentEncoderRight < pulsesRight) { // solange wir noch nicht da sind
        currentEncoderLeft = getEncoderLeft() - startEncLeft;
        currentEncoderRight = getEncoderRight() - startEncRight;
        // check ob gegner auf stregge brauchen wir hier nicht
        counter++;
        if(counter >= syncCounterTurn) {
            if(currentEncoderLeft != 0 && currentEncoderRight != 0) { // fehler vermeiden
                float newPwmLeft = pulsesPerSec/(1000/syncCounterTurn) / abs(currentEncoderLeft) * currentPwmLeft; // geteilt durch 5 wegen syncCounterTurn
                float newPwmRight = pulsesPerSec/(1000/syncCounterTurn) / abs(currentEncoderRight) * currentPwmRight;
                drive(newPwmLeft, newPwmRight);
                startEncLeft = getEncoderLeft();
                startEncRight = getEncoderRight();
                // odom calc start
                leftEncoderChange = currentEncoderLeft;
                rightEncoderChange = currentEncoderRight;
                updatePosition(leftEncoderChange, rightEncoderChange);
                // odom calc end
            }
        }
    }
    // odom manual start -> not recommended
    // theta += degrees;
    // theta = fmod((theta + 360.0), 360.0);
    // odom manual end
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
    while(distancePulses > (currentEncoderLeft + currentEncoderRight)/2) { // might need correction
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
    pinMode(8, INPUT);

    std::thread t(getEncoderDataThread);
    t.detach();

    delay(startDelay); // start delay - needs to be changed
    setEncoderZero(); // just to be sure
}


void loop() {
    println(pullCordConnected()?"Ja":"Nein");
    println(digitalRead(8));
    // print(" - ");
    // println(digitalRead(9));
    delay(50);
}



// ---

int main() {
    setup();
    while(true) {loop();}
}
