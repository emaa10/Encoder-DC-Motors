#ifndef MAIN_H
#define MAIN_H

#include "./globals.h"
#include "./pathplanning.h"
#include "./structs.h"

/* some constants */

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


extern float x;
extern float y;
extern float theta;
extern long int lastEncLeft;
extern long int lastEncRight;
extern long int leftEncoderChange;
extern long int rightEncoderChange;

extern long int encoderLeft;
extern long int encoderRight;

extern float currentPwmLeft;
extern float currentPwmRight;

extern int counter;
extern long int oldEncoderLeft;
extern long int oldEncoderRight;


template<typename T>
void print(const T& input);

void print(const char* input);

template<typename T>
void println(const T& input);

void println(const char* input);

extern bool pullCordConnected();

void getEncoderData();

// thread to get enc data 24/7
void getEncoderDataThread();

extern long int getEncoderLeft();

extern long int getEncoderRight();

void stopMotor();

void setPwmZero();

// SETS ENCODER DATA TO 0 PERMANENTLY, will be set on the arduino!!!
void setEncoderZero();

/**
 * @description: Return the current angle in degrees.
 * @param input: Input to calculate degrees from radians. Default: theta (current value of bot)
*/
extern float getAngle(float input);

void sendPWMValues(float pwmLeft, float pwmRight);

extern std::vector<Vector> generatePath(int from_x, int from_y, double angle, int to_x, int to_y);

void getNextCoord(int current_x, int current_y, std::vector<Vector> npath);

extern bool isPathFree(int current_x, int current_y, double current_angle, std::vector<Vector> npath);

// here tracking encoder data for odometry and sending it to the megas
void drive(float drivePwmLeft, float drivePwmRight);

// updates the position, based on the last time this func was ran
void updatePosition(float leftEncChange, float rightEncChange);

void turn(float degrees);

/**
* @description: Drive a specific distance in MM while syncing with encoders
* @param distance: distance in mm
*/
void driveDistance(int distance);

void printPath(const vector<Vector>& path);

void setup();

void loop();


#endif