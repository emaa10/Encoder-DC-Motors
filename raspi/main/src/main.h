#ifndef MAIN_H
#define MAIN_H

#include "./globals.h"
#include "./structs.h"

/* some constants */

// encoder stuff
const int teamSwitch = 9;
const int pullCord = 7;

const bool yellow = true;


extern float x;
extern float y;
extern float theta;

extern bool driving;

template<typename T>
void print(const T& input);

void print(const char* input);

template<typename T>
void println(const T& input);

void println(const char* input);

extern bool pullCordConnected();

void stopMotor();

void interruptDriving();

void turn(float degrees);

void driveUntilSwitch();

/**
* @description: Drive a specific distance in MM while syncing with encoders
* @param distance: distance in mm
*/
void driveDistance(int distance);

void driveTo(int to_x, int to_y);

void getDataThread();

void getData();

void setup();

void loop();


#endif
