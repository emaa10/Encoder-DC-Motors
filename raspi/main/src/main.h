#ifndef MAIN_H
#define MAIN_H

#include "./globals.h"
#include "./pathplanning.h"
#include "./structs.h"

/* some constants */

// encoder stuff
const int pullCord = 8;

const bool yellow = true;


extern float x;
extern float y;
extern float theta;

template<typename T>
void print(const T& input);

void print(const char* input);

template<typename T>
void println(const T& input);

void println(const char* input);

extern bool pullCordConnected();

void stopMotor();

void turn(float degrees);

/**
* @description: Drive a specific distance in MM while syncing with encoders
* @param distance: distance in mm
*/
void driveDistance(int distance);

extern float getCurrentX();

extern float getCurrentY();

void getDataThread();

void getData();

void sendData();

void setup();

void loop();


#endif