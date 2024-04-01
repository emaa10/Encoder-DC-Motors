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

extern std::vector<Vector> generatePath(int from_x, int from_y, double angle, int to_x, int to_y);

void getNextCoord(int current_x, int current_y, std::vector<Vector> npath);

extern bool isPathFree(int current_x, int current_y, double current_angle, std::vector<Vector> npath);

void turn(float degrees);

/**
* @description: Drive a specific distance in MM while syncing with encoders
* @param distance: distance in mm
*/
void driveDistance(int distance);

void printPath(const vector<Vector>& path);

extern float getCurrentX();

extern float getCurrentY();

void setup();

void loop();


#endif