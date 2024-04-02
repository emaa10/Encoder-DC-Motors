#include "./main.h"
using namespace std;

const std::string serialMega = "/dev/ttyACM0"; // enc and dc
std::ifstream serial(serialMega.c_str());
int sPort = serialOpen(serialMega.c_str(), 115200);
Pathplanner p(-20, 10, 50, 150, yellow);

//odom
float x=0; // curent bot x
float y=0; // current bot y
float theta=0; // current bot theta


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

void signalHandler(int signal) {
    // run code on ctrl c
    stopMotor();
    exit(signal);
}

bool pullCordConnected() {
    return (digitalRead(pullCord) == 0);
}

void stopMotor() {setPwmZero();}


void drive(float drivePwmLeft, float drivePwmRight) {
    if(drivePwmLeft > 150) {drivePwmLeft = 150;}
    if(drivePwmRight > 150) {drivePwmRight = 150;}
    if(drivePwmLeft < -150) {drivePwmLeft = -150;}
    if(drivePwmRight < -150) {drivePwmRight = -150;}
    println("SENT VALUES");
    sendPWMValues(drivePwmLeft, drivePwmRight);
    // std::cout << "in drive func: " << drivePwmLeft << " " << drivePwmRight << std::endl;
    // here odometry
};


void setup() {
    //
}

void loop() {
    //
}





int main() {
    setup();
    while(1) {loop();}
}