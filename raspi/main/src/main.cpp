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

void stopMotor() { // 
}

void turn(float degrees) {
    // 
}

void driveDistance(float distance) {
    std::string message = "d," + std::to_string(distance);
    serialPrintf(sPort, "%s\n", message.c_str());
}

float getCurrentX() {
    //
}

float getCurrentY() {
    //
}

void setup() {
    // initialize stream
    if (!serial.is_open()) {
        std::cerr << "Port error on Mega A Encoder, Port" << serialMega << std::endl;
    }

    // initialize wiringpi
    if (wiringPiSetup() == -1) {
        std::cerr << "Fehler beim Initialisieren von WiringPi." << std::endl;
    }

    std::signal(SIGINT, signalHandler); // control c stops motors
    pinMode(8, INPUT);
    std::thread t(getPosThread); // get current pos from arduino
    t.detach();

    stopMotor();

    while(pullCordConnected()) {delay(20);}
    delay(500);
    
    driveDistance(500);
}

void loop() {
    //
}





int main() {
    setup();
    while(1) {loop();}
}