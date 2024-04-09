#include "./main.h"

using namespace std;

const std::string serialMega = "/dev/ttyACM0"; // enc and dc
int sPort = serialOpen(serialMega.c_str(), 115200);
const char* command1 = "screen -XS platformio quit";
const char* command = "screen -d -m platformio /home/bot/.local/bin/pio device monitor -p /dev/ttyACM0 -b 115200";
std::ifstream serial(serialMega.c_str());
Pathplanner p(-20, 0, 10, 200, yellow);

//odom
float x=0; // curent bot x
float y=0; // current bot y
float theta=0; // current bot theta

bool driving = false;


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
    std::string message = "s";
    serialPrintf(sPort, "%s\n", message.c_str());
}

void interruptDriving() {
    std::string message = "s";
    std::cout << "interrupted driving" << std::endl;
    serialPrintf(sPort, "%s\n", message.c_str()); 
}

void turn(float degrees) {
    while(degrees >= 360) {
        degrees -= 360;
    }
    while(degrees >= 180) {
        degrees -= 180;
    }
    while(degrees <= -180) {
        degrees += 360;
    }
    std::string message = "t," + std::to_string(degrees);
    std::cout << "degrees: " << degrees << std::endl;
    serialPrintf(sPort, "%s\n", message.c_str());
    while (driving == false) {
        delay(5);
    }
    while(driving == true) {
        delay(5);
        // hier lidar check
        // wenn enemy detected, interruptDriving() ausführen
    }
}

void driveUntilSwitch() {
    std::string message = "w";
    std::cout << "drive until switch triggered" << std::endl;
    serialPrintf(sPort, "%s\n", message.c_str()); 
    while(driving == false) {
        delay(5);
    }
    while(driving == true) {
        delay(5);
        // hier lidar check
    }
}

void driveDistance(int distance) {
    std::string message = "d," + std::to_string(distance);
    serialPrintf(sPort, "%s\n", message.c_str());
    while (driving == false) {
        delay(5);
    }
    while(driving == true) {
        delay(5);
        // hier lidar check
        // wenn enemy detected, interruptDriving() ausführen
    }
}

void driveTo(int to_x, int to_y) {
    std::cout << "To X: " << to_x << ", To Y: " << to_y << std::endl;
    std::cout << "X: " << x << ", Y: " << y << std::endl;
    float deltaX = to_x - x;
    float deltaY = to_y - y;
    float distance = sqrt((deltaX*deltaX) + (deltaY*deltaY));
    float angle = theta*180/M_PI;

    std::cout << "Angle davor: " << angle << std::endl;
    angle = atan2(deltaY,deltaX) * 180/PI - angle;
    // std::cout << "Delta X: " << deltaX << ", Delta Y: " << deltaY << std::endl;
    std::cout << "Angle: " << angle << std::endl;
    std::cout << "Distance: " << distance << std::endl;
    std::cout << "X: " << x << endl;
    cout << "Y: " << y << endl;

    turn(angle);
    // delay(2000);
    // cout << " TURN DONE " << endl;
    driveDistance(distance);
}

void turnTo(int degree) {
    float angle = theta*180/M_PI;
    float toTurn = degree - angle;
    turn(toTurn);
}

void getDataThread() {
    while(true) {
        getData();
        // sendData();
    }
}


void getData() {
    std::string line;
    while (std::getline(serial, line)) { // Lese eine Zeile vom seriellen Port
        std::stringstream ss(line);

        char tempchar;
        ss >> tempchar;
        driving = (tempchar == 'd');
        
        ss >> tempchar;
        ss >> x; // Lese den Wert
        ss >> tempchar;
        ss >> y;
        ss >> tempchar;
        ss >> theta;

        // std::cout << "X: " << x << std::endl;
        // std::cout << "Y: " << y << std::endl;
        // std::cout << "Angle: " << theta*180/M_PI << std::endl;
        std::cout << "Line: " << line << std::endl;
        line = "";
    }

}


void setup() {
    // initialize stream
    // if (!serial.is_open()) {
    //     std::cerr << "Port error on Mega A Encoder, Port" << serialMega << std::endl;
    // }

    // initialize wiringpi
    if (wiringPiSetup() == -1) {
        std::cerr << "Fehler beim Initialisieren von WiringPi." << std::endl;
    }

    if (sPort < 0) {
        std::cerr << "Fehler beim Öffnen des seriellen Ports." << std::endl;
    }

    std::signal(SIGINT, signalHandler); // control c stops motors
    pinMode(8, INPUT);
    std::thread t(getDataThread); // get current pos from arduino
    t.detach();

    system(command1);
    delay(100);
    system(command);

    delay(2000);

    // driveTo(0, 500);

    driveTo(500, 500);
    delay(5000);

    driveTo(200, 500);
    delay(5000);

    driveTo(200, 200);
    delay(5000);

    driveTo(500, 0);
    delay(5000);

    driveTo(500,500);
    delay(5000);
}

void loop() {
    // println(x);
    delay(5);
}





int main() {
    setup();
    while(1) {loop();}
}