#include "./main.h"
using namespace std;

const std::string serialMega = "/dev/ttyACM0"; // enc and dc
std::ifstream serial(serialMega.c_str());
int sPort = serialOpen(serialMega.c_str(), 115200);
Pathplanner p(-20, 0, 0, 150, yellow);

//odom
float x=0; // curent bot x
float y=0; // current bot y
float theta=0; // current bot theta

char serial_data;


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

void turn(float degrees) {
    std::string message = "t," + std::to_string(degrees);
    serialPrintf(sPort, "%s\n", message.c_str());
}

void driveDistance(int distance) {
    std::string message = "d," + std::to_string(distance);
    serialPrintf(sPort, "%s\n", message.c_str());
}

void driveTo(int to_x, int to_y) {
    float deltaX = to_x - x;
    float deltaY = to_y - y;
    float distance = sqrt((deltaX*deltaX) + (deltaY*deltaY));
    float angle = theta*180/M_PI;

    angle = atan2(deltaY,deltaX) * 180/PI - angle;

    turn(angle);
    delay(2000); // SDFKJJKLSDFJKLFSDLJKSDFÖDLJKSDFJKLÖSDKLJFFSJKLJKDFSSFJKLSDKJLFDJSKLÖJKSDFÖJKLFDSJLSDJKLSDF
    driveDistance(distance);
    delay(2000);
}

float getCurrentX() {
    //
}

float getCurrentY() {
    //
}

void getDataThread() {
    while(true) {
        getData();
        sendData();
    }
}

void getData() {
    std::string line;
    while (std::getline(serial, line)) { // Lese eine Zeile vom seriellen Port
        std::stringstream ss(line);
        char type;
        double value;
        ss >> type; // Lese den Typ (x, y oder theta)
        ss.ignore(); // Ignoriere das Komma
        ss >> value; // Lese den Wert

        if (type == 'x') {
            x = value;
        } else if (type == 'y') {
            y = value;
        } else if (type == 't') {
            theta = value;
        }
    }

}

void sendData() { // send pullcord
    int state = pullCordConnected();
    std::string message = "p," + std::to_string(state);
    serialPrintf(sPort, "%s\n", message.c_str()); 
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
    std::thread t(getDataThread); // get current pos from arduino
    t.detach();

    stopMotor();

    // while(pullCordConnected()) {delay(20);}
    delay(500);
    
    // println(int(pullCordConnected()));
    std::vector<Vector> path = p.getPath({{225, 225}, 0}, group1);
    for (Vector target : path) {
        std::cout << "target x: " << target.x << std::endl;
        std::cout << "target y: " << target.y << std::endl;
        driveTo(target.x, target.y);
    }
    std::cout << "path size: " << path.size() << std::endl;
}

void loop() {
    delay(5);
}





int main() {
    setup();
    while(1) {loop();}
}