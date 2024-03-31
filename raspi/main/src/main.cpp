#include "./main.h"
using namespace std;

const std::string serialMegaA = "/dev/ttyACM0"; // enc
const std::string serialMegaB = "/dev/ttyACM1"; // dc
std::ifstream serial(serialMegaA.c_str());
int sPortB = serialOpen(serialMegaB.c_str(), 9600);
int sPortA = serialOpen(serialMegaA.c_str(), 115200);


Pathplanner p(-20, 0, 0, 200, yellow);

//odom
float x=0; // curent bot x
float y=0; // current bot y
float theta=0; // current bot theta
long int lastEncLeft=0;   // last enc position left
long int lastEncRight=0;


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

void signalHandler(int signal) {
    // run code on ctrl c
    stopMotor();
    exit(signal);
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

void getEncoderDataThread() {
    while(true) {
        getEncoderData();
        delay(5);
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

void stopMotor() {setPwmZero();}

void setPwmZero() {
    sendPWMValues(0, 0);
}

void setEncoderZero() {
    serialPrintf(sPortA, "%s\n", std::string("0").c_str());
}

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

void updatePositionThread() {
    float leftEnc1 = getEncoderLeft();
    float rightEnc1 = getEncoderRight();
    delay(1000);
    float leftEnc2 = getEncoderLeft();
    float rightEnc2 = getEncoderRight();
    updatePosition(leftEnc2 - leftEnc1, rightEnc2 - rightEnc1);
}

void turn(float degrees) {
    float distance = turnValue * degrees;
    float pulsesLeft = -1.0f * (distance * pulsesPerMM); // links rückwärts... sollte passen ig
    float pulsesRight = distance * pulsesPerMM;

    int startEncLeft = getEncoderLeft();
    int startEncRight = getEncoderRight();
    int lastEncLeft = getEncoderLeft();
    int lastEncRight = getEncoderRight();
    long int currentEncoderLeft = 0;
    long int currentEncoderRight = 0;
    long int currentPIDleft = 0;
    long int currentPIDright = 0;

    print(-pwmSpeed);
    print(" ");
    println(pwmSpeed);
    drive(-pwmSpeed, pwmSpeed); // links rückwrts
    counter = 0;
    while(currentEncoderLeft > pulsesLeft || currentEncoderRight < pulsesRight) { // solange wir noch nicht da sind
        currentEncoderLeft = getEncoderLeft() - startEncLeft;
        currentEncoderRight = getEncoderRight() - startEncRight;
        currentPIDleft = getEncoderLeft() - lastEncLeft;
        currentPIDright = getEncoderRight() - lastEncRight;
        // check ob gegner auf stregge brauchen wir hier nicht
        counter++;
        if(counter >= syncCounterTurn) {
            if(currentEncoderLeft != 0 && currentEncoderRight != 0) { // fehler vermeiden
                float newPwmLeft = pulsesPerSec/(1000/syncCounterTurn) / abs(currentPIDleft) * currentPwmLeft; // geteilt durch 5 wegen syncCounterTurn
                float newPwmRight = pulsesPerSec/(1000/syncCounterTurn) / abs(currentPIDright) * currentPwmRight;
                drive(newPwmLeft, newPwmRight);
                lastEncLeft = getEncoderLeft();
                lastEncRight = getEncoderRight();
                // odom calc start
                updatePosition(currentPIDleft, currentPIDright);
                // odom calc end
            }
        }
    }
    drive(0, 0);
    updatePosition(currentPIDleft, currentPIDright);
    // odom manual start -> not recommended
    // theta += degrees;
    // theta = fmod((theta + 360.0), 360.0);
    // odom manual end
}

void driveDistance(int distance) {
    // RUN BEFORE DRIVING!!
    int startEncLeft = getEncoderLeft();
    int startEncRight = getEncoderRight();
    int lastEncLeft = getEncoderLeft();
    int lastEncRight = getEncoderRight();
    float distancePulses = distance * pulsesPerMM;

    // need these 2 lines to recalculate current enc values. 
    long int currentEncoderLeft = 0; // for driving
    long int currentEncoderRight = 0;
    long int currentPIDleft = 0;
    long int currentPIDright = 0;

    drive(pwmSpeed, pwmSpeed); // start with 100 pwm
    counter = 0;
    while(distancePulses > (currentEncoderLeft + currentEncoderRight)/2 + pulsesPerSec/12) { // might need correction
        print("durchschnitt enc: ");
        println((currentEncoderLeft + currentEncoderRight)/2);
        // solange wir noch nicht da sind
        currentEncoderLeft = getEncoderLeft() - startEncLeft;
        currentEncoderRight = getEncoderRight() - startEncRight;
        currentPIDleft = getEncoderLeft() - lastEncLeft;
        currentPIDright = getEncoderRight() - lastEncRight;
        // hier check ob gegner auf strecke
        counter++;
        if(counter >= syncCounter) { //wenn bestimmte zeit vergangen
            // neue pwm werte basierend auf encoder daten berechnen und positionsbestimmung
            if(currentEncoderLeft != 0 && currentEncoderRight != 0) {
                float newPwmLeft = pulsesPerSec / abs(currentPIDleft) * currentPwmLeft;
                float newPwmRight = pulsesPerSec / abs(currentPIDright) * currentPwmRight;
                // std::cout << "before drive func: " << pulsesPerSec / abs(currentEncoderLeft) * currentPwmLeft << ", " << pulsesPerSec << ", " << abs(currentEncoderLeft) << ", " << currentPwmLeft << std::endl;
                drive(newPwmLeft, newPwmRight);
                print("Newpwmleft: ");
                print(newPwmLeft);
                print(", Newpwmright: ");
                println(newPwmRight);
                lastEncLeft = getEncoderLeft();
                lastEncRight = getEncoderRight();
                // updatePosition(currentPIDleft, currentPIDright);
            }
            counter = 0;
        }
        delay(5);
    }
    drive(0, 0); // stop motor
    // updatePosition(currentPIDleft, currentPIDright);
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
    std::signal(SIGINT, signalHandler); // control c stops motors
    pinMode(8, INPUT);

    std::thread t(getEncoderDataThread);
    t.detach();
    std::thread t2(updatePositionThread);
    t2.detach();

    setPwmZero();
    while(pullCordConnected()) {delay(20);}
    delay(500);
    setEncoderZero(); // just to be sure
    delay(1000);

    println("START");
    
    driveDistance(500);
    // turn(90);
    println("SIND DA");
}


void loop() {
    print("Encoder left: ");
    print(getEncoderLeft());
    print(", Encoder right: ");
    println(getEncoderRight());
    delay(5);
}



// ---

int main() {
    setup();
    while(true) {loop();}
}