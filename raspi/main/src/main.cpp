#include "./main.h"
#include "lidar.h"

using namespace std;

const std::string serialMega = "/dev/ttyACM0"; // enc and dc
const std::string serialESP = "/dev/ttyUSB0"; // sima and fahne
int sPort = serialOpen(serialMega.c_str(), 115200);
// int sPortE = 0;
int sPortE = serialOpen(serialESP.c_str(), 115200);
const char *command1 = "screen -XS platformio quit";
const char *command = "screen -d -m platformio /home/bot/.local/bin/pio device "
                      "monitor -p /dev/ttyACM0 -b 115200";
std::ifstream serial(serialMega.c_str());
std::ifstream serialE(serialESP.c_str());
LIDAR ldr;

// odom
float x = 225;     // curent bot x
float y = 225;     // current bot y
float theta = 0; // current bot theta
float tox = 0;   // for COA
float toy = 0;
bool gegi = true;
bool teamYellow = true;
bool gegiTriggered = false;

bool driving = false;

template <typename T> void print(const T &input) { std::cout << input; }
void print(const char *input) { std::cout << input; }
template <typename T> void println(const T &input) { std::cout << input << std::endl; }
void println(const char *input) { std::cout << input << std::endl; }

void signalHandler(int signal) {
  stopMotor();
  exit(signal);
}

bool pullCordConnected() { return (digitalRead(pullCord) == 0); }

void startSIMAs(bool teamBlue = !teamYellow) {
  std::string message = "s" + std::to_string(teamBlue);
  serialPrintf(sPortE, "%s\n", message.c_str());
}

// 0: aus, 1 und 2 an
void setSolar(int mode = 1) {
  std::string message = "c" + std::to_string(mode);
  serialPrintf(sPortE, "%s\n", message.c_str());
}

void setDisplay(int number) { //
  std::string message = "x" + std::to_string(number);
  serialPrintf(sPortE, "%s\n", message.c_str());
}

// 0 oben, 1, 2, 3 ganz ausgeklappt
void setGripperAngle(int mode = 3) {
  std::string message = "u" + std::to_string(mode);
  serialPrintf(sPortE, "%s\n", message.c_str());
}

// 1 unten, 2 mitte, 3 oben
void setGripperHeight(int mode = 1) {
  std::string message = "b" + std::to_string(mode);
  serialPrintf(sPortE, "%s\n", message.c_str());
}

void resetBelt() {
  std::string message = "r";
  serialPrintf(sPortE, "%s\n", message.c_str());
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
void continueDriving() {
  std::string message = "c";
  std::cout << "interrupted driving" << std::endl;
  serialPrintf(sPort, "%s\n", message.c_str());
}

void changeSpeed(int newSpeed) { serialPrintf(sPort, "g,%d", newSpeed); }

void turn(float degrees) {
  degrees = teamYellow ? degrees : -degrees;
  while (degrees >= 360) {
    degrees -= 360;
  }
  while (degrees > 180) {
    degrees -= 180;
  }
  while (degrees <= -180) {
    degrees += 360;
  }
  std::string message = "t," + std::to_string(degrees);
  std::cout << "degrees: " << degrees << std::endl;
  serialPrintf(sPort, "%s\n", message.c_str());
  while (driving == false) {
    delay(5);
  }
  while (driving == true) {
    delay(5);
    // hier lidar check
    if (gegi) {
      if (!ldr.freeTurn(
              {{1500, 1000}, theta * 180 / M_PI})) { // wenn vorne blockiert
        interruptDriving();
        gegiTriggered = true;
      } else {
        continueDriving();
      }
    }
  }
}

void driveUntilSwitch(bool dir = false) {
  std::string message = "w," + std::to_string(dir);
  // std::cout << "drive until switch triggered" << std::endl;
  serialPrintf(sPort, "%s\n", message.c_str());
  while (driving == false) {
    delay(5);
  }
  while (driving == true) {
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
  while (driving == true) {
    delay(5);
    // hier lidar check
    if (gegi) {
      if (distance > 0 && !ldr.freeBack({{1500, 1000}, theta * 180 / M_PI})) { // wenn vorne blockiert
        interruptDriving();
        gegiTriggered = true;
      } else if (distance < 0 && !ldr.freeFront({{1500, 1000}, theta * 180 / M_PI})) {
        interruptDriving();
        gegiTriggered = true;
      } else {
        continueDriving();
      }
    }
  }

}

void driveTo(int to_x, int to_y) {
  tox = to_x;
  toy = to_y;
  std::cout << "To X: " << to_x << ", To Y: " << to_y << std::endl;
  std::cout << "X: " << x << ", Y: " << y << std::endl;
  float deltaX = to_x - x;
  float deltaY = to_y - y;
  float distance = sqrt((deltaX * deltaX) + (deltaY * deltaY));
  float angle = theta * 180 / M_PI;

  std::cout << "Angle davor: " << angle << std::endl;
  angle = atan2(deltaY, deltaX) * 180 / M_PI - angle;
  std::cout << "Angle: " << angle << std::endl;
  std::cout << "Distance: " << distance << std::endl;
  std::cout << "X: " << x << endl;
  cout << "Y: " << y << endl;

  if (!(angle <= 3 && angle >= -3))
    turn(angle);
  // if(gegiTriggered) {
  //     gegiTriggered = false;
  //     float angle = theta*180/M_PI;
  //     float deltaX = to_x - x;
  //     float deltaY = to_y - y;
  //     float distance = sqrt((deltaX*deltaX) + (deltaY*deltaY));
  //     angle = atan2(deltaY,deltaX) * 180/M_PI - angle;
  // }
  if (distance != 0)
    driveDistance(distance);
  // if(gegiTriggered) {
  //     gegiTriggered = false;
  //     float angle = theta*180/M_PI;
  //     float deltaX = to_x - x;
  //     float deltaY = to_y - y;
  //     float distance = sqrt((deltaX*deltaX) + (deltaY*deltaY));
  //     angle = atan2(deltaY,deltaX) * 180/M_PI - angle;
  // }
}

void turnTo(int degree) {
  float angle = theta * 180 / M_PI;
  float toTurn = degree - angle;
  turn(toTurn);
}

void homing(bool teamYellowN) {
  gegi = false;
  driveUntilSwitch(false);
  driveDistance(160);
  turn(-90);
  driveUntilSwitch(false);
  driveDistance(210);
  gegi = true;
}

void timingsThread() {
  delay(90000);
  startSIMAs();
  // drive home
  delay(9000);
  stopMotor();
  delay(50);
  system(command);
  delay(100);
  system(command1);
}

void getDataThread() {
  while (true) {
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
    x = teamYellow ? x : -x;
    ss >> tempchar;
    ss >> y;
    y = teamYellow ? y : -y;
    ss >> tempchar;
    ss >> theta;

    std::cout << "Line: " << line << std::endl;
    line = "";
  }
}

void setup() {
  if (wiringPiSetup() == -1) {
    std::cerr << "Fehler beim Initialisieren von WiringPi." << std::endl;
  }

  if (sPort < 0) {
    std::cerr << "Fehler beim Öffnen des seriellen Ports. (Arduino)" << std::endl;
  }
  if (sPortE < 0) {
    std::cerr << "Fehler beim Öffnen des seriellen Ports. (ESP)" << std::endl;
  }
  std::signal(SIGINT, signalHandler); // control c stops motors
  pinMode(pullCord, INPUT);
  pinMode(teamSwitch, INPUT);
  std::thread t(getDataThread); // get current pos from arduino
  t.detach();

  system(command1);
  delay(100);
  system(command);

  delay(1000);
  resetBelt();
  delay(1000);
  
  // while(true) {driveUntilSwitch(true); delay(500000000);}

  setGripperAngle(0);
  delay(500);
  setGripperHeight(1);
  delay(500);
  // teamYellow = false;
  if(digitalRead(teamSwitch) == 0) {
    teamYellow = true;
  } else {
    teamYellow = false;
  }
  homing(true);
  setDisplay(20);

  while(pullCordConnected()) { delay(5); }
  // teamYellow = (digitalRead(teamSwitch) == 0);
  // while(true) {
  // std::cout << teamYellow << std::endl;
  // delay(50);
  // }
  std::thread u(timingsThread); // check if simas, drive home, etc.
  u.detach();
  
  // start
  gegi = false;
  // delay(10000);
  // startSIMAs();
  driveDistance(500);
  driveDistance(-100);
  turn(90);
  delay(500);
  setGripperHeight(1);
  delay(2000);
  setGripperAngle(3);
  delay(1500);
  driveDistance(900);
  setGripperAngle(2);
  delay(2000);
  setGripperHeight(4);
  delay(2000);
  turn(178);
  driveDistance(900);
  driveUntilSwitch(true);
  setGripperHeight(2);
  delay(2000);
  driveDistance(-400);
  gegi = true;
  setGripperAngle(0);
  delay(2000);
  setGripperHeight(1);
  turn(90);
  driveDistance(650);
  turn(90);
  setGripperAngle(3);
  delay(2000);
  driveDistance(400);
  setGripperAngle(2);
  delay(2000);
  setGripperHeight(4);
  delay(2000);
  driveDistance(-300);
  turn(90);
  driveDistance(1000);
  driveUntilSwitch(true);
  setGripperHeight(2);
  delay(2000);
  setGripperAngle(2);
  delay(1000);
  driveDistance(-500);
  //driveDistance(1400);
  setGripperAngle(0);
  delay(2000);
  setGripperHeight(1);
  turn(175);
  driveDistance(1000);
  while(true) delay(5);
  setGripperHeight(3);
  driveUntilSwitch(true);
  driveDistance(-60);
  // drehding
  turn(90);
  driveDistance(1200);
  setSolar(1);
  turn(5);
  driveDistance(-1200);
  setSolar(0);
  gegi = true;

}

void loop() {
  // std::cout << "pullcord: " + std::to_string(digitalRead(pullCord)) + " sw: " + std::to_string(digitalRead(teamSwitch)) << std::endl;
  // std::cout << "Freefront: " << ldr.freeFront({{500, 500}, 0});
  // std::cout << " Freeback: " << ldr.freeBack({{500, 500}, 0});
  // std::cout << " Freeturn: " << ldr.freeTurn({{500, 500}, 0}) << std::endl;
  // std::cout << "X: " << x << " Y: " << y << " Angle: " << theta*180/M_PI << std::endl;
  delay(5);
  if(pullCordConnected()) { // wenn pullcord nochmal eingesteckt wird, arduino reset
    system(command);
    delay(200);
    system(command1);
    std::exit(0); // brauche nen while loop vom betriebssystem her
  }
}

int main() {
  setup();
  while (1) {
    loop();
  }
}
