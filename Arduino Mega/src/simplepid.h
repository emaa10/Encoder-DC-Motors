#ifndef PIDCLASS
#define PIDCLASS

#include <Arduino.h>

// Class
class SimplePID {
private:
  float kp, kd, ki, umax; // Parameters
  float eprev, eintegral; // Storage

public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(100), eprev(0.0), eintegral(0.0) {}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir) {
    // error
    int e = target - value;

    // derivative
    float dedt = (e - eprev) / (deltaT);

    // integral
    eintegral = eintegral + e * deltaT;

    // control signal
    float u = kp * e + kd * dedt + ki * eintegral;

    // motor power
    pwr = (int)fabs(u);

    // motor direction
    dir = u > 0 ? 1 : u < 0 ? -1 : 0;
    // store previous error
    eprev = e;
  }
};

#endif
