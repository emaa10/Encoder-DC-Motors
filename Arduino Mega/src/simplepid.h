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
  void evalu(int value1, int value2, int target, float deltaT, long &pwr,
             int &dir) {
    // error
    int e = target - value1;

    // derivative
    float dedt = (fabs(value2) - fabs(value1)) / (deltaT);
    dedt = e < 0 ? -dedt : dedt;

    // integral
    eintegral = eintegral + e * deltaT;

    // control signal
    float u = kp * e + kd * dedt + ki * eintegral;

    // motor power
    pwr = (long)fabs(u);

    // motor direction
    dir = u > 0 ? 1 : u < 0 ? -1 : 0;
    // store previous error
    eprev = e;
  }
};

#endif
