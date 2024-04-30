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
  void evalu(int value1, int value2, int target1, int target2, float deltaT,
             long &pwr, int &dir) {
    // error
    int e1 = target1 - value1;
    int e2 = target2 - value2;

    // derivative
    float dedt = (fabs(value2) - fabs(value1)) / (deltaT);
    dedt = e1 < 0 ? -dedt : dedt;
    if (kp * e1 < 255 || kp * e2 < 255)
      dedt = 0;

    // integral
    eintegral = eintegral + e1 * deltaT;

    // control signal
    float u = kp * e1 + kd * dedt + ki * eintegral;

    // motor power
    pwr = (long)fabs(u);

    // motor direction
    dir = u > 0 ? 1 : u < 0 ? -1 : 0;
    // store previous error
    eprev = e1;
  }
};

#endif
