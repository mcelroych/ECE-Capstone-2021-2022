#pragma once

#include <stdint.h>
#include <math.h>

class PID {
  public:
    PID(double _Sp, double _Dt, double _Kp, double _Ki = 0, double _Kd = 0);
    double controlFunc(uint8_t Pv);
    void reset();

  private:
    double Sp;
    double Dt;
    double Integral;
    double pErr;
    double Kp;
    double Ki;
    double Kd;
};
