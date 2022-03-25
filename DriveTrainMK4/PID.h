#pragma once

#include <stdint.h>
#include <math.h>

class PID {
  public:
    PID(uint8_t _Sp, double _Dt, double _Kp, double _Ki = 0, double _Kd = 0);
    double controlFunc(uint8_t Pv);
    void reset();
    void changeGain(double _Kp, double _Ki = 0, double _Kd = 0);

  private:
    uint8_t Sp;
    double Dt;
    double Integral;
    double pErr;
    double Kp;
    double Ki;
    double Kd;
};
