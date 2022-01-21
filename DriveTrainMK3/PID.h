#pragma once

#include <stdint.h>
#include <math.h>

class PID {
  public: 
    PID(double _Sp, double _Dt, double _Kp = 0, double _Ki = 0, double _Kd = 0);
    double controlFunc(double Pv);
    
  private:
    double Sp;
    double Dt;
    double Integral;
    double pErr;
    double Kp;
    double Ki;
    double Kd;
};
