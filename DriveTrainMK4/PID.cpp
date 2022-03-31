# include "PID.h"


// Constructors
PID::PID(uint8_t _Sp, double _Kp, double _Ki, double _Kd):
  Sp(_Sp), Integral(0), pErr(0), Kp(_Kp), Ki(_Ki), Kd(_Kd)
{
  // intentionally blank
}

double PID::controlFunc(uint8_t Pv) {

  // Calculate Error
  double Err = Sp - Pv;

  // Calculate Proportional term
  double Pp = Kp * Err;

  // Calculate Integral term
  Integral += Err;
  double Ip = Ki * Integral;

  // Calculate Derivative term
  double Derivative = Err - pErr;
  double Dp = Kd * Derivative;

  // Sum the individual terms
  double Ut = Pp + Ip + Dp;

  // Save current Error
  pErr = Err;

  return Ut;
}

void PID::dump() {
  Integral = 0;
  pErr = 0;
}

void PID::changeGain(double _Kp = 0, double _Ki = 0, double _Kd = 0) {
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
}
