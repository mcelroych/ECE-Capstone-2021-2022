# include "PID.h"

// Constructors
PID::PID(double _Sp, double _Dt, double _Kp, double _Ki, double _Kd):
  Sp(_Sp), Dt(_Dt), Integral(0), pErr(0), Kp(_Kp), Ki(_Ki), Kd(_Kd)
{
  // intentionally blank
}

double PID::controlFunc(double Pv) {

  // Calculate Error
  double Err = Sp - Pv;

  // Calculate Proportional term
  double Pp = Kp * Err;

  // Calculate Integral term
  Integral += Err * Dt;
  double Ip = Ki * Integral;

  // Calculate Derivative term
  double Derivative = (Err - pErr) / Dt;
  double Dp = Kd * Derivative;

  // Sum the individual terms
  double Ut = Pp + Ip + Dp;

  // Save current Error
  pErr = Err;

  return Ut;
}
