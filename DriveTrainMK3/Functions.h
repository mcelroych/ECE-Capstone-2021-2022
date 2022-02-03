#pragma once

#include "Config.h"

void readADC() {
   // Select the first ADC input channel
  adcI = 0;
  ADMUX &= ~0x1F;
  ADMUX |= adcI;

  // Hold until conversions are complete
  while (adcI < 7) {
    // Set the ADSC bit to start conversion
    ADCSRA |= 0x40;
  }
}

//
void trackLine() {
  readADC();
  
  Pv = lADCvalue - rADCvalue;
  diff = Pid.controlFunc(Pv);

  if (diff > 0) {
    lMotor.initSpeed(baseSpeed - diff);
    rMotor.initSpeed(baseSpeed + diff);
  }
  else if (diff < 0) {
    rMotor.initSpeed(baseSpeed - diff);
    lMotor.initSpeed(baseSpeed + diff);
  }
}

//
void reverse() {
  readADC();
  
  Pv = lADCvalue - rADCvalue;
  diff = Pid.controlFunc(Pv);

  if (diff > 0) {
    lMotor.initSpeed(baseSpeed - diff);
    rMotor.initSpeed(baseSpeed + diff);
  }
  else if (diff < 0) {
    rMotor.initSpeed(baseSpeed - diff);
    lMotor.initSpeed(baseSpeed + diff);
  }
}

//
void turnAround() {
  rMotor.changeDir();

  while ((rADCvalue > 0x00) || (lADCvalue > 0x00));
  while (rADCvalue != 0x01);

  rMotor.changeDir();
}

//
void turnLeft() {
  lMotor.brake();
  rMotor.initSpeed(baseSpeed);
  while (rADCvalue < 7);
}

//
void turnRight() {
  rMotor.brake();
  lMotor.initSpeed(baseSpeed);
  while (lADCvalue < 7);
}

//
void brake() {
  lMotor.brake();
  rMotor.brake();
}