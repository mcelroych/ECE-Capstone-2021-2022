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

   while ((rADCvalue > 0x00) || (lADCvalue > 0x00))
    readADC();
  while (rADCvalue != 0x01)
    readADC();

  rMotor.changeDir();
}

//
void turnLeft() {
  lMotor.brake();
  rMotor.initSpeed(baseSpeed);
  while (rADCvalue < 7)
    readADC();
}

//
void turnRight() {
  rMotor.brake();
  lMotor.initSpeed(baseSpeed);
  while (lADCvalue < 7)
    readADC();
}

//
void brake() {
  lMotor.brake();
  rMotor.brake();
}

//
void nextState() {
  switch (state)
  {
  case 0: // trackLine State
    if ((lADCvalue == 0x0F) && (rADCvalue != 0x0F))
      state = 1;
    else if ((rADCvalue == 0x0F) && (lADCvalue != 0x0F))
      state = 2;
    else if ((lADCvalue | rADCvalue) == 0x00)
      state = 5;
    break;
  
  case 1: // turnLeft State
    prevState = 1;
    state = 0;
    break;

  case 2: // turnRight State
    prevState = 2;
    state = 0;
    break;

  case 3: // reverse State
    /* code */
    break;

  case 4: // turnAround State
    prevState = 4;
    state = 0;
    break;

  case 5: // brake State
    /* code */
    break;

  default:
    prevState = 0;
    state = 0;
    break;
  }
}