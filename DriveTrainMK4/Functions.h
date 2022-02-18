#pragma once

#include "Config.h"

// Initializes PWM on Timer1
void initPWM() {

  // Set all bits in TCCRnX registers to 0
  TCCR1A &= ~0xFF;
  TCCR1B &= ~0xDF;

  // Set the COMnX bits in the TCCRnA registers
  TCCR1A |= 0xA0;

  // Set the WGM bits in the TCCRnA registers
  TCCR1A |= 0x01;

  // Set the CS bits in TCCRnB for a prescaler of 1
  TCCR1B |= 0x01;

  // Set the last WGM bits in TCCRnB for fast PWM, 8-bit
  TCCR1B |= 0x08;
}

//
void getDistance() {
  
  if ((PINK & 0x01) == 0x01) 
    IRvalue++;
  else 
    IRvalue = 0;

  if(IRvalue == 3)
    turnCond = true;
  else  
    turnCond = false;
}

//
void getLine() {
  lineValue = ~PINF;
}

//
void trackLine() {
  getDistance();
  getLine();
  int diff = Pid.controlFunc(lineValue);

  if (diff > 0) {
    lMotor.initSpeed(baseSpeed + diff);
    rMotor.initSpeed(baseSpeed - diff);
  }
  else if (diff < 0) {
    rMotor.initSpeed(baseSpeed + diff);
    lMotor.initSpeed(baseSpeed - diff);
  }
}

//
void reverse() {
  getLine();
  lMotor.initSpeed(baseSpeed - 4);
  rMotor.initSpeed(baseSpeed);
}

//
void turnAround() {
  rMotor.changeDir();

  getLine();

  while ((lineValue & 0x80) != 0x80)
    getLine();
  while (lineValue > 0x00)
    getLine();
  while ((lineValue & 0x10) != 0x10)
    getLine();

  rMotor.changeDir();
}

//
void turnLeft() {
  lMotor.brake();
  rMotor.initSpeed(baseSpeed);

  while ((lineValue & 0xF0) > 0x00)
    getLine();
  while ((lineValue & 0x08) != 0x08)
    getLine();
}

//
void turnRight() {
  rMotor.brake();
  lMotor.initSpeed(baseSpeed);

  while ((lineValue & 0x0F) > 0x00)
    getLine();
  while ((lineValue & 0x10) != 0x10)
    getLine();
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
    case 0: // start State

      if (lineValue == 0xFF)
        inStart = false;

      if (inStart == false)
        if ((lineValue & 0x01) != 0x01) {
          state = 1;
          lastState = 0;
        }

      break;

    case 1: // trackLine State

      if ((lineValue & 0x0F) == 0x0F) {
        state = 2;
        lastState = 1;
      }

      break;

    case 2: // turnRight State

      state = 3;
      lastState = 2;

      break;

    case 3: // down State

      if (turnCond == true) {
        state = 4;
        lastState = 3;
      }

      break;

    case 4: // turnAround State

      if (lastState == 3)
        state = 5;

      else
        state = 1;

      lastState = 4;

      break;

    case 5: // back State

      if ((lineValue & 0xF0) == 0xF0) {
        state = 6;
        lastState = 5;
      }

      break;

    case 6: // turnLeft State

      state = 7;
      lastState = 6;

      break;

    case 7: // end State

      if (lineValue == 0x00) {
        PORTA ^= 0x03;
        state = 8;
        lastState = 7;
      }

      break;

    case 8: // reverse State

      if (lineValue == 0xFF) {
        PORTA ^= 0x03;
        state = 4;
        lastState = 8;
      }

      break;

    case 9: // brake State
      brake();
      break;

    default:
      state = 1;
      break;
  }
}
