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

// Initializes interrupt pins
void initISR() {
  PCICR &= ~0x07;
  PCMSK0 &= ~0xFF;

  PCICR |= 0x01;
  PCMSK0 |= 0x0F;
}

//
void getDistance() {

  if ((PINK & 0x03) == 0x03)
    IRvalue++;
  else
    IRvalue = 0;

  if (IRvalue == 3)
    turnCond = true;
  else
    turnCond = false;
}

//
void getLine() {
  lineValue = ~PINF;
}

//
void trackLine(uint8_t speed = baseSpeed) {
  getDistance();
  getLine();
  int diff = Pid.controlFunc(lineValue);

  if (diff > 0) {
    lMotor.initSpeed(speed + diff);
    rMotor.initSpeed(speed - diff);
  }
  else if (diff < 0) {
    rMotor.initSpeed(speed + diff);
    lMotor.initSpeed(speed - diff);
  }
}

//
void reverse(uint8_t speed = baseSpeed) {
  getLine();
  lMotor.initSpeed(speed - 4);
  rMotor.initSpeed(speed);
}

//
void turnAround() {
  rMotor.changeDir();

  delay(100);
  getLine();

  while ((lineValue & 0x80) != 0x80)
    getLine();
  while (lineValue > 0x00)
    getLine();
  while (lineValue == 0x00)
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
  switch (state) {

    case 0: // stall State
      if ((PINL & 0x01) == 0x01) {
        state = 1;
        lastState = 0;
      }

      break;

    case 1: // start State

      if (lineValue == 0xFF)
        inStart = false;

      if (inStart == false)
        if ((lineValue & 0x01) != 0x01) {
          state = 2;
          lastState = 1;
        }

      break;

    case 2: // trackLine State

      if ((lineValue & 0x0F) == 0x0F) {
        state = 3;
        lastState = 2;
      }

      break;

    case 3: // turnRight State

      state = 4;
      lastState = 3;

      break;

    case 4: // down State

      if (turnCond == true) {
        state = 5;
        lastState = 4;
      }

      break;

    case 5: // turnAround State

      if (lastState == 4)
        state = 6;

      else
        state = 2;

      lastState = 5;

      break;

    case 6: // back State

      if ((lineValue & 0xF0) == 0xF0) {
        state = 7;
        lastState = 6;
      }

      break;

    case 7: // turnLeft State

      state = 8;
      lastState = 7;

      break;

    case 8: // end State

      if (lineValue == 0x00) {
        PORTA ^= 0x03;
        state = 9;
        lastState = 8;
      }

      break;

    case 9: // reverse State

      if (lineValue == 0xFF) {
        PORTA ^= 0x03;
        state = 5;
        lastState = 9;
      }

      break;

    case 10: // brake State

      break;

    case 11: // allignFront State

      break;

    case 12: // allignBack State

      break;

    default:
      state = 0;
      break;
  }
  if (state < 10)
    returnState = state;
}
