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
  PCMSK0 |= 0x09;
}

// Checks the compared results from the two distance sensors
// sets the turnCond boolean true if within range of the wall
// range set with a potentiomete on the breakout board
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

// Reads the compared results from the line sensor array
// threshold is set by a potentiometer on the breakout board
void getLine() {

  uint8_t compValue = ~PINF;
  uint8_t lineAverage = 0x00;

  for (int i = 0; i < 3; i++) {
    lineAverage ^= ~PINF;
    delay(10);
  }

  if (compValue == lineAverage)
    lineValue = lineAverage;

  rValue = lineValue & 0x0F;
  lValue = (lineValue >> 4) & 0x0F;
}

// Recieves control signal from the PID controller class
// seeks to drive the system to be centered on the line
// and moving forward
void trackLine() {
  getDistance();
  getLine();

  int diff = Pid.controlFunc(lValue - rValue);

  if (diff < 0) {
    lMotor.initSpeed(baseSpeed + diff);
    rMotor.initSpeed(baseSpeed - diff);
  }
  else if (diff > 0) {
    rMotor.initSpeed(baseSpeed + diff);
    lMotor.initSpeed(baseSpeed - diff);
  }
 
}


// Sets the motor speed to account for drift in the
// reverse state, tracking the line is unreliable due to
// sensor array being on the opposite end of travel
void reverse() {
  getLine();
  if ((lValue == 0x00) && (rValue = 0x00)) {
    lMotor.initSpeed(baseSpeed + 4);
    rMotor.initSpeed(baseSpeed);
  }

  else {
    int diff = Pid.controlFunc(lValue - rValue);

    if (diff > 0) {
      lSpeed += diff;
      rSpeed -= diff;
    }
    else if (diff < 0) {
      rSpeed += diff;
      lSpeed -= diff;
    }
    rMotor.initSpeed(rSpeed);
    lMotor.initSpeed(lSpeed);
  }
}

// function has multiple while loops designed to
// gate the function from exiting until the sensor array
// fully leaves the line and returns, on the return it
// looks for the robot to be centered enough for the PID
// to take over and stabalize
void turnAround() {

  lMotor.initSpeed(baseSpeed);
  rMotor.initSpeed(baseSpeed);

  rMotor.changeDir();

  getLine();
  while (lineValue > 0x00)
    getLine();
  while (rValue < 0x07)
    getLine();
  while ((lValue & 0x01) != 0x01)
    getLine();
}

//
void turnLeft() {
  lMotor.brake();
  rMotor.initSpeed(maxSpeed);

  while (lValue > 0x00)
    getLine();
  while (lValue == 0x00)
    getLine();
  while (lValue > 0x03)
    getLine();
}

//
void turnRight() {
  rMotor.brake();
  lMotor.initSpeed(maxSpeed);

  while (rValue != 0x00)
    getLine();
  while (rValue == 0x00)
    getLine();
  while (lValue < 0x01)
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
      if ((PINL & 0x80) == 0x80) {
        state = 1;
        lastState = 0;
      }

      break;

    case 1: // start State

      if ((lValue == 0x0F) && (rValue = 0x0F))
        inStart = false;

      if (inStart == false)
        if ((rValue & 0x08) != 0x08) {
          state = 2;
          lastState = 1;
        }

      break;

    case 2: // trackLine State

      if (rValue == 0x0F) {
        state = 3;
        lastState = 2;
      }

      break;

    case 3: // turnRight State

      state = 4;
      returnState = 4;
      lastState = 3;

      break;

    case 4: // down State

      if (turnCond == true) {
        state = 12;
        lastState = 4;
      }

      break;

    case 5: // turnAround State

      if (lastState == 4) {

        state = 12;
        rMotor.changeDir();
      }

      else {
        state = 2;
        rMotor.changeDir();
      }

      lastState = 5;

      break;

    case 6: // back State

      if (lValue == 0x0F) {
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
        state = 9;
        lastState = 8;
        Pid.changeGain(2.00);
        PORTA ^= 0x03;
      }

      break;

    case 9: // reverse State

      if (lineValue == 0xFF) {
        state = 5;
        lastState = 9;
        Pid.changeGain(1.00);
        PORTA ^= 0x03;
      }

      break;

    case 10: // brake State

      break;

    case 11:
    
      if(lastState == 4)
        state = 5;
        
      else
        state = returnState;
      
      break;

    case 12:
      if (lastState == 4)
        state = 5;

      if (lastState == 5)
        state = 6;

      returnState = state;

      break;

    default:
      state = 0;
      break;
  }
  if (state < 10)
    returnState = state;
}
