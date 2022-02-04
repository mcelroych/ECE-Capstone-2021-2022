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

// Initalizes ADC Conversions
void initADC() {

  // Set all bits in the ADCMUX and ADCSRX Registers to 0
  ADMUX &= ~0xFF;
  ADCSRA &= ~0xFF;
  ADCSRB &= ~0x4F;

  // Set the REFSn bits for a reference voltage of AVCC
  ADMUX |= 0x40;

  // Set the ADEN bit to enable ADC conversions
  ADCSRA |= 0x80;

  // Set the ADIE bit to enable interrupts
  ADCSRA |= 0x08;

  // Set the ADPSn bits for a prescaler of 2
  ADCSRA |= 0x01;

  // Set the ADSC bit to start conversion
  ADCSRA |= 0x40;
}

//
void initTimer2Trig() {
  TCCR2A |= 0x03; // Fast PWM
  OCR2A = 0x01;   // Compare value of 1
  TIMSK2 |= 0x03; // Output Compare A Interrupt and OVF Enabled
}

//
void  initINT4() {
  EICRB |= 0x01;
  EIMSK |= 0x10;
}

//
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
void getDistance() {
  TCCR2A |= 0x80; // OC2A set to non-inverting mode
  TCCR2B |= 0x01; // Turn on timer2 with prescaler of 1
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