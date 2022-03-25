#pragma once

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "Motor.h"
#include "PID.h"

// Global variables
uint8_t lValue,rValue, lineValue;
int IRvalue;
bool turnCond;
uint8_t minSpeed = 0x30;
uint8_t baseSpeed = 0x3A;
uint8_t maxSpeed = 0x44;
int state, lastState, returnState;
bool inStart;

// Initialization of class objects
Motor lMotor = Motor(&OCR1A, &PORTA, 0x01, minSpeed, baseSpeed, maxSpeed);
Motor rMotor = Motor(&OCR1B, &PORTA, 0x02, minSpeed, baseSpeed, maxSpeed);
PID Pid = PID(0x00, 1.00, 1.00);

void startUp() {
  init();

  Serial.begin(9600);

  // Configure Direction pins for motor driver
  // PA0/Pin22: Left wheel, PA1/Pin23: right wheel
  // 0 = forward, 1 = reverse
  DDRA |= 0x03;   // Set PA0 and PA1 as outputs
  PORTA &= ~0x03;  // Drive PINA0 and PINA1 low

  // Configure PWM outputs
  // PB5/Pin11: OCR1A (Left wheel), PB6/Pin12: OCR1B (Right Wheel)
  DDRB |= 0x60; // Set PB5 and PB6 as outputs

  // Configure Pins on PortF for Linesensor Array
  // PF0/A0: Sensor8, -> ... -> , PF7/A7: Sensor1
  DDRF &= ~0xFF; // Set PF0 - PF7 as inputs
  PORTF |= 0xFF; // Activate Pull-up Resistors

  // Configure Pins on PortK for IR-Sensor
  // PK0/A8: RightSensor , PK1/A9: LeftSensor
  DDRK &= ~0x03; // Set PK0 - PK1 as inputs
  PORTK |= 0x03; // Activate Pull-up resistors

  // Configure PCINT0 Pins for interrupts
  // PB0/Pin53: BrakePin, PB1/Pin52: FrontPin,
  // PB2/Pin51: BackPin, PB3/Pin50: ReturnPin
  DDRB &= ~0x0F; // Set PB0 - PB3 as inputs
  PORTB &= ~0x0F; // Disable Pull-up Resistors

  // Configure start pin PL7/Pin42
  DDRL &= ~0x80; // Set PL7 as input
  PORTL &= ~0x80; // Disable Pull-up Resistor

  // Initialize variables
  state = 1;
  lastState = 0;
  inStart = true;
  IRvalue = 0;      
  turnCond = false; 
}
