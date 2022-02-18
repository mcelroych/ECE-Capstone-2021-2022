#pragma once

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "Motor.h"
#include "PID.h"

// Global variables
uint8_t lineValue;
int IRvalue;
bool turnCond;
uint8_t minSpeed = 0x27;
uint8_t baseSpeed = 0x35;
uint8_t maxSpeed = 0x3E;
int state, lastState, returnState;
bool inStart;

// Initialization of class objects
Motor lMotor = Motor(&OCR1A, &PORTA, 0x01, minSpeed, baseSpeed, maxSpeed);
Motor rMotor = Motor(&OCR1B, &PORTA, 0x02, minSpeed, baseSpeed, maxSpeed);
PID Pid = PID(0x08, 1.00, 2.00);

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
  DDRF &= ~0xFF; // Set PF0 - PF7 as inputs
  PORTF |= 0xFF; // Activate Pull-up Resistors

  // Configure Pins on PortK for IR-Sensor
  DDRK &= ~0x01; // Set PK0 - PK2 as inputs
  PORTK |= 0x01; // Activate Pull-up resistors

  // Configure PCINT0 Pins for interrupts
  DDRB &= ~0x0F; // Set PB0 - PB3 as inputs
  PORTL &= ~0x0F; // Disable Pull-up Resistors

  // Configure start pin
  DDRL &= ~0x01; // Set PL0 as input
  PORTL &= ~0x01; // Disable Pull-up Resistor

  // Initialize variables
  state = 0;
  lastState = 0;
  inStart = true;
  turnCond = false;
  IRvalue = 0;
}
