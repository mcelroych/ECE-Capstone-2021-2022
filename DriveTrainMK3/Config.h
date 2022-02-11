#pragma once

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "Motor.h"
#include "PID.h"

// Global variables
const uint16_t adcThresh = 0x012C; // Threshold of 300
volatile uint8_t lADCvalue, rADCvalue;
int adcI;
uint8_t baseSpeed = 0x40;
double Pv;
int diff;
int state, lastState;
bool fallingEdge;
bool inStart;
int timer2OVF;
int t1;
float inches;

// Initialization of class objects
Motor lMotor = Motor(&OCR1A, &PORTA, 0x01);
Motor rMotor = Motor(&OCR1B, &PORTA, 0x02);
PID Pid = PID(0.00, 1.00, 3.00);

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

  // Configure ADC pins on PortF pins
  DDRF &= ~0xFF; // Set PF0 - PF7 as inputs

  // Configure HC-SR04 sensor pins
  // PB4/Pin10: Trigger Pin, PE4/Pin2: Echo Pin
  DDRB |= 0x10; // set Pin10 as output
  DDRE &= 0x10; // Set PIN2 as input

  // Initialize variables
  state = 0;
  lastState = 0;
  timer2OVF = 0;
  fallingEdge = false;
  inStart = true;
  inches = 0.00;
}
