/*  Program to test the drive train prototype

    Pinout:
      Sensor array:
        Sensors 1-8 -> Analog pins A0-A7 respectively
      Motor Driver:
        Dir1 -> Pin22 (PA0)
        PWM1 -> Pin11 (PB5)
        Dir2 -> Pin23 (PA1)
        PWM2 -> Pin12 (PB6)

    Recommended changes:
      Lower Sensor array to comply with advised distance from surface (0.25")
      Ideal Distance 0.125" (1/8")

*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

const uint16_t adcThresh = 0x01F4; // Threshold of 500
const uint8_t maxSpeed = 45;
uint8_t motorASpeed;
uint8_t motorBSpeed;
uint8_t adcMux[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
volatile uint8_t ADCvalue;
int adcI;

#include "Functions.h"

int main() {

  init();

  Serial.begin(9600);
  // Configure Direction pins for motor driver
  // PA0/Pin22: Left wheel, PA1/Pin23: right wheel
  // 0 = forward, 1 = reverse
  DDRA |= 0x03;   // Set PA0 and PA1 as outputs
  PINA &= ~0x03;  // Write a 0 to PINA0 and PINA1

  // Configure PWM outputs
  // PB5/Pin11: OCR1A (Left wheel), PB6/Pin12: OCR1B (Right Wheel)
  DDRB |= 0x60; // Set PB5 and PB6 as outputs

  // Configure ADC pins on PortF pins
  DDRF &= ~0xFF; // Set PF0 - PF7 as inputs

  // Select the first input channel on the adcMux
  adcI = 0;

  motorASpeed = 0;
  motorBSpeed = 0;

  cli();

  initPWM();
  initADC();

  sei();

  int nextState = 3 ;

  bool turnCheck;

  // Infinite Loop
  for (;;) {

    switch (nextState) {
      case 0: // trackLineState
        while ((ADCvalue & 0x18) > 0x00) {
          if ((ADCvalue & 0x10) == 0x00)
            decelerateL();
          else if ((ADCvalue & 0x08) == 0x00)
            decelerateR();
          else {
            accelerateL();
            accelerateR();
          }
          if ((ADCvalue & 0x81) != 0x00)
            break;
        }
        if ((ADCvalue & 0x81) == 0x81)
          nextState = 4;
        else if ((ADCvalue & 0x81) == 0x01)
          nextState = 1;
        else if ((ADCvalue & 0x81) == 0x80)
          nextState = 2;

        break;
      case 1: // turnLeftState
        setSpeedR(maxSpeed);
        setSpeedL(maxSpeed / 2);

        while ((ADCvalue & 0x80) == 0x80);

        if ((ADCvalue & 0x08) == 0x08)
          nextState = 0;

        break;
      case 2: // turnRightState
        setSpeedL(maxSpeed);
        setSpeedR(maxSpeed / 2);

        while ((ADCvalue & 0x01) == 0x01);

        if ((ADCvalue & 0x10) == 0x10)
          nextState = 0;

        break;
      case 3: // turnAroundState
        setSpeedR(maxSpeed);
        setSpeedL(maxSpeed);
        changeDir(0x01);

        while (ADCvalue > 0x00);

        while ((ADCvalue & 0x08) != 0x08);
        nextState = 0;

        changeDir(0x00);

        break;
      case 4: // haltState
        halt();

        break;
      default:
        break;
    }

    Serial.println(ADCvalue);
  }
  return 0;
}

// Initializes PWM on Timer1
void initPWM() {

  // Set all bits in TCCRnX registers to 0
  TCCR1A &= ~0xFF;
  TCCR1B &= ~0xDF;

  // Set the COMnX bits in the TCCRnA registers
  TCCR1A |= 0xA0;

  // Set the WGM bits in the TCCRnA registers
  TCCR1A |= 0x01;

  // Set the CS bits in TCCRnB for a prescaler of 64
  TCCR1B |= 0x03;

  // Set the last WGM bits in TCCRnB for fast PWM, 8-bit
  TCCR1B |= 0x08;

  // Set the OCRnA registers for a 50% duty cycle
  OCR1A = 0x00;
  OCR1B = 0x00;
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

  // Set the ADCnD bits to 0 to disable digital inputs (save power)
  //DIDR0 |= 0xFF;

  // Set the ADSC bit to start conversion
  ADCSRA |= 0x40;
}

// ISR for ADC Completion
ISR(ADC_vect) {

  if (ADC < adcThresh) {
    ADCvalue |= 1 << adcI;
  }

  else {
    ADCvalue &= ~(1 << adcI);
  }

  // Increment adcI
  // Prevent it from overflowing
  adcI++;
  if (adcI > 7)
    adcI = 0;

  // Select the next ADC input channel on the mux
  ADMUX &= ~0x1F;
  ADMUX |= adcMux[adcI];

  // Set the ADSC bit to start conversion
  ADCSRA |= 0x40;
}
