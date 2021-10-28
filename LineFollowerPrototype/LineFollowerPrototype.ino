/*  Program to test the drive train prototype

    Pin out:
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
const uint8_t maxSpeed = 75;
uint8_t motorASpeed;
uint8_t motorBSpeed;
uint8_t adcMux[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
volatile uint8_t ADCvalue;
int adcI;


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

  // Infinite Loop
  for (;;) {
    /*if ((ADCvalue[0] < adcThresh) && (ADCvalue[7] < adcThresh)) { //1XXX XXX1
      halt();
      }
      else if (ADCvalue[0] < adcThresh) { //1XXX XXXX or XXXX XXX1
      handleCorner();
      }
      while (((ADCvalue[3] < adcThresh) || (ADCvalue[4] < adcThresh)) &&
           ((ADCvalue[0] > adcThresh) && (ADCvalue[7] > adcThresh))) {  //0XXX 1XX0 or 0XX1 XXX0
      lineStraight();
      }*/
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

// Function to gradually increase motor speed
// to prevent the robot from leaping forward
/*uint8_t accelerate(uint8_t motorXSpeed) {
  if (motorXSpeed < maxSpeed)
    motorXSpeed += 5;
  return motorXSpeed;
  }

  // Function to gradually decrease motor speed
  uint8_t decelerate(uint8_t motorXSpeed) {
  if (motorXSpeed >= 0x05)
    motorXSpeed -= 5;
  return motorXSpeed;
  }

  // Function to stop drivetrain
  void halt () {
  OCR1A = 0x00;
  OCR1B = 0x00;
  }

  // Follows the straight line
  // Checks to make sure middle two sensors always sees white
  void lineStraight() {
  changeDir(0x00);
  if ((ADCvalue[4] > adcThresh)) {  //XXX0 XXXX
    motorASpeed  = decelerate(motorASpeed);
    OCR1A = motorASpeed;
  }
  else if ((ADCvalue[3] > adcThresh)) { //XXXX 0XXX
    motorBSpeed  = decelerate(motorBSpeed);
    OCR1B = motorBSpeed;
  }
  else {
    motorASpeed = accelerate(motorASpeed);
    motorBSpeed = accelerate(motorBSpeed);
    OCR1A = motorASpeed;
    OCR1B = motorBSpeed;
  }
  }

  void changeDir(uint8_t dir) {
  PINA &= ~0x03;
  PINA |= dir;
  }
  void handleCorner() {
  changeDir(0x01);
  while ((ADCvalue[2] < adcThresh) || (ADCvalue[4] > adcThresh)) {  //XXX1 XXXX or XXXX X1XX

  }

  changeDir(0x00);
  }*/
