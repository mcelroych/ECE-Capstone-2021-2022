/*
*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Motor.h"

Motor *lMotor = nullptr;
Motor *rMotor = nullptr;

// Global variables
const uint16_t adcThresh =  0x01F4; // Threshold of 500
uint8_t adcMux[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
volatile uint8_t ADCvalue;
int adcI;
int decoder[2];
int nextState;

int main(void) {
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

  // Configure PE4 and PE5 as inputs for decoders
  DDRE &= ~0x30;

  DDRC &= ~0x40;

  // Select the first input channel on the adcMux
  adcI = 0;

  cli();

  initPWM();
  initADC();
  //initDEC();

  sei();

  // Initialization of class objects
  lMotor = new Motor(&OCR1A, &PORTA, 0x01);
  rMotor = new Motor(&OCR1B, &PORTA, 0x02);

  // Initialization of variables
  nextState = 0;
  decoder[0] = 0;
  decoder[1] = 0;

  while((PINC & 0x60) == 0x00);
  
  // Infinite Loop
  for (;;) {
    //Serial.println(ADCvalue);
    switch (nextState) {
      case 0:   // DriveForwardState1
        lMotor->initSpeed(0x42);
        rMotor->initSpeed(0x40);

        while ((ADCvalue & 0xC3) != 0x00);
        nextState = 2;
        break;

      case 1:   // CenterState
        while ((ADCvalue & 0x18) != 0x18) {
          if ((ADCvalue & 0x10) == 0x00) {
            rMotor->decelerate();
          }
          else if ((ADCvalue & 0x08) == 0x00) {
            lMotor->decelerate();
          }
        }
        nextState = 2;
        break;

      case 2:   //TrackLineState1
        while ((ADCvalue & 0x18) > 0x00) {
          if ((ADCvalue & 0x10) == 0x00)
            rMotor->decelerate();
          else if ((ADCvalue & 0x08) == 0x00)
            lMotor->decelerate();
          else {
            lMotor->accelerate();
            rMotor->accelerate();
          }
          if ((ADCvalue & 0x81) != 0x00)
            break;
        }
        if (ADCvalue == 0x00)
          nextState = 6;
        else if ((ADCvalue & 0x81) == 0x81)
          nextState = 0;
        else if ((ADCvalue & 0x81) == 0x01)
          nextState = 4;
        else if ((ADCvalue & 0x81) == 0x80)
          nextState = 3;
        break;

      case 3:   // TurnLeftState

        lMotor->brake();
        rMotor->initSpeed(0x40);

        while ((ADCvalue & 0x80) == 0x80);

        if ((ADCvalue & 0x04) == 0x04)
          nextState = 8;

        break;

      case 4:   // TurnRightState
        rMotor->brake();
        lMotor->initSpeed(0x40);

        while ((ADCvalue & 0x01) == 0x01);

        if ((ADCvalue & 0x20) == 0x20)
          nextState = 2;

        break;

      case 5:   // TurnAroundState

        rMotor -> changeDir();

        while (ADCvalue > 0x00);
        while ((ADCvalue & 0x18) != 0x18);

        lMotor -> changeDir();

        nextState = 2;
        break;

      case 6:   // ReverseState
        lMotor -> changeDir();
        rMotor -> changeDir();
        nextState = 5;
        while (ADCvalue == 0x00);
        break;

      case 7:   // HaltState
        lMotor->brake();
        rMotor->brake();
        break;

      case 8:   // DriveForwardState2
        lMotor->initSpeed(0x40);
        rMotor->initSpeed(0x40);

        if (ADCvalue == 0x00)
          nextState = 7;
        break;

      default:
        break;
    }
    //Serial.println(ADCvalue);
  }

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

  // Checking for a white line, represented by a 1
  // If over white ADC is less than the threshhold
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

void initDEC() {
  //Set all bits in the EICRB, EIMSK, and EIFR Registers to 0
  EICRB &= ~0xFF;
  EIMSK &= ~0xFF;

  //
  EICRB |= 0x0F;

  //
  EIMSK |= 0x30;
}

ISR(INT4_vect) {
  decoder[0]++;
}

ISR(INT5_vect) {
  decoder[1]++;
}
