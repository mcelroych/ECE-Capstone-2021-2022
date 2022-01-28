/*
*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Motor.h"
#include "PID.h"



Motor *lMotor = nullptr;
Motor *rMotor = nullptr;
PID *Pid = nullptr;

// Global variables
const uint16_t adcThresh =  0x01F4; // Threshold of 500
uint8_t adcMux[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
volatile uint8_t lADCvalue, rADCvalue;
int adcI;
int lDec, rDec;
int state;

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

  // Select the first input channel on the adcMux
  adcI = 0;

  cli();

  initPWM();
  initADC();
  initDEC();

  sei();

  // Initialization of class objects
  lMotor = new Motor(&OCR1A, &PORTA, 0x01);
  rMotor = new Motor(&OCR1B, &PORTA, 0x02);
  Pid = new PID(0.00, 1.00, 2.00, 0.00 );

  // Initialization of variables
  state = 5;
  lDec = 0;
  rDec = 0;

  // Infinite Loop
  for (;;) {
    switch (state) {
      case 0: // stallState

        // while(;;);

        state = 1;

        break;

      case 1: // forwardState
        PORTA &= ~0x03;
        lMotor->initSpeed(0x40);
        rMotor->initSpeed(0x40);
        lDec = 0;
        rDec = 0;

        while (state == 1) {
          if (lDec - rDec > 2)
            lMotor->decelerate();
          else if (rDec - lDec > 2)
            lMotor->accelerate();
          Serial.print(lDec);
          Serial.print(" ");
          Serial.println(rDec);
        }

        break;

      case 2: // reverseState
        PORTA |= 0x03;
        lMotor->initSpeed(0x40);
        rMotor->initSpeed(0x40);
        lDec = 0;
        rDec = 0;

        while (state == 2) {
          if (lDec - rDec > 4)
            lMotor->decelerate();
          else if (rDec - lDec > 4)
            lMotor->accelerate();

        }
        break;

      case 3: // haltState
        lMotor->brake();
        rMotor->brake();
        break;

      case 4: // turnAroundState

        rMotor -> changeDir();

        while (rADCvalue > 0x00);
        while (rADCvalue != 0x03);

        lMotor -> changeDir();

        state = 5;
        break;

      case 5: // trackLineState
        Pid->reset();
        lMotor->initSpeed(0x40);
        rMotor->initSpeed(0x40);
        while (state == 5)
        {
          double Pv = lADCvalue - rADCvalue;
          int diff = Pid -> controlFunc(Pv);

          if (diff < 0)
            lMotor->decelerate();
          else if (diff > 0)
            rMotor->decelerate();
          else {
            lMotor->accelerate();
            rMotor->accelerate();
          }
        }

        break;

      default:
        break;

    }
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
  TCCR1B |= 0x02;

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

// ISR for ADC Completion
ISR(ADC_vect) {
  // Checking for a white line, represented by a 1
  // If over white ADC is less than the threshhold
  if (adcI < 4) {

    if (ADC < adcThresh) {
      rADCvalue |= 1 << (3 - adcI);
    }

    else {
      rADCvalue &= ~(1 << (3 - adcI));
    }
  }
  else {
    if (ADC < adcThresh) {
      lADCvalue |= 1 << (adcI - 4);
    }

    else {
      lADCvalue &= ~(1 << (adcI - 4));
    }
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
  EICRB &= ~0xFF;
  EIMSK &= ~0xFF;

  EICRB |= 0x0F;
  EIMSK |= 0x30;
}

ISR(INT4_vect) {
  lDec++;
}

ISR(INT5_vect) {
  rDec++;
}
