/*
*/

#include "Config.h"
#include "Functions.h"

int main(void) {
  startUp();

  cli();

  initPWM();
  initADC();

  sei();

  // Infinite Loop
  for (;;) {

    switch (state) {

      case 0: // trackLine State
        trackLine();
        break;

      case 1: // turnLeft State
        turnLeft();
        break;

      case 2: // turnRight State
        turnRight();
        break;

      case 3: // reverse State
        reverse();
        break;

      case 4: // turnAround State
        turnAround();
        break;

      case 5: // brake State
        brake();
        break;

      default:
        state = 0;
        break;
    }

    nextState();
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
  // iterate till the end of the sensors
  // let it overflow, will be used to detect end of conversions
  adcI++;

  if (adcI < 7) {
    // Select the next ADC input channel on the mux
    ADMUX &= ~0x1F;
    ADMUX |= adcI;

    // Set the ADSC bit to start conversion
    ADCSRA |= 0x40;
  }
}
