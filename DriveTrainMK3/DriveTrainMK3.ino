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
    
    /*switch (state) {
      case 0: // stallState

        // while(;;);

        state = 1;

        break;

      case 1: // forwardState
        PORTA &= ~0x03;
        lMotor.initSpeed(baseSpeed);
        rMotor.initSpeed(baseSpeed);

        while (((lADCvalue & 0x08) != 0x00) || ((rADCvalue & 0x08) != 0x00));

        state = 5;
        break;

      case 2: // reverseState
        PORTA |= 0x03;
        lMotor.initSpeed(baseSpeed);
        rMotor.initSpeed(baseSpeed);
        break;

      case 3: // haltState
        lMotor.brake();
        rMotor.brake();
        break;

      case 4: // turnAroundState

        rMotor.changeDir();

        while (rADCvalue > 0x00);
        while (rADCvalue != 0x03);

        lMotor.changeDir();

        state = 5;
        break;

      case 5: // trackLineState
        Pid.reset();

        lMotor.initSpeed(baseSpeed);
        rMotor.initSpeed(baseSpeed);

        while (state == 5)
        {
          Pv = lADCvalue - rADCvalue;
          diff = Pid.controlFunc(Pv);

          //inches = echo / 18.50;

          if (diff > 0) {
            lMotor.initSpeed(baseSpeed - diff);
            rMotor.initSpeed(baseSpeed + diff);
          }
          else if (diff < 0) {
            rMotor.initSpeed(baseSpeed - diff);
            lMotor.initSpeed(baseSpeed + diff);
          }

          if (rADCvalue == 0x0F) {
            rMotor.brake();
            lMotor.initSpeed(baseSpeed);
            while (lADCvalue > 3);
            while (lADCvalue < 1);
            rMotor.initSpeed(baseSpeed);
          }

          else if (lADCvalue == 0x0F) {
            lMotor.brake();
            rMotor.initSpeed(baseSpeed);
            while (rADCvalue < 7);
          }

          else if ((lADCvalue == 0x0F) && (rADCvalue == 0x0F)) {
            state = 4;
            lMotor.changeDir();
            rMotor.changeDir();
            lMotor.initSpeed(baseSpeed);
            rMotor.initSpeed(baseSpeed);
            while ((lADCvalue == 0x00) && (rADCvalue == 0x00));

            break;
          }

        }
        break;

      default:
        break;

      }*/
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
  // Prevent it from overflowing
  adcI++;

  if (adcI > 7)
    adcI = 0;

  // Select the next ADC input channel on the mux
  ADMUX &= ~0x1F;
  ADMUX |= adcI;

  // Set the ADSC bit to start conversion
  ADCSRA |= 0x40;
}
