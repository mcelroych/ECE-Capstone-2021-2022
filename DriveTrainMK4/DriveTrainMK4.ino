/*
*/

#include "Config.h"
#include "Functions.h"

int main(void) {
  startUp();

  cli();

  initPWM();
  initADC();
  initTimer2Trig();
  initINT4();

  sei();

  // Infinite Loop
  for (;;) {
    switch (state) {

      case 0: // start State
        trackLine();
        break;

      case 1: // trackLine State
        trackLine();
        break;

      case 2: // turnRight State
        turnRight();
        break;

      case 3: // down State
        trackLine();
        break;

      case 4: // turnAround State
        turnAround();
        break;

      case 5: // back State
        trackLine();
        break;

      case 6: // turnLeft State
        turnLeft();
        break;

      case 7: // end State
        trackLine();
        break;

      case 8: // reverse State
        reverse();
        break;

      case 9: // brake State
        brake();
        break;

      default:
        state = 0;
        break;
    }

    nextState();
  }
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

//
ISR(TIMER2_COMPA_vect) {
  TCCR2A &= ~0x80; // Disconnect OCR2A from the pin
}

//
ISR(TIMER2_OVF_vect) {
  timer2OVF++; // Increment for final computation
}

//
ISR(INT4_vect) {
  if (fallingEdge) {
    inches = (TCNT2 + (255 * timer2OVF) - t1) / 18.50;
    fallingEdge = false;
    TCCR2B &= ~0x01;
  }
  else {
    t1 = TCNT2;
    timer2OVF = 0;
    fallingEdge = true;
  }
}
