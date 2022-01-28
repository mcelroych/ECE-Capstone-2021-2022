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
bool fallingEdge;
bool trig;
int timer2OVF;
int t1;
int echo;


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

  // Configure HC-SR04 trig and echo pins
  DDRB |= 0x10; // set Pin10 as output
  DDRE &= 0x10; // Set PIN2 as inout

  // Configure ADC pins on PortF pins
  DDRF &= ~0xFF; // Set PF0 - PF7 as inputs

  // Select the first input channel on the adcMux
  adcI = 0;

  cli();

  initPWM();
  initADC();
  //initTimer2Trig();
  //initINT4();

  sei();

  // Initialization of class objects
  lMotor = new Motor(&OCR1A, &PORTA, 0x01);
  rMotor = new Motor(&OCR1B, &PORTA, 0x02);
  Pid = new PID(0.00, 1.00, 0.80);

  // Initialization of variables
  int state = 5;
  uint8_t baseSpeed = 0x40;
  timer2OVF = 0;
  t1 = 0;
  echo = 80;
  float inches;
  fallingEdge = false;
  trig = false;
  double Pv;
  int diff;

  // Infinite Loop
  for (;;) {
    switch (state) {
      case 0: // stallState

        // while(;;);

        state = 1;

        break;

      case 1: // forwardState
        PORTA &= ~0x03;
        lMotor->initSpeed(baseSpeed);
        rMotor->initSpeed(baseSpeed);

        while (((lADCvalue & 0x08) != 0x00) || ((rADCvalue & 0x08) != 0x00));

        state = 5;
        break;

      case 2: // reverseState
        PORTA |= 0x03;
        lMotor->initSpeed(baseSpeed);
        rMotor->initSpeed(baseSpeed);
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

        lMotor->initSpeed(baseSpeed);
        rMotor->initSpeed(baseSpeed);

        while (state == 5)
        {
          Pv = lADCvalue - rADCvalue;
          diff = Pid -> controlFunc(Pv);

          //inches = echo / 18.50;

          if (diff > 0) {
            lMotor->initSpeed(baseSpeed - diff);
            rMotor->initSpeed(baseSpeed + diff);
          }
          else if (diff < 0) {
            rMotor->initSpeed(baseSpeed - diff);
            lMotor->initSpeed(baseSpeed + diff);
          }

          if (rADCvalue == 0x0F) {
            rMotor->brake();
            lMotor->initSpeed(baseSpeed);
            while (lADCvalue > 3);
            while (lADCvalue < 1);
            rMotor->initSpeed(baseSpeed);
          }

          else if (lADCvalue == 0x0F) {
            lMotor->brake();
            rMotor->initSpeed(baseSpeed);
            while (rADCvalue < 7);
          }

          else if ((lADCvalue == 0x0F) && (rADCvalue == 0x0F)) {
            state = 4;
            lMotor -> changeDir();
            rMotor -> changeDir();
            lMotor->initSpeed(baseSpeed);
            rMotor->initSpeed(baseSpeed);
            while ((lADCvalue == 0x00) && (rADCvalue == 0x00));

            break;
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
  ADMUX |= adcMux[adcI];

  // Set the ADSC bit to start conversion
  ADCSRA |= 0x40;
}

void initTimer2Trig() {
  TCCR2A |= 0x83; // Fast PWM non inverting mode
  OCR2A = 0x01;   // Compare value of 1
  TIMSK2 |= 0x03; // Output Compare A Interrupt and OVF Enabled
  TCCR2B |= 0x01; // Prescaler of 1
}

ISR(TIMER2_COMPA_vect) {
  if (trig) {
    TCCR2A |= 0x80;
    trig = false;
  }
  else {
    TCCR2A &= ~0x80;
  }
}

ISR(TIMER2_OVF_vect) {
  timer2OVF++;
  if (fallingEdge == false) {
    timer2OVF = 0;
  }
}

void  initINT4() {
  EICRB |= 0x01;
  EIMSK |= 0x10;
}

ISR(INT4_vect) {
  if (fallingEdge) {
    echo = TCNT2 + (256 * timer2OVF) - t1;
    fallingEdge = false;
    trig = true;
  }
  else {
    t1 = TCNT2;
    fallingEdge = true;
  }
}
