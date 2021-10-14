#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <QTRSensors.h>

#define MotorMask 0x03    // Selects
#define PwmMask 0xC0      // Selects
#define EncoderMask 0xF0  // Selects
#define LineMask 0xFF     // Selects 

int main() {

  // Configuration of the interrupt registers
  init();

  cli();

  // Configuration of I/O registers
  DDRA |= MotorMask;    // configures outputs on PORTA
  
  DDRB |= PwmMask;      // Configures outputs on PORTB

  DDRK &= ~EncoderMask; // Configures inputs on PORTK
  PORTK |= EncoderMask; // Enables pull-up resistor

  DDRL &= ~LineMask;    // Configures inputs on PORTL
  PORTL |= LineMask;    // Enables pull-up resistor


  for (;;) {

  }

  return 0;
}

void initPWM() {
  
}
