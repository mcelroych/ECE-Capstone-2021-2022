/*  Program to test the drive train prototype
 *  
 *  Pin out:
 *    Sensor array:
 *      Sensors 1-8 -> Analog pins A0-A7 respectively
 *    Motor Driver:
 *      Dir1 -> Pin22 (PA0)
 *      PWM1 -> Pin11 (PB5)
 *      Dir2 -> Pin23 (PA1)
 *      PWM2 -> Pin12 (PB6)
 *    
 *  Recommended changes: 
 *    Lower Sensor array to comply with advised distance from surface (0.25")  
 * 
 */
 #include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

int main() {

  init();

  cli();

  // Configure Direction pins for motor driver
  // PA0: Left wheel  PA1: right wheel
  // 0 = forward, 1 = reverse
  DDRA |= 0x03;   // Set PA0 and PA1 as outputs
  PINA &= ~0x03;  // Write a 0 to PINA0 and PINA1

  // Configure PWM outputs
  // PB5: OC1A (Left wheel), PB6: OC1B (Right Wheel)
  DDRB |= 0x60;
 
  initPWM();
  
  for (;;) {
    
  }
  return 0;
}

// Initializes PWMs on Timer1 and Timer3
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
  OCR1A = 0x80;
  OCR1B = 0x80;
}

// Initalizes ADC Conversions
void initADC() {
  
}
