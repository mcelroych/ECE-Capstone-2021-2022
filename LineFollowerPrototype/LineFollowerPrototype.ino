/*  Program to test the drive train prototype
 *  
 *  Pin out:
 *    Sensor array:
 *      Sensors 1-8 -> Analog pins A0-A7 respectively
 *    Motor Driver:
 *      Dir1 -> Pin22 (PA0)
 *      PWM1 -> Pin11 (PB5)
 *      Dir2 -> Pin23 (PA1)
 *      PWM2 -> Pin5  (PE3)
 *    
 *  Recommended changes: 
 *    Lower Sensor array to comply with advised distance from surface (0.25")  
 * 
 */
 #include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int main() {

  init();

  cli();

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A0, A1, A2, A3, A4, A5, A6, A7
  }, SensorCount);

  // Configure Direction pins for motor driver
  // PA0: Left wheel  PA1: right wheel
  // 0 = forward, 1 = reverse
  DDRA |= 0x03;   // Set PA0 and PA1 as outputs
  PINA &= ~0x03;  // Write a 0 to PINA0 and PINA1

  // Configure PWM outputs
  // PB5: OC1A (Left wheel), PE3: OC3A (Right wheel)
  DDRB |= 0x20;
  DDRE |= 0x08;

  initCalibration();
  initPWM();
  
  for (;;) {
    // read calibrated sensor values and obtain a measure of the line position
    // from 0 to 5000
    uint16_t position = qtr.readLineWhite(sensorValues);

    if (position > 4000) {
      OCR3A = 0x40;
    }
    
    if (position < 3000) {
      OCR1A = 0x40;
    }
    
    if (3000 < position < 4000) {
      OCR1A = 0x80;
      OCR3A = 0x80;
    }

  }

  return 0;
}

// Initializes PWMs on Timer1 and Timer3
void initPWM() {
  // Set all bits in TCCRnX registers to 0
  TCCR1A &= ~0xFF;
  TCCR1B &= ~0xDF;
  TCCR3A &= ~0xFF;
  TCCR3B &= ~0xDF;

  // Set the COMnA bits in the TCCRnA registers
  TCCR1A |= 0x80;
  TCCR3A |= 0x80;

  // Set the WGM bits in the TCCRnA registers
  TCCR1A |= 0x01;
  TCCR3A |= 0x01;

  // Set the CS bits in TCCRnB for a prescaler of 64
  TCCR1B |= 0x03;
  TCCR3B |= 0x03;

  // Set the last WGM bits in TCCRnB for fast PWM, 8-bit
  TCCR1B |= 0x08;
  TCCR3B |= 0x08;

  // Set the OCRnA registers for a 50% duty cycle
  OCR1A = 0x80;
  OCR3A = 0x80;

}

// Calibrates the QTR sensor runs for 10 seconds
void initCalibration() {
  // turn on Arduino's LED to indicate we are in calibration mode
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  // turn off Arduino's LED to indicate we are through with calibration
  digitalWrite(LED_BUILTIN, LOW);
}
