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

  DDRA |= 0x03;
  PINA |= 0x03;
  DDRB |= 0x20;
  DDRE |= 0x08;

  initCalibration();
  initPWM();
  for (;;) {
    // read calibrated sensor values and obtain a measure of the line position
    // from 0 to 5000
    uint16_t position = qtr.readLineWhite(sensorValues);

  }

  return 0;
}

// Initializes PWMs on Timer0 and Timer1
void initPWM() {
  TCCR1A &= ~0xFF;
  TCCR1B &= ~0xDF;
  TCCR3A &= ~0xFF;
  TCCR3B &= ~0xDF;
  TCCR1A |= 0x83;
  TCCR1B |= 0x1B;
  TCCR3A |= 0x83;
  TCCR3B |= 0x1B;
  OCR1A = 0xFFFF;
  OCR3A = 0x0F00;
}

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
