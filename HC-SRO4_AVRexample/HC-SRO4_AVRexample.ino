/* Test for HC-SR04 Sensor

    Pinout:
      Trig -> Pin10 (PB4) OCR2A
      Echo -> Pin2  (PE4) INT[4]

*/
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

bool fallingEdge;
bool trig;
int timer2OVF;
int t1;
int t2;

int main() {
  init();
  Serial.begin(9600);

  DDRB |= 0x10; // set Pin10 as output
  DDRE &= 0x10; // Set PIN2 as inout

  timer2OVF = 0;
  t1 = 0;
  t2 = 0;
  float uS;
  float inches;
  fallingEdge = false;
  trig = false;

  cli();

  initTimer2Trig();
  initINT4();

  sei();


  for (;;) {
    uS = (t2-t1)*8.00;
    inches = uS/148.00;
    Serial.print(uS);
    Serial.print(" ");
    Serial.println(inches);
  }
  return 0;
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
    t2 = TCNT2 + (256 * timer2OVF);
    fallingEdge = false;
    trig = true;
  }
  else {
    t1 = TCNT2;
    fallingEdge = true;
  }
}
