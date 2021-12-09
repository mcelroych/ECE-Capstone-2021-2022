#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int Direction = 22; // assigns pin 22 to DIR1 of Motor Driver
int PWM = 11; // assigns 11 to PWM1 of Motor Driver

void setup()  // setup loop
{ Serial.begin(9600);
  pinMode(Direction, OUTPUT); // declares pin 22 as output
  pinMode(PWM, OUTPUT);  // declares pin 11 as output
 

}


int main(void) {
  init();
  DDRB |= 0x20;
  cli();
  initPWM();
  sei();

  




}

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
  OCR1A = 0x00;
  OCR1B = 0x00;
}


/* digitalWrite(Direction, HIGH);
  for (int i=0; i<10000; i++){
    analogWrite(PWM, i);
  Serial.print(i);
  //for(;;);

  }

  delay(100);*/
