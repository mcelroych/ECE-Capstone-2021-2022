#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//FIX THESE VALUES BY CALIBRATING           
#define USMIN  100 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  1600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


uint8_t servonum5 = 5;
uint8_t servonum4 = 4;
uint8_t servonum2 = 2;
uint8_t servonum1 = 1;
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  pwm.setPWM(servonum4, 1024, 1024);

}

void screenmove() {

  /*for (uint16_t microsec = USMIN; microsec < USMAX; microsec++) {
    pwm.writeMicroseconds(servonum5, microsec);
    delay(1);
  }

  delay(3000);
  
  for (uint16_t microsec = USMAX; microsec < USMIN; microsec--) {
    pwm.writeMicroseconds(servonum5, microsec);
    delay(1);
  }
  
  delay(3000);*/

  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum5, 0, pulselen);
  }

  delay(3000);
  
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum5, 0, pulselen);
  }

  delay(3000);
  
 
}

void loop() {

  screenmove();

}
