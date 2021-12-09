// Internal
#include "Globals.h"
#include "PWM.h"

// Third-party
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

uint8_t servonum2 = 2;
uint8_t servonum1 = 1;
uint8_t servonum = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pinMode(2, OUTPUT);

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  pwm.writeMicroseconds(servonum1, USMINSWIVEL);
  pwm.writeMicroseconds(servonum, USMINROD);

  digitalWrite(2, LOW);
  pwm.setPWM(servonum2, 500, 3500);
  delay(4000); //change to 4 secs for initialization

  delay(10);
}

void grabBead()
{

  Serial.println(servonum);

  /* for (uint16_t pulselen = SERVOMIN; pulselen < 500; pulselen++) {
      pwm.setPWM(servonum, 0, pulselen); 
  }

  delay(1000);
  
   for (uint16_t pulselen = SERVOMAX; pulselen < SERVOMIN; pulselen--) {
      pwm.setPWM(servonum, 0, pulselen); 
  }*/

  //movement for swivel
  for (uint16_t microsec = USMINSWIVEL; microsec < USMAXSWIVEL; microsec++)
  {
    pwm.writeMicroseconds(servonum1, microsec);
  }

  //movement for rod
  for (uint16_t microsec = USMINROD; microsec < USMAXROD; microsec++)
  {
    pwm.writeMicroseconds(servonum, microsec);
  }

  delay(500);

  for (uint16_t microsec = USMAXSWIVEL; microsec > USDROPSWIVEL; microsec--)
  {
    pwm.writeMicroseconds(servonum1, microsec);
  }

  delay(500);

  for (uint16_t microsec = USMAXROD; microsec > USDROPROD; microsec--)
  {
    pwm.writeMicroseconds(servonum, microsec);
  }

  delay(500);

  //servonum++;
  if (servonum > 1)
    servonum = 0;
}

void extendup()
{

  digitalWrite(2, HIGH);

  pwm.setPWM(servonum2, 500, 3500);

  delay(1000);

  for (uint16_t microsec = USMINROD; microsec < USVERTICALROD; microsec++)
  {
    pwm.writeMicroseconds(servonum, microsec);
  }

  delay(3000);

  for (uint16_t microsec = USVERTICALROD; microsec > USSTRAIGHTROD; microsec--)
  {
    pwm.writeMicroseconds(servonum, microsec);
  }

  delay(4000);

  for (uint16_t microsec = USSTRAIGHTROD; microsec < USMAXROD; microsec++)
  {
    pwm.writeMicroseconds(servonum, microsec);
  }

  delay(2000);

  pwm.setPWM(servonum2, 1024, 1024);

  for (uint16_t microsec = USMINSWIVEL; microsec > USDROPSWIVEL; microsec--)
  {
    pwm.writeMicroseconds(servonum1, microsec);
  }

  for (uint16_t microsec = USMAXROD; microsec > USDROPROD; microsec--)
  {
    pwm.writeMicroseconds(servonum, microsec);
  }

  delay(1000);

  for (uint16_t microsec = USDROPROD; microsec > USMINROD; microsec--)
  {
    pwm.writeMicroseconds(servonum, microsec);
  }

  for (uint16_t microsec = USDROPSWIVEL; microsec < USMINSWIVEL; microsec++)
  {
    pwm.writeMicroseconds(servonum1, microsec);
  }

  digitalWrite(2, LOW);

  pwm.setPWM(servonum2, 500, 3500);

  delay(14000);

  pwm.setPWM(servonum2, 1024, 1024);

  delay(1000);
}

void loop()
{

  //grabBead();
  extendup();
}
