#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


//FIX THESE VALUES BY CALIBRATING           

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


uint8_t servo = 3;
uint8_t Motor = 4;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pinMode(3, OUTPUT);
  

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
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    Arm();
    delay(1000);
    Fire();    
 
}



void Arm(){
  
  digitalWrite(3, LOW);
  pwm.setPWM(Motor, 500, 3500);
  delay(3000); //change to 4 secs for initialization
  pwm.setPWM(Motor, 1024, 1024);
  pwm.setPWM(servo, 0, 320);
  delay(2000); //change to 4 secs for initialization
  //pwm.setPWM(servo, 0, 150);
  digitalWrite(3, HIGH);
  pwm.setPWM(Motor, 500, 3500);
  delay(2400); //change to 4 secs for initialization
  pwm.setPWM(Motor, 1024, 1024);



  
  
}
 void Disarm(){
  
  digitalWrite(3, HIGH);
  pwm.setPWM(servo, 0, 150);
  pwm.setPWM(Motor, 500, 3500);
  delay(3000); //change to 4 secs for initialization
  pwm.setPWM(Motor, 1024, 1024);
  
   
}


void Fire() {
  pwm.setPWM(servo, 0, 150);
}



void loop() {
 
 
  

}
