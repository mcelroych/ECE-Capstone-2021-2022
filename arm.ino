#include <Servo.h>

Servo rotate;
Servo upDown;
Servo screen;

#define LA_PWM_PIN       2
#define ROTATE_SERVO_PIN 3
#define UPDOWN_SERVO_PIN 4
#define SCREEN_SERVO_PIN 7
#define LA_DIR_PIN       23

#define ROTATE_START 173
#define ROTATE_CUP   45
#define ROTATE_STEPS ROTATE_START - ROTATE_CUP

#define UPDOWN_START 10
#define UPDOWN_DUMP  55
#define UPDOWN_HOLD  100
#define UPDOWN_TOP   155
#define UPDOWN_STEPS UPDOWN_TOP - UPDOWN_HOLD

#define LA_UP_DIR   1
#define LA_DOWN_DIR 0


void armInit(){
  pinMode(LA_DIR_PIN, OUTPUT);
  pinMode(LA_PWM_PIN, OUTPUT);

  analogWrite(LA_PWM_PIN, 0);
  
  rotate.attach(ROTATE_SERVO_PIN);
  upDown.attach(UPDOWN_SERVO_PIN);

  rotate.write(ROTATE_START);
  upDown.write(UPDOWN_START);
}

void armGrabBead(){

  //it takes 8.5 seconds to reach the top so make delay not go past the
  //.75 seconds is used in the prep
  digitalWrite(LA_DIR_PIN, LA_UP_DIR);
  analogWrite(LA_PWM_PIN, 254);
  
  for(int i = 0; i < UPDOWN_STEPS; i ++ ){
    upDown.write(UPDOWN_TOP - i);
    delay(40);
  }//this will  total a delay of 2.2 seconds

  delay(5550);
  analogWrite(LA_PWM_PIN, 0);

  //drop in the launcher phase
  for(int i = 0; i < ROTATE_STEPS; i++){
    rotate.write(ROTATE_START - i);
    delay(10);
  }

  upDown.write(UPDOWN_DUMP);
  delay(2000);

  //reset
  rotate.write(ROTATE_START);
  upDown.write(UPDOWN_START);

  digitalWrite(LA_DIR_PIN, LA_DOWN_DIR);
  analogWrite(LA_PWM_PIN, 254);
  delay(8500);
  analogWrite(LA_PWM_PIN, 0);
  
}

void armPrep(){
  digitalWrite(LA_DIR_PIN, LA_UP_DIR);
  analogWrite(LA_PWM_PIN, 254);
  delay(750);
  upDown.write(UPDOWN_TOP);
  analogWrite(LA_PWM_PIN, 0);
}

void screenInit(){
  screen.attach(SCREEN_SERVO_PIN);
  screen.write(10);
}

void screenMove(){
  screen.write(170);
  delay(1000);
  screen.write(10);
}

void setup() {
  armInit();
  screenInit();
  //screenMove();
  delay(2000);
  armPrep();
  delay(1000);
  armGrabBead();

}

void loop() {
  // put your main code here, to run repeatedly:

}
