#include <Servo.h>

Servo rotate;
Servo upDown;
Servo screen;
Servo launcherServo;  // create servo object to control a servo

#define LA_PWM_PIN       2
#define ROTATE_SERVO_PIN 3
#define UPDOWN_SERVO_PIN 4
#define SCREEN_SERVO_PIN 7
#define LA_DIR_PIN       23

#define ROTATE_START 173
#define ROTATE_CUP   65
#define ROTATE_STEPS ROTATE_START - ROTATE_CUP

#define UPDOWN_START 10
#define UPDOWN_DUMP  37
#define UPDOWN_HOLD  110
#define UPDOWN_TOP   155
#define UPDOWN_STEPS UPDOWN_TOP - UPDOWN_HOLD
#define UPDOWN_DUMP_STEPS UPDOWN_HOLD - UPDOWN_DUMP

#define LA_UP_DIR   0
#define LA_DOWN_DIR 1

#define LAUNCH_VALUE 2300
#define ENCODER_PIN 21
#define SERVO_PIN 5
#define WINCH_PIN 6
#define WINCH_DIR_PIN 26
#define LIMIT_SW_PIN 24

#define FIRE_PIN  30
#define ARM_GRAB  31
#define READY     32
#define START_BTN 52
#define DT_ENABLE 47
#define DT_RESUME 49

volatile int encoderValue = 0;
bool armed = false;

void setup() {
  // put your setup code here, to run once:
  armInit();
  screenInit();
  launcherInit();

  pinMode(DT_ENABLE, OUTPUT);
  pinMode(FIRE_PIN, INPUT);
  pinMode(READY, OUTPUT);
  pinMode(ARM_GRAB, INPUT);
  pinMode(START_BTN, INPUT);
  pinMode(DT_RESUME, OUTPUT);
  digitalWrite(DT_RESUME, LOW);
  while (!digitalRead(START_BTN));
  
  //setup**************************
  launcherArm();
  armPrep();
  digitalWrite(DT_ENABLE, HIGH);

  //grab beads one*****************
  while (!digitalRead(ARM_GRAB));
  armGrabBead();

  digitalWrite(READY, HIGH);
  delay(500);
  digitalWrite(READY, LOW);
  
  digitalWrite(DT_RESUME, HIGH);
  delay(500);
  digitalWrite(DT_RESUME, LOW);

  //Fire One***********************
  while (!digitalRead(FIRE_PIN));
  launcherFire();
  delay(500);
  launcherArm();
  armPrep();

  digitalWrite(READY, HIGH);
  digitalWrite(DT_RESUME, HIGH);
  delay(500);
  digitalWrite(READY, LOW);
  digitalWrite(DT_RESUME, LOW);  

  //grab beads TWO*****************
  while (!digitalRead(ARM_GRAB));
  armGrabBead();

  digitalWrite(READY, HIGH);
  digitalWrite(DT_RESUME, HIGH);
  delay(500);
  digitalWrite(READY, LOW);
  digitalWrite(DT_RESUME, LOW);

  //Fire Two***********************
  while (!digitalRead(FIRE_PIN));
  launcherFire();
  delay(500);
  launcherArm();
  armPrep();

  digitalWrite(READY, HIGH);
  digitalWrite(DT_RESUME, HIGH);
  delay(500);
  digitalWrite(READY, LOW);
  digitalWrite(DT_RESUME, LOW);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}

void launcherInit() {
  pinMode(ENCODER_PIN, INPUT);
  pinMode(WINCH_PIN, OUTPUT);
  pinMode(WINCH_DIR_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);
  launcherServo.attach(SERVO_PIN);
  launcherServo.write(15);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), count, FALLING);
}

void launcherArm() {
  launcherFire();
  //pull launcher down if it is not armed
  if (armed == false) {
    for (;;) {
      // Wind up winch
      digitalWrite(WINCH_DIR_PIN, HIGH);
      analogWrite(WINCH_PIN, 255);

      // Wait for limit switch to be pressed
      if (digitalRead(LIMIT_SW_PIN) == 1) {
        analogWrite(WINCH_PIN, 0);
        break;
      }
    }
    lock();
    encoderValue = 0;
    delay(2000);
    //let out string
    for (;;) {
      // Switch Directions
      digitalWrite(WINCH_DIR_PIN, LOW);
      analogWrite(WINCH_PIN, 255);

      if (encoderValue >= LAUNCH_VALUE) {
        analogWrite(6, 0); //stop the winch
        armed = true;
        break;
      }
    }
  }
}

void launcherFire() {
  launcherServo.write(15);
  armed = false;
}

bool isArmed() {
  return armed;
}

volatile void count() {
  encoderValue++;
}


void lock() {
  launcherServo.write(105);
}

void armInit() {
  pinMode(LA_DIR_PIN, OUTPUT);
  pinMode(LA_PWM_PIN, OUTPUT);

  analogWrite(LA_PWM_PIN, 0);

  rotate.attach(ROTATE_SERVO_PIN);
  upDown.attach(UPDOWN_SERVO_PIN);

  rotate.write(ROTATE_START);
  upDown.write(UPDOWN_START);
}

void armGrabBead() {

  //it takes 8.5 seconds to reach the top so make delay not go past the
  //.75 seconds is used in the prep
  digitalWrite(LA_DIR_PIN, LA_UP_DIR);
  analogWrite(LA_PWM_PIN, 254);

  for (int i = 0; i < UPDOWN_STEPS; i ++ ) {
    upDown.write(UPDOWN_TOP - i);
    delay(49);
  }//this will  total a delay of 2.2 seconds

  delay(5545);
  analogWrite(LA_PWM_PIN, 0);

  //drop in the launcher phase
  for (int i = 0; i < ROTATE_STEPS; i++) {
    rotate.write(ROTATE_START - i);
    delay(10);
  }

  digitalWrite(LA_DIR_PIN, LA_DOWN_DIR);
  analogWrite(LA_PWM_PIN, 254);
  delay(1000);
  analogWrite(LA_PWM_PIN, 0);

  for (int i = 0; i < UPDOWN_DUMP_STEPS; i++) {
    upDown.write(UPDOWN_HOLD - i);
    delay(10);
  }
  
  delay(2000);

  //reset
  rotate.write(ROTATE_START);
  upDown.write(UPDOWN_START);

  digitalWrite(LA_DIR_PIN, LA_DOWN_DIR);
  analogWrite(LA_PWM_PIN, 254);
  delay(7500);
  analogWrite(LA_PWM_PIN, 0);

}

void armPrep() {
  digitalWrite(LA_DIR_PIN, LA_UP_DIR);
  analogWrite(LA_PWM_PIN, 254);
  delay(750);
  upDown.write(UPDOWN_TOP);
  analogWrite(LA_PWM_PIN, 0);
}

void screenInit() {
  screen.attach(SCREEN_SERVO_PIN);
  screen.write(10);
}

void screenMove() {
  screen.write(170);
  delay(1000);
  screen.write(10);
}
