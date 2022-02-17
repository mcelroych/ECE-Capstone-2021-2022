#include <Wire.h>
#define SERVO_FREQ 6000
#include <Servo.h>

#define LAUNCH_VALUE 2000
#define ENCODER_PIN 21
#define SERVO_PIN 5
#define WINCH_PIN 6
#define WINCH_DIR_PIN 26
#define LIMIT_SW_PIN 24
volatile int encoderValue = 0;
bool armed = false;

Servo launcherServo;  // create servo object to control a servo

void launcherInit(){
  pinMode(ENCODER_PIN,INPUT);
  pinMode(WINCH_PIN,OUTPUT);
  pinMode(WINCH_DIR_PIN,OUTPUT);
  pinMode(SERVO_PIN,OUTPUT);
  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);
  launcherServo.attach(SERVO_PIN);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN),count,FALLING);
}

void launcherArm(){
  launcherFire();
  //pull launcher down
  digitalWrite(WINCH_DIR_PIN, HIGH);
  analogWrite(WINCH_PIN, 255);

  //wait for the limit switch to be hit then stop
  while(digitalRead(LIMIT_SW_PIN) == 0);
  analogWrite(WINCH_PIN, 0);

  lock();
  encoderValue = 0;
  delay(3000);
  //let out string
  digitalWrite(WINCH_DIR_PIN, LOW);
  analogWrite(WINCH_PIN, 255);
  while (encoderValue < LAUNCH_VALUE);
  analogWrite(6, 0); //stop the winch
  armed = true;
  
}

void launcherFire() {
  launcherServo.write(15);
  armed = false;
}

bool isArmed(){
  return armed;
}

volatile void count(){
  encoderValue++;
}


void lock() {
  launcherServo.write(105);
}

//*******this part is for testing only*********************
//*******don't include in the main robot code**************
void setup ()// code for counting the increasing values of encoder ticks void setup()
{
  Serial.begin(9600);

  launcherInit();
  
  loop();
  launcherArm();
  delay(2000);
  launcherFire();
}

void loop()
{
  Serial.print("Starting\n");
  Serial.print("Encoder Value="); 
  Serial.println(encoderValue);

}

 
