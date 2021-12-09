#pragma once

//FIX THESE VALUES BY CALIBRATING
#define SERVOMIN 150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600 // This is the 'maximum' pulse length count (out of 4096)

#define USMINROD 600  // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAXROD 1700 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define USDROPROD 900
#define USSTRAIGHTROD 1300
#define USVERTICALROD 2000

#define USMINSWIVEL 2200
#define USMAXSWIVEL 2200
#define USDROPSWIVEL 1150

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates