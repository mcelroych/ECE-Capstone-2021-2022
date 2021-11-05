#pragma once

void accelerateL() {
  if (motorASpeed < maxSpeed)
    motorASpeed += 1;
  OCR1A = motorASpeed;
}

void accelerateR() {
  if (motorBSpeed < maxSpeed)
    motorBSpeed += 1;
  OCR1B = motorBSpeed;
}

void decelerateL() {
  if (motorASpeed > 0x05)
    motorASpeed -= 1;
  OCR1A = motorASpeed;
}

void decelerateR() {
  if (motorBSpeed >= 0x05)
    motorBSpeed -= 1;
  OCR1B = motorBSpeed;
}

void setSpeedL(int newSpeed) {
  OCR1A = newSpeed;
}

void setSpeedR(int newSpeed) {
  OCR1B = newSpeed;
}
void changeDir(uint8_t newDir) {
  PINA &= ~0x03;
  PINA |= newDir;
}

void brakeL() {
  OCR1A = 0x00;
}

void brakeR() {
  OCR1B = 0x00;
}

void halt() {
  OCR1A = 0x00;
  OCR1B = 0x00;
}
