#pragma once

#include <stdint.h>

class Motor {
  public:
    Motor(uint16_t *_OCRnX, uint8_t *_PORTX, uint8_t _bitMask);
    void accelerate();
    void decelerate();
    void initSpeed(uint8_t newSpeed);
    void changeDir();
    void brake();
    int getOCRnX();
    int getPORTX();
    int getSpeed();

  private:
    const uint8_t minSpeed = 0x40; //
    const uint8_t maxSpeed = 0x50; //
    uint8_t currSpeed;
    uint16_t *OCRnX;
    uint8_t *PORTX;
    uint8_t bitMask;
    uint8_t dir;
};
