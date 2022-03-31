//

#include "Config.h"
#include "Functions.h"

int main(void) {
  startUp();

  cli();

  initPWM();
  initISR();

  sei();

  // Infinite Loop
  for (;;) {
    Serial.print(state);
    Serial.print(" ");
    Serial.println(returnState);
    switch (state) {

      case 0: // stall State
        brake();
        break;

      case 1: // start State
        lMotor.initSpeed(baseSpeed + 5);
        rMotor.initSpeed(baseSpeed);
        getLine();
        break;

      case 2: // trackLine State
        trackLine();
        break;

      case 3: // turnRight State
        turnRight();
        break;

      case 4: // down State
        trackLine();
        break;

      case 5: // turnAround State
        turnAround();
        break;

      case 6: // back State
        trackLine();
        break;

      case 7: // turnLeft State
        turnLeft();
        break;

      case 8: // end State
        trackLine();
        break;

      case 9: // reverse State
        reverse();
        break;

      case 10: // brake State
        brake();
        break;

      case 11: // return State
        lMotor.initSpeed(baseSpeed);
        rMotor.initSpeed(baseSpeed);
        break;

      case 12: // Delay State

        brake();

        delay(1000);
        
        break;

      default:
        state = 0;
        break;
    }

    nextState();
  }
}

//
ISR(PCINT0_vect) {
  if ((PINB & 0x01) == 0x01) // brake state PB0 D53
    state = 10;
  else if ((PINB & 0x08) == 0x08) //PB3 D50
    state = 11;
}
