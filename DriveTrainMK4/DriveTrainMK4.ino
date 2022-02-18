//

#include "Config.h"
#include "Functions.h"

int main(void) {
  startUp();

  cli();

  initPWM();
  initISR();

  sei();

  //while(1);

  // Infinite Loop
  for (;;) {
    switch (state) {

      case 0: // stall State

        break;

      case 1: // start State
        trackLine();
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

      case 11: // allignFront State
        trackLine(minSpeed);
        break;

      case 12: // allignBack State
        reverse(minSpeed);
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
  if ((PINB & 0x0f) == 0x01)
    state = 10;
  else if ((PINB & 0x0f) == 0x02)
    state = 11;
  else if ((PINB & 0x0f) == 0x04)
    state = 12;
  else if ((PINB & 0x0f) == 0x08)
    state = returnState;
}
