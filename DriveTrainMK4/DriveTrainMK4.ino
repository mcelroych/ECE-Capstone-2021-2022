//


#include "Config.h"
#include "Functions.h"

int main(void) {
  startUp();

  cli();

  initPWM();

  sei();

  //while(1);

  // Infinite Loop
  for (;;) {
    switch (state) {

      case 0: // start State
        trackLine();
        break;

      case 1: // trackLine State
        trackLine();
        break;

      case 2: // turnRight State
        turnRight();
        break;

      case 3: // down State
        trackLine();
        break;

      case 4: // turnAround State
        turnAround();
        break;

      case 5: // back State
        trackLine();
        break;

      case 6: // turnLeft State
        turnLeft();
        break;

      case 7: // end State
        trackLine();
        break;

      case 8: // reverse State
        reverse();
        break;

      case 9: // brake State
        brake();
        break;

      default:
        state = 0;
        break;
    }

    nextState();
  }
}
