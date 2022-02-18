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

      default:
        state = 0;
        break;
    }

    nextState();
  }
}
