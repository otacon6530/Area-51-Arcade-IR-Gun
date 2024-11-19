#include <digitalWriteFast.h>

//Cache locality attempt
struct Coordinates {
    int x = 25;
    int y = 131;
};

Coordinates coords;
const int aimPin = 7;  // Analog output pin that the LED is attached to
const int firePin = 6;  // Analog output pin that the LED is attached to
const byte vSyncPin = 3;
const byte hSyncPin = 2;
int counter = 0;
int cycleCounter = 0;

const int MAX_Y = 262; 
const int MIN_Y = 0;
const int MAX_X = 49;
const int MIN_X = 3;

void setup()
{
  pinModeFast(vSyncPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(vSyncPin), vSync, RISING);
  pinModeFast(hSyncPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hSyncPin), hSync, RISING);
 
}

void vSync(){
  counter = 225;
}

void hSync() {
    // If counter matches coords.y (between 19 and 239)
    if (counter == coords.y) {
        delayMicroseconds(coords.x); // Delay between 4 and 47 microseconds
        pinModeFast(aimPin, OUTPUT);
        pinModeFast(aimPin, INPUT);
    }
    counter = (counter + 1) % MAX_Y;
}

inline void test(){
  cycleCounter = (cycleCounter + 1) % 10000;
  if(cycleCounter==1) {
    coords.x++;
    if(coords.x >= 48) {
      coords.x = 0;
      coords.y++;
      if(coords.y >= 261) {
        coords.y = 0;
      }
    }
  }
}

void loop()
{
  test();
}

