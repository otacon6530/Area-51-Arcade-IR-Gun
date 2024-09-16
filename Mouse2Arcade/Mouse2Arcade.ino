#include <hidboot.h>
#include <digitalWriteFast.h>
#include <TimerOne.h>

//Cache locality attempt
struct Coordinates {
    int x = 25;
    int y = 131;
};

Coordinates coords;
const int digitalPin = 7;  // Analog output pin that the LED is attached to
const int firePin = 6;  // Analog output pin that the LED is attached to
int counter = 0;
int cycleCounter = 0;

const int MAX_Y = 263; 
const int MIN_Y = 0;
const int MAX_X = 49;
const int MIN_X = 3;

const byte vSyncPin = 3;
const byte hSyncPin = 2;

class MouseRptParser : public MouseReportParser
{
protected:
	void OnMouseMove	(MOUSEINFO *mi);
	void OnLeftButtonUp	(MOUSEINFO *mi);
	void OnLeftButtonDown	(MOUSEINFO *mi);
	void OnRightButtonUp	(MOUSEINFO *mi);
	void OnRightButtonDown	(MOUSEINFO *mi);
	void OnMiddleButtonUp	(MOUSEINFO *mi);
	void OnMiddleButtonDown	(MOUSEINFO *mi);
};
void MouseRptParser::OnMiddleButtonUp	(MOUSEINFO *mi){};
void MouseRptParser::OnMiddleButtonDown	(MOUSEINFO *mi){};
void MouseRptParser::OnLeftButtonUp	(MOUSEINFO *mi){
  pinModeFast(firePin,INPUT);
};
void MouseRptParser::OnRightButtonUp	(MOUSEINFO *mi){};

void MouseRptParser::OnMouseMove(MOUSEINFO *mi)
{
    // If both dX and dY are 0, no movement, exit early
    if ((mi->dX | mi->dY) == 0) {
        return;
    }

    // Bitwise arithmetic to calculate sign (-1, 0, or 1) without comparisons
    int dX = (mi->dX >> 31) | (!!mi->dX); // Gets -1, 0, or 1 based on sign
    int dY = 4 * ((mi->dY >> 31) | (!!mi->dY)); // -4, 0, or 4 for dY

    // Update coordinates
    coords.x += dX;
    coords.y += dY;

    coords.x = constrain(coords.x, MIN_X, MAX_X);
    coords.y = constrain(coords.y, MIN_Y, MAX_Y);
}
void MouseRptParser::OnLeftButtonDown	(MOUSEINFO *mi)
{
  pinModeFast(firePin,OUTPUT);
};
void MouseRptParser::OnRightButtonDown	(MOUSEINFO *mi)
{
    int tempX = coords.x;
  int tempY = coords.y;
  coords.x = -262;
  coords.y = -262;
  pinModeFast(firePin,OUTPUT);
  pinModeFast(firePin,INPUT);
  coords.x = tempX;
  coords.y = tempY;
};

USB     Usb;
HIDBoot<USB_HID_PROTOCOL_MOUSE>    HidMouse(&Usb);
MouseRptParser                               Prs;

void setup()
{
   if (Usb.Init() == -1){
    delay( 200 );
  }
  HidMouse.SetReportParser(0, &Prs);
  //Serial.begin(115200);
  //Serial.println("Start");
  pinModeFast(vSyncPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(vSyncPin), vSync, RISING);
  pinModeFast(hSyncPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hSyncPin), hSync, RISING);
 
}

void vSync(){
  counter = 225;
}

void hSync(){
    counter++;
    if(counter==coords.y){ //19 to 239
      delayMicroseconds(coords.x); //4 to 47 range
      pinModeFast(digitalPin,OUTPUT);
      pinModeFast(digitalPin,INPUT);
    }
    if(counter>=262){
      counter=0;
    }
}
void hSync() {
    counter++;

    // If counter matches coords.y (between 19 and 239)
    if (counter == coords.y) {
        delayMicroseconds(coords.x); // Delay between 4 and 47 microseconds
        pinModeFast(digitalPin, OUTPUT);
        pinModeFast(digitalPin, INPUT);
    }
    counter = (counter >= 262) ? 0 : counter;
}

inline void test(){
  cycleCounter++;
  if(cycleCounter >= 20000) {
    cycleCounter = 0;
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
  Usb.Task();
  //test();
}

