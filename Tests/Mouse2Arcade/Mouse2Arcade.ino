#include <hidboot.h>
#include <usbhub.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// -------- CONFIGURATION --------
const byte aimPin = 7;     // PD7
const byte firePin = 6;    // PD6
const byte vSyncPin = 3;   // INT1
const byte hSyncPin = 2;   // INT0

const int MAX_Y = 262;
const int MIN_Y = 0;
const int MAX_X = 49;
const int MIN_X = 3;

// -------- GLOBAL STATE --------
volatile int counter = 0;

struct Coordinates {
    int x = 25;
    int y = 131;
};
volatile Coordinates coords;

volatile bool aimFlag = false;
volatile int aimDelay = 0;

// -------- MOUSE PARSER --------
class MouseRptParser : public MouseReportParser {
protected:
    void OnMouseMove(MOUSEINFO *mi);
    void OnLeftButtonUp(MOUSEINFO *mi);
    void OnLeftButtonDown(MOUSEINFO *mi);
    void OnRightButtonDown(MOUSEINFO *mi);
};

void MouseRptParser::OnMouseMove(MOUSEINFO *mi) {
    if ((mi->dX | mi->dY) == 0) return;

    int dX = (mi->dX > 0) - (mi->dX < 0);       // -1, 0, 1
    int dY = 4 * ((mi->dY > 0) - (mi->dY < 0)); // -4, 0, 4

    noInterrupts();
    coords.x = constrain(coords.x + dX, MIN_X, MAX_X);
    coords.y = constrain(coords.y + dY, MIN_Y, MAX_Y);
    interrupts();
}

void MouseRptParser::OnLeftButtonDown(MOUSEINFO *mi) {
    DDRD |= (1 << 6);  // firePin = OUTPUT
}

void MouseRptParser::OnLeftButtonUp(MOUSEINFO *mi) {
    DDRD &= ~(1 << 6); // firePin = INPUT
}

void MouseRptParser::OnRightButtonDown(MOUSEINFO *mi) {
    noInterrupts();
    int tempX = coords.x;
    int tempY = coords.y;
    coords.x = -1;
    coords.y = -1;
    DDRD |= (1 << 6);  // Pulse fire
    DDRD &= ~(1 << 6);
    coords.x = tempX;
    coords.y = tempY;
    interrupts();
}

// -------- USB SETUP --------
USB Usb;
HIDBoot<USB_HID_PROTOCOL_MOUSE> HidMouse(&Usb);
MouseRptParser Prs;

// -------- SETUP --------
void setup() {
    Usb.Init();
    HidMouse.SetReportParser(0, &Prs);

    pinMode(vSyncPin, INPUT_PULLUP);
    pinMode(hSyncPin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(vSyncPin), vSyncISR, RISING);
    attachInterrupt(digitalPinToInterrupt(hSyncPin), hSyncISR, RISING);
}

// -------- VSYNC --------
void vSyncISR() {
    counter = 225;
}

void waitMicrosecondsTight(int us) {
  // ~16 NOPs per microsecond at 16 MHz
  for (int i = 0; i < us; ++i) {
    for (int j = 0; j < 4; ++j) {
      asm volatile("nop");
    }
  }
}

// -------- HSYNC --------
void hSyncISR() {
    int yCopy, xCopy;

    noInterrupts();
    yCopy = coords.y;
    xCopy = coords.x;
    interrupts();

    if (counter == yCopy && xCopy >= 0) {
        waitMicrosecondsTight(xCopy);
        DDRD |= (1 << 7);  // aimPin = OUTPUT
        DDRD &= ~(1 << 7); // aimPin = INPUT
    }

    counter = (counter + 1) % MAX_Y;
}

// -------- LOOP --------
void loop() {
    Usb.Task();
}
