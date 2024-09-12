#include <hidboot.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#include "Arduino.h"
#include "Wire.h"
#include <math.h>

#define MOUSE 1
#define DEBUG 2
#define DEFAULT -1
#define topLeft 0
#define topRight 1
#define bottomLeft 2
#define bottomRight 3
#define PI 3.14159265358979323846

int state = DEFAULT;

class MouseRptParser : public MouseReportParser
{
protected:
    void OnMouseMove(MOUSEINFO *mi) override;
    void OnLeftButtonUp(MOUSEINFO *mi) override;
    void OnLeftButtonDown(MOUSEINFO *mi) override;
    void OnRightButtonUp(MOUSEINFO *mi) override;
    void OnRightButtonDown(MOUSEINFO *mi) override;
    void OnMiddleButtonUp(MOUSEINFO *mi) override;
    void OnMiddleButtonDown(MOUSEINFO *mi) override;
};

#include "DFRobotIRPosition.h"
DFRobotIRPosition myDFRobotIRPosition;

struct {
              float coords[4][2];
              double sides[4];
              int cameraCenter=512;
              float x = 0;
              float y= 0;
            }beacon;
struct {
              
              int width = 47;
              int height = 239;
              int x = 25;
              int y = 125;
              int vSyncOffset=240;
              int hSyncCounter=0;
            } screen;

//Define used pins
const int digitalPin = 7; 
const int firePin = 6;     // Digital output pin for the fire action
const byte tempTrig = 4;
const byte vSyncPin = 3;
const byte hSyncPin = 2;

void printResult();
void vSync();
void hSync();
void test();
void IRPosition();

void MouseRptParser::OnMouseMove(MOUSEINFO *mi)
{
    int dX = (mi->dX < -3) ? -1 : (mi->dX > 3) ? 1 : 0;
    int dY = (mi->dY < -1) ? -4 : (mi->dY > 1) ? 4 : 0;

    screen.x = constrain(screen.x + dX, 3, 49);
    screen.y = constrain(screen.y + dY, 0, 263);
}

void MouseRptParser::OnLeftButtonUp(MOUSEINFO *mi) {}
void MouseRptParser::OnLeftButtonDown(MOUSEINFO *mi)
{
    //USB Left mouse button will fire gun.
    pinMode(firePin, OUTPUT);
    digitalWrite(firePin, LOW);
    delay(17);
    pinMode(firePin, INPUT);
}

void MouseRptParser::OnRightButtonUp(MOUSEINFO *mi) {}
void MouseRptParser::OnRightButtonDown(MOUSEINFO *mi)
{
    int tempX = screen.x;
    int tempY = screen.y;
    screen.x = -262;
    screen.y = -262;
    delay(50);
    pinMode(firePin, OUTPUT);
    digitalWrite(firePin, LOW);
    delay(17);
    pinMode(firePin, INPUT);
    delay(100);
    screen.x = tempX;
    screen.y = tempY;
}

void MouseRptParser::OnMiddleButtonUp(MOUSEINFO *mi) {}
void MouseRptParser::OnMiddleButtonDown(MOUSEINFO *mi) {}

USB Usb;
USBHub Hub(&Usb);
HIDBoot<USB_HID_PROTOCOL_MOUSE> HidMouse(&Usb);
MouseRptParser Prs;

void setup()
{
    Serial.begin(115200);
    Serial.println("Start");

    myDFRobotIRPosition.begin();

    //Interrupt for Vertical and Horizontal sync.
    pinMode(vSyncPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(vSyncPin), vSync, FALLING);
    pinMode(hSyncPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(hSyncPin), hSync, RISING);

    pinMode(tempTrig, INPUT_PULLUP);

    if (Usb.Init() == -1) {
        Serial.println("USB Initialization failed!");
        while (1); // Halt if USB initialization fails
    }

    delay(200);
    HidMouse.SetReportParser(0, &Prs);
}

void vSync()
{
    screen.hSyncCounter = screen.vSyncOffset;
}

void hSync()
{
    screen.hSyncCounter++;
    if (screen.hSyncCounter == screen.y) {
        delayMicroseconds(screen.x);
        pinMode(digitalPin, OUTPUT);
        digitalWrite(digitalPin, LOW);
        pinMode(digitalPin, INPUT);
    }
    if (screen.hSyncCounter >= screen.height) {
        screen.hSyncCounter = 0;
    }
}

void test()
{
    delay(10);
    screen.y++;
    if (screen.x == screen.width) screen.x = 0;
    if (screen.y >= screen.height) {
        screen.y = 0;
        screen.x++;
    }
}
// Function to calculate the slope between two points
float calculateSlope(float x1, float y1, float x2, float y2) {
    if (x2 == x1) return NAN;  // Vertical line
    return (y2 - y1) / (x2 - x1);
}

// Function to calculate the angle between two points relative to the horizontal
float calculateAngle(float x1, float y1, float x2, float y2) {
    return atan2(y2 - y1, x2 - x1);
}

// Function to rotate a point around a specific point (px, py) by an angle
void rotatePoint(float &xx, float &yy, float angle, float px, float py) {
    // Translate point to origin
    float xTranslated = xx - px;
    float yTranslated = yy - py;

    // Perform the rotation
    float xNew = xTranslated * cos(angle) - yTranslated * sin(angle);
    float yNew = xTranslated * sin(angle) + yTranslated * cos(angle);

    // Translate point back
    xx = xNew + px;
    yy = yNew + py;
}

// Function to ensure the long sides of the rectangle are horizontal
void alignRectangle(float coords[4][2], float px, float py) {
    // Calculate lengths of sides
    float lengths[4];
    for (int i = 0; i < 4; i++) {
        int next = (i + 1) % 4;
        lengths[i] = sqrt(pow(coords[next][0] - coords[i][0], 2) + pow(coords[next][1] - coords[i][1], 2));
    }

    // Determine the angle required to align the longest side horizontally
    float longestSideAngle = 0;
    float longestLength = lengths[0];
    for (int i = 1; i < 4; i++) {
        int next = (i + 1) % 4;
        if (lengths[i] > longestLength) {
            longestLength = lengths[i];
            longestSideAngle = calculateAngle(coords[i][0], coords[i][1], coords[next][0], coords[next][1]);
        }
    }

    // Calculate the rotation angle to align the longest side horizontally
    float rotationAngle = -longestSideAngle;

    // Rotate all points around (px, py)
    for (int i = 0; i < 4; i++) {
        rotatePoint(coords[i][0], coords[i][1], rotationAngle, px, py);
    }
}

void IRPosition()
{
    myDFRobotIRPosition.requestPosition();

    if (myDFRobotIRPosition.available()) {
        for (int i = 0; i < 4; i++) {
            int RFx = myDFRobotIRPosition.readX(i);
            int RFy = myDFRobotIRPosition.readY(i);
            int xQuadrant = (RFx > 512) ? 1 : -1;
            int yQuadrant = (RFy > 512) ? 1 : -1;

            if (xQuadrant == -1 && yQuadrant == 1) {
                beacon.coords[topLeft][0] = RFx;
                beacon.coords[topLeft][1] = RFy;
            } else if (xQuadrant == 1 && yQuadrant == 1) {
                beacon.coords[topRight][0] = RFx;
                beacon.coords[topRight][1] = RFy;
            } else if (xQuadrant == -1 && yQuadrant == -1) {
                beacon.coords[bottomLeft][0] = RFx;
                beacon.coords[bottomLeft][1] = RFy;
            } else if (xQuadrant == 1 && yQuadrant == -1) {
                beacon.coords[bottomRight][0] = RFx;
                beacon.coords[bottomRight][1] = RFy;
            }
        }

        float topM = calculateSlope(beacon.coords[topLeft][0],beacon.coords[topLeft][1],beacon.coords[topRight][0],beacon.coords[topRight][1]);

        if (beacon.coords[topLeft][0] >= 1023 || beacon.coords[bottomRight][0] >= 1023 || beacon.coords[bottomLeft][0] >= 1023 || beacon.coords[topRight][0] >= 1023) {
            screen.y = -screen.height;
            screen.x = -screen.width;
        } else {
            // Align the rectangle
            for (int i = 0; i < 4; i++) {
                float px = beacon.coords[i][0];
                float py = beacon.coords[i][1];
                float s = sin(atan(-topM));
                float c = cos(atan(-topM));

                px -= beacon.cameraCenter;
                py -= beacon.cameraCenter;

                float xnew = px * c - py * s;
                float ynew = px * s + py * c;

                beacon.coords[i][0] = xnew + beacon.cameraCenter;
                beacon.coords[i][1] = ynew + beacon.cameraCenter;
            }

            if (beacon.coords[topLeft][0] < 1023 && beacon.coords[topRight][0] < 1023) {
                beacon.x = float(beacon.cameraCenter - beacon.coords[topLeft][0]) / (beacon.coords[topRight][0] - beacon.coords[topLeft][0]);
            }
            screen.x = screen.width * (1 - beacon.x);

            if (beacon.coords[topLeft][0] < 1023 && beacon.coords[bottomLeft][0] < 1023) {
                beacon.y = float(beacon.coords[topLeft][1] - beacon.cameraCenter) / (beacon.coords[topLeft][1] - beacon.coords[bottomLeft][1])+0.10;
            }
            screen.y = screen.height * (1 - beacon.y);
            printResult(beacon.x,beacon.y);
        }
    } else {
        Serial.println("Device not available!");
    }
}

int lastButtonState = 0;
void loop()
{
    switch (state)
    {
    case MOUSE:
        Usb.Task();
        break;

    case DEBUG:
        test();
        break;

    default:
        IRPosition();
        int buttonState = digitalRead(tempTrig);

        if (buttonState != lastButtonState) {
            if (buttonState == LOW) {
                pinMode(firePin, OUTPUT);
                digitalWrite(firePin, LOW);
                delay(17);
                pinMode(firePin, INPUT);
            }
        }
        lastButtonState = buttonState;
    }
}

void printResult(float xPerc, float yPerc)
{
    Serial.print("IR Positions:\t");
    for (int i = 0; i < 4; i++) {
        Serial.print(beacon.coords[i][0]);
        Serial.print("\t,");
        Serial.print(beacon.coords[i][1]);
        Serial.print(";\t");
    }
    Serial.print("Rotated:\t");
    Serial.print("Resolution: ");
    Serial.print(beacon.coords[1][0] - beacon.coords[0][0]);
    Serial.print(",");
    Serial.print(beacon.coords[0][1] - beacon.coords[2][1]);
    Serial.print("\tPerc:");
    Serial.print(beacon.x);
    Serial.print(",");
    Serial.print(beacon.y);
    Serial.print(";\tPosition:");
    Serial.print(screen.x);
    Serial.print(",");
    Serial.print(screen.y);
    Serial.println();
}
