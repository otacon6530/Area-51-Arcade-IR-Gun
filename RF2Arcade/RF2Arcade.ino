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

float coords[4][2];
double rotX[4];    ///< Store the X position
double rotY[4];    ///< Store the Y position
int cameraCenter = 512;
float xPerc = 0;
float yPerc = 0;

int maxX = 47;
int maxY = 239;
int x = 25;
int y = 135;

const int digitalPin = 7;  // Digital output pin for the LED
const int firePin = 6;     // Digital output pin for the fire action

int counter = 0;

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

    x = constrain(x + dX, 3, 49);
    y = constrain(y + dY, 0, 263);
}

void MouseRptParser::OnLeftButtonUp(MOUSEINFO *mi) {}
void MouseRptParser::OnLeftButtonDown(MOUSEINFO *mi)
{
    Serial.println("L Butt Dn");
    pinMode(firePin, OUTPUT);
    digitalWrite(firePin, LOW);
    delay(50);
    pinMode(firePin, INPUT);
}

void MouseRptParser::OnRightButtonUp(MOUSEINFO *mi) {}
void MouseRptParser::OnRightButtonDown(MOUSEINFO *mi)
{
    int tempX = x;
    int tempY = y;
    x = 0;
    y = 0;
    delay(500);
    pinMode(firePin, OUTPUT);
    digitalWrite(firePin, LOW);
    delay(17);
    pinMode(firePin, INPUT);
    delay(100);
    x = tempX;
    y = tempY;
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

    pinMode(vSyncPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(vSyncPin), vSync, FALLING);

    pinMode(hSyncPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(hSyncPin), hSync, RISING);

    if (Usb.Init() == -1) {
        Serial.println("USB Initialization failed!");
        while (1); // Halt if USB initialization fails
    }

    delay(200);
    HidMouse.SetReportParser(0, &Prs);
    pinMode(tempTrig, INPUT_PULLUP);
}

void vSync()
{
    counter = 240;
}

void hSync()
{
    counter++;
    if (counter == y) {
        delayMicroseconds(x);
        pinMode(digitalPin, OUTPUT);
        digitalWrite(digitalPin, LOW);
        pinMode(digitalPin, INPUT);
    }
    if (counter >= 262) {
        counter = 0;
    }
}

void test()
{
    delay(10);
    y++;
    if (x == 48) x = 0;
    if (y >= 262) {
        y = 0;
        x++;
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
void rotatePoint(float &x, float &y, float angle, float px, float py) {
    // Translate point to origin
    float xTranslated = x - px;
    float yTranslated = y - py;

    // Perform the rotation
    float xNew = xTranslated * cos(angle) - yTranslated * sin(angle);
    float yNew = xTranslated * sin(angle) + yTranslated * cos(angle);

    // Translate point back
    x = xNew + px;
    y = yNew + py;
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
                coords[topLeft][0] = RFx;
                coords[topLeft][1] = RFy;
            } else if (xQuadrant == 1 && yQuadrant == 1) {
                coords[topRight][0] = RFx;
                coords[topRight][1] = RFy;
            } else if (xQuadrant == -1 && yQuadrant == -1) {
                coords[bottomLeft][0] = RFx;
                coords[bottomLeft][1] = RFy;
            } else if (xQuadrant == 1 && yQuadrant == -1) {
                coords[bottomRight][0] = RFx;
                coords[bottomRight][1] = RFy;
            }
        }

        float topM = calculateSlope(coords[topLeft][0],coords[topLeft][1],coords[topRight][0],coords[topRight][1]);
        float cx = 512;
        float cy = 512;

        if (coords[topLeft][0] >= 1023 || coords[bottomRight][0] >= 1023 || coords[bottomLeft][0] >= 1023 || coords[topRight][0] >= 1023) {
            y = -262;
            x = -262;
        } else {
            // Define the point around which to rotate
            float centerX = 512;
            float centerY = 512;

            // Align the rectangle
            //alignRectangle(coords, centerX, centerY);
            for (int i = 0; i < 4; i++) {
                float px = coords[i][0];
                float py = coords[i][1];
                float s = sin(atan(-topM));
                float c = cos(atan(-topM));

                px -= cx;
                py -= cy;

                float xnew = px * c - py * s;
                float ynew = px * s + py * c;

                coords[i][0] = xnew + cx;
                coords[i][1] = ynew + cy;
            }

            if (coords[topLeft][0] < 1023 && coords[topRight][0] < 1023) {
                xPerc = float(cameraCenter - coords[topLeft][0]) / (coords[topRight][0] - coords[topLeft][0]);
            }
            x = maxX * (1 - xPerc);

            if (coords[topLeft][0] < 1023 && coords[bottomLeft][0] < 1023) {
                yPerc = float(coords[topLeft][1] - cameraCenter) / (coords[topLeft][1] - coords[bottomLeft][1])+0.10;
            }
            y = maxY * (1 - yPerc);
            printResult(xPerc,yPerc);
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
        Serial.print(coords[i][0]);
        Serial.print("\t,");
        Serial.print(coords[i][1]);
        Serial.print(";\t");
    }
    Serial.print("Rotated:\t");
    for (int i = 0; i < 4; i++) {
        Serial.print(rotX[i]);
        Serial.print("\t,");
        Serial.print(rotY[i]);
        Serial.print(";\t");
    }
    Serial.print("Resolution: ");
    Serial.print(coords[1][0] - coords[0][0]);
    Serial.print(",");
    Serial.print(coords[0][1] - coords[2][1]);
    Serial.print("\tPerc:");
    Serial.print(xPerc);
    Serial.print(",");
    Serial.print(yPerc);
    Serial.print(";\tPosition:");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.println();
}
