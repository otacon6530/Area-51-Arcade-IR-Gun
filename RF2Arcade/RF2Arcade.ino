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

int positionX[4];  ///< Store the X position
int positionY[4];  ///< Store the Y position
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

void IRPosition()
{
    myDFRobotIRPosition.requestPosition();

    if (myDFRobotIRPosition.available()) {
        for (int i = 0; i < 4; i++) {
            int RFx = myDFRobotIRPosition.readX(i);
            int RFy = myDFRobotIRPosition.readY(i);

            if (RFx >= 1023 || RFy >= 1023) {
                positionX[topLeft] = 0;
            }

            int xQuadrant = (RFx > 512) ? 1 : -1;
            int yQuadrant = (RFy > 512) ? 1 : -1;

            if (xQuadrant == -1 && yQuadrant == 1) {
                positionX[topLeft] = RFx;
                positionY[topLeft] = RFy;
            } else if (xQuadrant == 1 && yQuadrant == 1) {
                positionX[topRight] = RFx;
                positionY[topRight] = RFy;
            } else if (xQuadrant == -1 && yQuadrant == -1) {
                positionX[bottomLeft] = RFx;
                positionY[bottomLeft] = RFy;
            } else if (xQuadrant == 1 && yQuadrant == -1) {
                positionX[bottomRight] = RFx;
                positionY[bottomRight] = RFy;
            }
        }

        float topM = float(positionY[topRight] - positionY[topLeft]) / (positionX[topRight] - positionX[topLeft]);
        float cx = 512;
        float cy = 512;

        if (positionX[topLeft] >= 1023 || positionX[bottomRight] >= 1023 || positionX[bottomLeft] >= 1023 || positionX[topRight] >= 1023) {
            y = -262;
        } else {
            for (int i = 0; i < 4; i++) {
                float px = positionX[i];
                float py = positionY[i];
                float s = sin(atan(-topM));
                float c = cos(atan(-topM));

                px -= cx;
                py -= cy;

                float xnew = px * c - py * s;
                float ynew = px * s + py * c;

                positionX[i] = xnew + cx;
                positionY[i] = ynew + cy;
            }

            if (positionX[topLeft] < 1023 && positionX[topRight] < 1023) {
                xPerc = float(cameraCenter - positionX[topLeft]) / (positionX[topRight] - positionX[topLeft]);
            }
            x = maxX * (1 - xPerc);

            if (positionX[topLeft] < 1023 && positionX[bottomLeft] < 1023) {
                yPerc = float(positionY[topLeft] - cameraCenter) / (positionY[topLeft] - positionY[bottomLeft]);
            }
            y = maxY * (1 - yPerc);
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
        Serial.print(positionX[i]);
        Serial.print("\t,");
        Serial.print(positionY[i]);
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
    Serial.print(positionX[1] - positionX[0]);
    Serial.print(",");
    Serial.print(positionY[0] - positionY[2]);
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
