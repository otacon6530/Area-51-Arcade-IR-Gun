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
	void OnMouseMove	(MOUSEINFO *mi);
	void OnLeftButtonUp	(MOUSEINFO *mi);
	void OnLeftButtonDown	(MOUSEINFO *mi);
	void OnRightButtonUp	(MOUSEINFO *mi);
	void OnRightButtonDown	(MOUSEINFO *mi);
	void OnMiddleButtonUp	(MOUSEINFO *mi);
	void OnMiddleButtonDown	(MOUSEINFO *mi);
};

#include "DFRobotIRPosition.h"

DFRobotIRPosition myDFRobotIRPosition;

int positionX[4];     ///< Store the X position
int positionY[4];     ///< Store the Y position
int cameraCenter = 512;
float xPerc = 0;
float yPerc = 0;

int maxX = 47;
int maxY = 239;
int x = 25;
int y = 135;
const int digitalPin = 7;  // Analog output pin that the LED is attached to
const int firePin = 6;  // Analog output pin that the LED is attached to
int counter = 0;

const byte vSyncPin = 3;
const byte hSyncPin = 2;

void printResult();
void MouseRptParser::OnMouseMove(MOUSEINFO *mi)
{

    int dX = mi->dX;
    if(dX<-3){
      dX=-1;
    }
    else if(dX>3){
      dX=1;
    }else{
      dX=0;
    }

    int dY = mi->dY;
    if(dY<-1){
      dY=-4;
    }
    else if(dY>1){
      dY=4;
    }else{
      dY=0;
    }
    x = x + dX;
    y = y + dY;
    if(y>262){
      //y -= 262;
      y=263;
    }else if(y<0){
      //y += 262;
      y=0;
    }
    if(x>48){
      x = 49;
    }else if(x<4){
      x = 3;
    }
   
};
void MouseRptParser::OnLeftButtonUp	(MOUSEINFO *mi){};
void MouseRptParser::OnLeftButtonDown	(MOUSEINFO *mi)
{
  Serial.println("L Butt Dn");
  pinMode(firePin,OUTPUT);
  digitalWrite(firePin, LOW);
  delay(50);
  pinMode(firePin,INPUT);
};
void MouseRptParser::OnRightButtonUp	(MOUSEINFO *mi){};
void MouseRptParser::OnRightButtonDown	(MOUSEINFO *mi)
{
    int tempX = x;
  int tempY = y;
  x = 0;
  y = 0;
  delay(500);
  pinMode(firePin,OUTPUT);
  digitalWrite(firePin, LOW);
  delay(17);
  pinMode(firePin,INPUT);
  delay(100);
  x = tempX;
  y = tempY;
};
void MouseRptParser::OnMiddleButtonUp	(MOUSEINFO *mi){};
void MouseRptParser::OnMiddleButtonDown	(MOUSEINFO *mi){};

USB     Usb;
USBHub     Hub(&Usb);
HIDBoot<USB_HID_PROTOCOL_MOUSE>    HidMouse(&Usb);

MouseRptParser                               Prs;

void setup()
{
  Serial.begin(115200);
  Serial.println("Start");
  
  myDFRobotIRPosition.begin();

  pinMode(vSyncPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(vSyncPin), vSync, FALLING);
  
  pinMode(hSyncPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hSyncPin), hSync, RISING);
    if (Usb.Init() == -1);
    delay( 200 );
    HidMouse.SetReportParser(0, &Prs);
}

void vSync(){
  counter = 240;
}

void hSync(){
    counter += 1;
    if(counter==y){ //19 to 239
      delayMicroseconds(x); //4 to 47 range
      pinMode(digitalPin,OUTPUT);
      digitalWrite(digitalPin, LOW);
      pinMode(digitalPin,INPUT);
    }
    if(counter>=262){
      counter = 0;
    }
}
void test(){
  delay(10);
    y++;
    if(x==48){
      x=0;
    }
    if(y>=262){
        y=0;
        x++;
    }
    //x++;
    //if(x==48){
    //  y++;
    //  x=0;
    //}
    //if(y>=262){
    //    y=0;
    //}
}

void IRPosition(){
  /*!
   *  @brief request the position
   */
  myDFRobotIRPosition.requestPosition();
  
  /*!
   *  @brief If there is data available, print it. Otherwise show the error message.
   */
  if (myDFRobotIRPosition.available()) {
    for (int i=0; i<4; i++) {

      int RFx=myDFRobotIRPosition.readX(i);
      int RFy=myDFRobotIRPosition.readY(i);

      if(RFx>=1023 or RFy>=1023){
        positionX[topLeft]=0;
      }

      //determine the corners for the sensors.
      int xQuadrant = 0; 
      int yQuadrant = 0;
      if(RFx>512){
        xQuadrant = 1;
      }else{
        xQuadrant = -1;
      }
      if(RFy>512){
        yQuadrant = 1;
      }else{
        yQuadrant = -1;
      }
    
      if(xQuadrant == -1 and yQuadrant == 1){
          positionX[topLeft] = RFx;
          positionY[topLeft] = RFy;
        }else if(xQuadrant == 1 and yQuadrant == 1){
          positionX[topRight] = RFx;
          positionY[topRight] = RFy;
        }else if(xQuadrant == -1 and yQuadrant == -1){
          positionX[bottomLeft] = RFx; 
          positionY[bottomLeft] = RFy;  
        }else if(xQuadrant == 1 and yQuadrant == -1){
          positionX[bottomRight] = RFx;  
          positionY[bottomRight] = RFy;   
        }
    }
    //printResult(0,0);
        //Get slope between two top corners
        double topM=(positionY[topRight] - positionY[topLeft]) / (positionX[topRight] - positionX[topLeft]);
        
        //rotation correction. Assuming top of rectable should have a slope of 0.
        //for (int i=0; i<4; i++) {
        //  double angle = atan(-topM); //convert slope to radians and reverse to get to 0.
        //   //Rotate clockwise, angle in radians
        //  int xPoint = round((cos(angle) * (positionX[i] - cameraCenter)) -
        //                    (sin(angle) * (positionY[i] - cameraCenter)) +
        //                    cameraCenter);
        //  int yPoint = round((sin(angle) * (positionX[i] - cameraCenter)) +
        //                    (cos(angle) * (positionY[i] - cameraCenter)) +
        //                    cameraCenter);
        //  positionX[i]=xPoint;
        //  positionY[i]=yPoint;
        //}
        //calculate percentage from left of camera.
        if(positionX[topLeft]<1023 and positionX[topRight]<1023){
          xPerc = (float)(cameraCenter-positionX[topLeft])/(positionX[topRight]-positionX[topLeft]);
        }
        //Apply percentage to new resolution for x.
        x = maxX*(1-xPerc);
        //calculate percentage from top of camera
        if(positionX[topLeft]<1023 and positionX[bottomLeft]<1023){
          yPerc = (float)(positionY[topLeft]-cameraCenter)/(positionY[topLeft]-positionY[bottomLeft]);
        }
        //Apply percentage to new resolution for x.
        y = maxY*(1-yPerc); 
        printResult(xPerc,yPerc);

      
  }
  else{
    Serial.println("Device not available!");
  }
  

}

void loop()
{
  switch(state)
    {
    case MOUSE:
        Usb.Task();
        break;

    case DEBUG:
        test();
        break;
    default:
        IRPosition();
    }

}

void printResult(float xPerc,float yPerc)
{
    
      Serial.print("IR Positions:\t");
      for (int i=0; i<4; i++) {
        Serial.print(positionX[i]);
        Serial.print("\t,");
        
        Serial.print(positionY[i]);
        Serial.print(";\t");
      }
      Serial.print("Resolution: ");
      Serial.print(positionX[1]-positionX[0]);
      Serial.print(",");
      Serial.print(positionY[0]-positionY[2]);
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
