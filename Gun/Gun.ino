/**
 * @file gun.ino
 * @author James Michael Stephens
 * @date 11/18/2024
 *
 * @brief Reads position of infrared lights and gyro values to determine where the camera is pointing in relation to the screen. Handles reporting the trigger button when pressed.
 */

// Define constants and macros
#define BAUD_RATE 9600
#define CONNECTED 1
#define DISCONNECTED 0
#define upperLeft 2
#define upperRight 3
#define lowerLeft 0
#define lowerRight 1

// Define pin assignments
#define Trigger_PIN 2

// Include libraries
#include <BTstackLib.h>
#include <SPI.h>
#include <Wire.h>
#include <DFRobotIRPosition.h>
#include <math.h>

// Global variables=
static int yPos = 1;
static int xPos = 2;
static int trigger = 3;
static int conn = DISCONNECTED;
DFRobotIRPosition myDFRobotIRPosition;         // declare a IRCam object
int positionX[4];               // RAW Sensor Values
int positionY[4];     
struct coord {
  float x;
  float y;
};
coord coords[4];
coord tCoords[4];
coord defaultCameraCoord = {512,384};
coord cameraCoord = {};
int posCounter;
float dst[4][2] = {{0, -100}, {100, -100}, {0, 0}, {100, 0}};
float matrix[3][3] = {0};  

void setup(void) {

  Serial.begin(BAUD_RATE);
  
  // set callbacks
  BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
  BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
  BTstack.setGATTCharacteristicRead(gattReadCallback);
  BTstack.setGATTCharacteristicWrite(gattWriteCallback);
  BTstack.setGATTCharacteristicWrittenCallback(gattWrittenCallback);
  BTstack.setGATTCharacteristicNotificationCallback(gattNotificationCallback);
  BTstack.setGATTCharacteristicSubscribedCallback(gattSubscribedCallback);
  BTstack.setGATTCharacteristicDiscoveredCallback(gattDiscoveredCallback);
  BTstack.setGATTCharacteristicIndicationCallback(gattIndicationCallback);
  BTstack.setGATTServiceDiscoveredCallback(gattServiceCallback);
  

  //BTstack.setGATTDoneCallback(cb);
  
  // setup GATT Database
  BTstack.addGATTService(new UUID("B8E06067-62AD-41BA-9231-206AE80AB551"));
  BTstack.addGATTCharacteristicDynamic(new UUID("f897177b-aee8-4767-8ecc-cc694fd5fcef"), ATT_PROPERTY_READ | ATT_PROPERTY_NOTIFY, 3); //value_handle=3
  BTstack.addGATTCharacteristicDynamic(new UUID("f897177b-aee8-4767-8ecc-cc694fd5fce0"), ATT_PROPERTY_READ | ATT_PROPERTY_NOTIFY, 5); //value_handle=5
  BTstack.addGATTCharacteristicDynamic(new UUID("f897177b-aee8-4767-8ecc-cc694fd5fcee"), ATT_PROPERTY_READ | ATT_PROPERTY_NOTIFY, 7); //value_handle=7
  bd_addr_t addr= {'0','0','0','0','0','1'};
  
  BTstack.setPublicBdAddr(addr);
  // startup Bluetooth and activate advertisements
  BTstack.setup();
  BTstack.startAdvertising();

  //Setup IR Camera
  Wire.setSDA(16);
  Wire.setSCL(17);
  Wire1.begin();
  myDFRobotIRPosition.begin();
  

}

void loop(void) {
  BTstack.loop();

  if(conn == CONNECTED){
    triggerCheck();
    //ebug();
      calculatePerspectiveTransform(coords, dst, matrix);

      // Print the transformation matrix
      //Serial.println("Transformation Matrix:");
      for (int i = 0; i < 4; i++) {
        float point[2] = {coords[i].x,coords[i].y};
        float transformedPoint[2] = {0};
        applyPerspectiveTransform(matrix, point, transformedPoint);
        tCoords[i].x = transformedPoint[0];
        tCoords[i].y = transformedPoint[1];
      }
      float point[2] = {defaultCameraCoord.x,defaultCameraCoord.y};
      float transformedPoint[2] = {0};
      applyPerspectiveTransform(matrix, point, transformedPoint);
      cameraCoord.x = transformedPoint[0];
      cameraCoord.y = transformedPoint[1];
      xPos = cameraCoord.x;
      yPos = cameraCoord.y;

      // Apply the transformation to a point
      //debug();
  }
}

void triggerCheck(){
  if (digitalRead(Trigger_PIN) == HIGH) {
    trigger = 1;
  }else{
    trigger = 0;
  }
}


void gattWrittenCallback(BLEStatus status,BLEDevice *device) {
 
  Serial.println("gattWritten");
}

int gattWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {
  (void) size;
  yPos = buffer[0];
  Serial.print("gattWriteCallback , value ");
  Serial.println(buffer[0], HEX);
  Serial.println(value_handle,HEX);
  return 0;
}

//void (*)(BLEStatus, BLEDevice *, BLEService *)
void gattServiceCallback(BLEStatus status,BLEDevice *device, BLEService * service) {

  Serial.println("Service: ");
}

//void (*)(BLEDevice *, uint16_t, uint8_t *, uint16_t)
void gattIndicationCallback(BLEDevice *device, uint16_t value_handle, uint8_t *value, uint16_t length) {
 (void) device;
  (void) value_handle;
  (void) length;
  Serial.print("Indication: ");
  Serial.println((const char *)value);
}


void cb(BLEStatus status,BLEDevice * device) {
  (void) status;
  Serial.println("Done!");
}

void gattDiscoveredCallback(BLEStatus status,BLEDevice * device, BLECharacteristic * characteristic) {
  (void) status;
  (void) characteristic;
  Serial.println("Discovered!");
}

void gattSubscribedCallback(BLEStatus status, BLEDevice * device) {
  (void) status;
  Serial.println("Subscribed!");
}

//void (*)(BLEDevice *, uint16_t, uint8_t *, uint16_t)
void gattNotificationCallback(BLEDevice *device, uint16_t value_handle, uint8_t *value, uint16_t length) {
 (void) device;
  (void) value_handle;
  (void) length;
  Serial.print("Notification: ");
  Serial.println((const char *)value);
}


/*
   @section Device Connected Callback

   @text When a remove device connects, device connected callback is callec.
*/
/* LISTING_START(LEPeripheralDeviceConnectedCallback): Device Connected Callback */
void deviceConnectedCallback(BLEStatus status, BLEDevice *device) {
  
  switch (status) {
    case BLE_STATUS_OK:
      conn = CONNECTED;
      Serial.println(device->getHandle());
      
      Serial.println(device->discoverGATTServices());
      break;
    default:
      break;
  }
}

/*
   @section Device Disconnected Callback

   @text If the connection to a device breaks, the device disconnected callback
   is called.
*/
void deviceDisconnectedCallback(BLEDevice * device) {
  (void) device;
  conn = DISCONNECTED;
}

/*
   @section Read Callback

   @text In BTstack, the Read Callback is first called to query the size of the
   Characteristic Value, before it is called to provide the data.
   Both times, the size has to be returned. The data is only stored in the provided
   buffer, if the buffer argeument is not NULL.
   If more than one dynamic Characteristics is used, the value handle is used
   to distinguish them.
*/
uint16_t gattReadCallback(uint16_t value_handle, uint8_t * buffer, uint16_t buffer_size) {
  (void) value_handle;
  (void) buffer_size;
  if (buffer) {
    Serial.print(value_handle);
    Serial.print("gattReadCallback, value: ");
    if(value_handle == 3){
      Serial.println(yPos, HEX);
      buffer[0] = yPos;
    }else if(value_handle == 5){
      Serial.println(xPos, HEX);
      buffer[0] = xPos;
    }else if(value_handle == 7){
      Serial.println(trigger, HEX);
      buffer[0] = trigger;
    }else {
      unsigned long test = 0x999b989;
      Serial.print(buffer[0]);
      buffer[0] = test;
    }
    Serial.println();
  }
  return 1;
}


/*
   @section Determine IR LED positioning

   @text 
*/

void setPosition() { 
  myDFRobotIRPosition.requestPosition();
  if (myDFRobotIRPosition.available()) {
    posCounter = 0;
    for (int i = 0; i < 4; i++) {
      int RFx = myDFRobotIRPosition.readX(i);
      int RFy = myDFRobotIRPosition.readY(i);
      if(RFx<1023){
        posCounter++;
            /**This logic is definitely flawed.**/
            int xQuadrant = (RFx > 512) ? 1 : -1;
            int yQuadrant = (RFy > 384) ? 1 : -1;
            /*Serial.print(RFx);
            Serial.print(",");
            Serial.print(RFy);
            Serial.print(",");
            Serial.print(xQuadrant);
            Serial.print(",");
            Serial.println(yQuadrant);
            */
            if (xQuadrant == -1 && yQuadrant == 1) {
                coords[upperLeft].x = RFx;
                coords[upperLeft].y = RFy;
            } else if (xQuadrant == 1 && yQuadrant == 1) {
                coords[upperRight].x = RFx;
                coords[upperRight].y = RFy;
            } else if (xQuadrant == -1 && yQuadrant == -1) {
                coords[lowerLeft].x = RFx;
                coords[lowerLeft].y = RFy;
            } else if (xQuadrant == 1 && yQuadrant == -1) {
                coords[lowerRight].x = RFx;
                coords[lowerRight].y = RFy;
            }
      }else{
        coords[4] = {};
        break;
      }
    }
  }
}

/*
   @section Create linear transformation matrix.

   @text This should fix the rectangle skewing and simplifying position calculation.
*/

void calculatePerspectiveTransform(coord src[4], float dst[4][2], float matrix[3][3]) {
    // Variables for the transformation matrix calculation
    float a[8][8] = {0};
    float b[8] = {0};
    float x[8] = {0};

    // Set up matrix equations
    for (int i = 0; i < 4; i++) {
        a[i * 2][0] = src[i].x;
        a[i * 2][1] = src[i].y;
        a[i * 2][2] = 1;
        a[i * 2][6] = -src[i].x * dst[i][0];
        a[i * 2][7] = -src[i].y * dst[i][0];
        b[i * 2] = dst[i][0];

        a[i * 2 + 1][3] = src[i].x;
        a[i * 2 + 1][4] = src[i].y;
        a[i * 2 + 1][5] = 1;
        a[i * 2 + 1][6] = -src[i].x * dst[i][1];
        a[i * 2 + 1][7] = -src[i].y * dst[i][1];
        b[i * 2 + 1] = dst[i][1];
    }

    // Solve the system of equations using Gaussian elimination
    gaussianElimination(a, b, x);

    // Fill the perspective transformation matrix
    matrix[0][0] = x[0];
    matrix[0][1] = x[1];
    matrix[0][2] = x[2];
    matrix[1][0] = x[3];
    matrix[1][1] = x[4];
    matrix[1][2] = x[5];
    matrix[2][0] = x[6];
    matrix[2][1] = x[7];
    matrix[2][2] = 1.0;
}

/*
   @section Perform a set of linear equations.

   @text Gaussian elimination is a method for solving systems of linear equations by transforming the system into upper triangular form through elementary row operations. This technique is widely used in linear algebra and numerical analysis.
*/

void gaussianElimination(float a[8][8], float b[8], float x[8]) {
    int n = 8;
    for (int i = 0; i < n; i++) {
        // Partial pivoting
        for (int k = i + 1; k < n; k++) {
            if (fabs(a[i][i]) < fabs(a[k][i])) {
                for (int j = 0; j < n; j++) {
                    float temp = a[i][j];
                    a[i][j] = a[k][j];
                    a[k][j] = temp;
                }
                float temp = b[i];
                b[i] = b[k];
                b[k] = temp;
            }
        }

        // Elimination process
        for (int k = i + 1; k < n; k++) {
            float t = a[k][i] / a[i][i];
            for (int j = 0; j < n; j++) {
                a[k][j] -= t * a[i][j];
            }
            b[k] -= t * b[i];
        }
    }

    // Back substitution
    for (int i = n - 1; i >= 0; i--) {
        x[i] = b[i];
        for (int j = i + 1; j < n; j++) {
            x[i] -= a[i][j] * x[j];
        }
        x[i] = x[i] / a[i][i];
    }
}

/*
   @text Apply linear transformation marix to coordinates.
*/

void applyPerspectiveTransform(float matrix[3][3], float src[2], float dst[2]) {
    float x = src[0];
    float y = src[1];

    float w = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2];
    dst[0] = (matrix[0][0] * x + matrix[0][1] * y + matrix[0][2]) / w;
    dst[1] = (matrix[1][0] * x + matrix[1][1] * y + matrix[1][2]) / w;
}

/*
   @text Renders debugging information in Serial Monitor
*/

void debug(){
  Serial.print("IRcount:");
  Serial.print(posCounter);
  Serial.print("\tCoordinates:");
  Serial.print("(");
  Serial.print(cameraCoord.x);
  Serial.print(",");
  Serial.print(cameraCoord.y);
  Serial.print(")");
  Serial.print("\tFinalCoord:");
  Serial.print("(");
  Serial.print(cameraCoord.x/100*48);
  Serial.print(",");
  Serial.print(cameraCoord.y/-100*262);
  Serial.print(")");
  Serial.print("\tpos:");
  for (int i = 0; i < 4; i++) {
    Serial.print("(");
    Serial.print(coords[i].x);
    Serial.print(",");
    Serial.print(coords[i].y);
    Serial.print(")");
  }
  Serial.print("\ttpos:");
  for (int i = 0; i < 4; i++) {
    Serial.print("(");
    Serial.print(tCoords[i].x);
    Serial.print(",");
    Serial.print(tCoords[i].y);
    Serial.print(")");
  }
  Serial.println();
}


