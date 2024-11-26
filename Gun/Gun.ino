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

UUID bleShieldServiceV2UUID("B8E06067-62AD-41BA-9231-206AE80AB551");
typedef struct characteristic_summary {
    UUID         uuid;
    const char * name;
    bool         found;
    uint16_t     handle;
    BLECharacteristic characteristic;
} characteristic_summary_t;

typedef enum characteristicIDs {
    charyPos = 1,
    charxPos = 2,
    chartrigger = 3,
    numCharacteristics  /* last one */
} characteristicIDs_t;

characteristic_summary characteristics[] = {
    { UUID("F897177B-AEE8-4767-8ECC-CC694FD5FCEF"), "yPos"       },
    { UUID("F897177B-AEE8-4767-8ECC-CC694FD5FCE0"), "xPos"       },
    { UUID("F897177B-AEE8-4767-8ECC-CC694FD5FCEE"), "trigger" }
};

int yPos;
int xPos;
bool trigger;

// Application state
BLEDevice  myBLEDevice;
BLEService myBLEService;
bool serviceFound;
bool sendCounter = false;

int counter = 0;
char counterString[20];

void setup(void) {

  Serial.begin(9600);

  // set callbacks
  BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
  BTstack.setGATTCharacteristicRead(gattReadCallback);
  BTstack.setGATTCharacteristicWrite(gattWriteCallback);
  BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
  BTstack.setGATTServiceDiscoveredCallback(gattServiceDiscovered);
  BTstack.setGATTCharacteristicDiscoveredCallback(gattCharacteristicDiscovered);
  BTstack.setGATTCharacteristicNotificationCallback(gattCharacteristicNotification);
  BTstack.setGATTCharacteristicReadCallback(gattReadCallback);
  BTstack.setGATTCharacteristicWrittenCallback(gattWrittenCallback);
  BTstack.setGATTCharacteristicSubscribedCallback(gattSubscribedCallback);

  // startup Bluetooth and activate advertisements
  BTstack.setup();
  BTstack.startAdvertising();

  //Setup IR Camera
  Wire.setSDA(16);
  Wire.setSCL(17);
  Wire1.begin();
  myDFRobotIRPosition.begin();
}
/* LISTING_END(LEPeripheralSetup): Setup */

void loop(void) {
  BTstack.loop();
  if(conn == CONNECTED){
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
      int oldXPos = xPos;
      int oldYPos = yPos;
      int oldTrigger = trigger;

      triggerCheck();
      xPos = cameraCoord.x;
      yPos = cameraCoord.y;
      trigger * 1000000+xPos*100000+xPos;
      
      //if(oldYPos!=yPos){
        
        myBLEDevice.writeCharacteristic(&characteristics[charyPos].characteristic, (uint8_t*) "Hello!", sizeof((uint8_t*) "Hello!"));
      //}



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

/* LISTING_END(LEPeripheralDeviceConnectedCallback): Device Connected Callback */

/*
   @section Device Disconnected Callback

   @text If the connection to a device breaks, the device disconnected callback
   is called.
*/
/* LISTING_START(LEPeripheralDeviceDisconnectedCallback): Device Disconnected Callback */
void deviceDisconnectedCallback(BLEDevice * device) {
  (void) device;
  Serial.println("Disconnected.");
}
/* LISTING_END(LEPeripheralDeviceDisconnectedCallback): Device Disconnected Callback */

/*
   @section Read Callback

   @text In BTstack, the Read Callback is first called to query the size of the
   Characteristic Value, before it is called to provide the data.
   Both times, the size has to be returned. The data is only stored in the provided
   buffer, if the buffer argeument is not NULL.
   If more than one dynamic Characteristics is used, the value handle is used
   to distinguish them.
*/
/* LISTING_START(LEPeripheralReadCallback): Read Callback */
uint16_t gattReadCallback(uint16_t value_handle, uint8_t * buffer, uint16_t buffer_size) {
  (void) value_handle;
  (void) buffer_size;
  if (buffer) {
    Serial.print("gattReadCallback, value: ");
    //Serial.println(characteristic_data, HEX);
    //buffer[0] = characteristic_data;
  }
  return 1;
}
/* LISTING_END(LEPeripheralDeviceDisconnectedCallback): Read Callback */

/*
   @section Write Callback

   @text When the remove device writes a Characteristic Value, the Write callback
   is called. The buffer arguments points to the data of size size/
   If more than one dynamic Characteristics is used, the value handle is used
   to distinguish them.
*/
/* LISTING_START(LEPeripheralWriteCallback): Write Callback */
int gattWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {
  (void) value_handle;
  (void) size;
  //characteristic_data = buffer[0];
  Serial.print("gattWriteCallback , value ");
 // Serial.println(characteristic_data, HEX);
  return 0;
}
/* LISTING_END(LEPeripheralWriteCallback): Write Callback */

/* LISTING_END(LECentralAdvertisementCallback): Advertisement Callback */

/*
 * @section Device Connected Callback
 *
 * @text At the end of bleConnect(), the device connected callback is callec.
 * The status argument tells if the connection timed out, or if the connection
 * was established successfully.
 *
 * On a successful connection, a GATT Service Discovery is started.
 */
/* LISTING_START(LECentralDeviceConnectedCallback): Device Connected Callback */
void deviceConnectedCallback(BLEStatus status, BLEDevice *device) {
    switch (status){
        case BLE_STATUS_OK:
            Serial.println("Device connected!");
            myBLEDevice = *device;
            counter = 0;
            myBLEDevice.discoverGATTServices();
            break;
        case BLE_STATUS_CONNECTION_TIMEOUT:
            Serial.println("Error while Connecting the Peripheral");
            break;
        default:
            break;
    }
}

/* LISTING_END(LECentralDeviceDisconnectedCallback): Device Disconnected Callback */

/*
 * @section Service Discovered Callback
 *
 * @text The service discovered callback is called for each service and after the 
 * service discovery is complete. The status argument is provided for this.
 *
 * The main information about a discovered Service is its UUID.
 * If we find our service, we store the reference to this service.
 * This allows to discover the Characteristics for our service after 
 * the service discovery is complete.
 */
/* LISTING_START(LECentralServiceDiscoveredCallback): Service Discovered Callback */
void gattServiceDiscovered(BLEStatus status, BLEDevice *device, BLEService *bleService) {
    switch(status){
        case BLE_STATUS_OK:
            if (bleService->matches(&bleShieldServiceV2UUID)) {
                serviceFound = true;
                Serial.print("Our Service Discovered: ");
                Serial.println(bleService->getUUID()->getUuidString());
                myBLEService = *bleService;
            }
            break;
        case BLE_STATUS_DONE:
            if (serviceFound) {
                device->discoverCharacteristicsForService(&myBLEService);
                conn = CONNECTED;
            }
            break;
        default:
            Serial.println("Service discovery error");
            break;
    }
}
/* LISTING_END(LECentralServiceDiscoveredCallback): Service Discovered Callback */

/*
 * @section Characteristic Discovered Callback
 *
 * @text Similar to the Service Discovered callback, the Characteristic Discovered
 * callback is called for each Characteristic found and after the discovery is complete.
 * 
 * The main information is again its UUID. If we find a Characteristic that we're 
 * interested in, it's name is printed and a reference stored for later.
 * 
 * On discovery complete, we subscribe to a particular Characteristic to receive 
 * Characteristic Value updates in the Notificaation Callback.
 */
/* LISTING_START(LECentralCharacteristicDiscoveredCallback): Characteristic Discovered Callback */
void gattCharacteristicDiscovered(BLEStatus status, BLEDevice *device, BLECharacteristic *characteristic) {
    switch(status){
        case BLE_STATUS_OK:
            int i;
            for (i=0;i<numCharacteristics;i++){
                if (characteristic->matches(&characteristics[i].uuid)){
                    Serial.print("Characteristic (");
                    Serial.print(characteristics[i].name);
                    Serial.print(") Discovered: ");
                    Serial.print(characteristic->getUUID()->getUuidString());
                    Serial.print(", handle 0x");
                    Serial.println(characteristic->getCharacteristic()->value_handle, HEX);
                    characteristics[i].found = 1;
                    characteristics[i].characteristic = *characteristic;
                    characteristics[i].handle = characteristic->getCharacteristic()->value_handle;
                    break;
                }
            }
            break;
        case BLE_STATUS_DONE:
            break;
        default:
            Serial.println("Characteristics discovery error");
            break;
    }
}
/* LISTING_END(LECentralCharacteristicDiscoveredCallback): Characteristic Discovered Callback */

/*
 * @section Subscribed Callback
 *
 * @text After the subcribe operation is complete, we get notified if it was 
 * successful. In this example, we read the Characteristic that contains the 
 * BD ADDR of the other device. This isn't strictly neccessary as we already
 * know the device address from the Advertisement, but it's a common pattern
 * with iOS as the device address is hidden from applications.
 */
/* LISTING_START(LECentralSubscribedCallback): Subscribed Callback */
void gattSubscribedCallback(BLEStatus status, BLEDevice * device){
    //device->readCharacteristic(&characteristics[charBdAddr].characteristic);
}
/* LISTING_END(LECentralSubscribedCallback): Subscribed Callback */

/*
 * @section Read Callback
 *
 * @text The Read callback is called with the result from a read operation.
 * Here, we write to the TX Characteristic next.
 */
/* LISTING_START(LECentralReadCallback): Read Callback */
void gattReadCallback(BLEStatus status, BLEDevice *device, uint8_t *value, uint16_t length) {
    Serial.print("Read callback: ");
    Serial.println((const char *)value);
    //device->writeCharacteristic(&characteristics[charTX].characteristic, (uint8_t*) "Hello!", 6);
}
/* LISTING_END(LECentralReadCallback): Read Callback */

/*
 * @section Written Callback
 *
 * @text After the write operation is complete, the Written Callback is callbed with
 * the result in the status argument. As we're done with the initial setup of the remote
 * device, we set the flag to write the test string as fast as possible.
 */
/* LISTING_START(LECentralWrittenCallback): Written Callback */
void gattWrittenCallback(BLEStatus status, BLEDevice *device){
    sendCounter = true;
}
/* LISTING_END(LECentralWrittenCallback): Written Callback */

/*
 * @section Notification Callback
 *
 * @text Notifictions for Characteristic Value Updates are delivered via the 
 * Notification Callback. When more than one Characteristic is subscribed,
 * the value handle can be used to distinguish between them. The 
 * BLECharacteristic.isValueHandle(int handle) allows to test if a value handle
 * belongs to a particular Characteristic.
 */
/* LISTING_START(LECentralNotificationCallback): Notification Callback */
void gattCharacteristicNotification(BLEDevice *device, uint16_t value_handle, uint8_t *value, uint16_t length) {
    Serial.print("Notification: ");
    Serial.println((const char *)value);
}
/* LISTING_END(LECentralNotificationCallback): Notification Callback */


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
