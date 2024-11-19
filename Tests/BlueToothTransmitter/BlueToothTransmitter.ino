// LE Peripheral Example - not working yet
#include <BTstackLib.h>
#include <SPI.h>
/*
   EXAMPLE_START(LEPeripheral): LE Peripheral

   @text BTstack allows to setup a GATT Services and Characteristics directly
   from the setup function without using other tools outside of the Arduino IDE.

   @section Setup

   @text First, a number of callbacks are set. Then, a Service with a Read-only
   Characteristic and a dynamic Characteristic is added to the GATT database.
   In BTstack, a dynamic Characteristic is a Characteristic where reads and writes
   are forwarded to the Sketch. In this example, the dynamic Characteristic is
   provided by the single byte variable yPos.
*/

/* LISTING_START(LEPeripheralSetup): Setup */
static char yPos = 'H';
static char xPos = 'H';

void setup(void) {

  Serial.begin(9600);
  
  // set callbacks
  BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
  BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
  BTstack.setGATTCharacteristicRead(gattReadCallback);
  BTstack.setGATTCharacteristicWrite(gattWriteCallback);
  
  // setup GATT Database
  BTstack.addGATTService(new UUID("B8E06067-62AD-41BA-9231-206AE80AB551"));
  BTstack.addGATTCharacteristicDynamic(new UUID("f897177b-aee8-4767-8ecc-cc694fd5fcef"), ATT_PROPERTY_READ | ATT_PROPERTY_WRITE, 0); //value_handle=3
  BTstack.addGATTCharacteristicDynamic(new UUID("f897177b-aee8-4767-8ecc-cc694fd5fce0"), ATT_PROPERTY_READ | ATT_PROPERTY_WRITE, 1); //value_handle=5

  // startup Bluetooth and activate advertisements
  BTstack.setup();
  BTstack.startAdvertising();
}
/* LISTING_END(LEPeripheralSetup): Setup */

void loop(void) {
  BTstack.loop();
  delay(10000);
  
  uint16_t handle = 3;            // Example handle
  uint8_t dataBuffer[] = {1};       // Example data buffer
  uint16_t bufferSize = sizeof(dataBuffer); // Size of the data buffer

  // Call the function
  int result = gattWriteCallback(handle, dataBuffer, bufferSize);

  handle = 5;
  uint8_t dataBuffer3[] = {3};
  result = gattWriteCallback(handle, dataBuffer3, bufferSize);


  delay(10000);
  uint16_t handle2 = 3; 
  uint8_t dataBuffer2[] = {2};       // Example data buffer
  bufferSize = sizeof(dataBuffer); // Size of the data buffer

  // Call the function
  result = gattWriteCallback(handle2, dataBuffer2, bufferSize);

  handle = 5;
  uint8_t dataBuffer4[] = {4};
  result = gattWriteCallback(handle, dataBuffer4, bufferSize);
}

/*
   @section Device Connected Callback

   @text When a remove device connects, device connected callback is callec.
*/
/* LISTING_START(LEPeripheralDeviceConnectedCallback): Device Connected Callback */
void deviceConnectedCallback(BLEStatus status, BLEDevice *device) {
  (void) device;
  switch (status) {
    case BLE_STATUS_OK:
      Serial.println("Device connected!");
      break;
    default:
      break;
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
    Serial.print(value_handle);
    Serial.print("gattReadCallback, value: ");
    if(value_handle == 3){
      Serial.println(yPos, HEX);
      buffer[0] = yPos;
    }else{
      Serial.println(xPos, HEX);
      buffer[0] = xPos;
    }
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
  Serial.print("gattWriteCallback , value ");
  if(value_handle == 3){
      yPos = buffer[0];
      Serial.println(yPos, HEX);
    }else{
      xPos = buffer[0];
      Serial.println(xPos, HEX);
    }
  return 0;
}
/* LISTING_END(LEPeripheralWriteCallback): Write Callback */

