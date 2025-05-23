/*Sender Test*/
#include <BTstackLib.h>
#include <SPI.h>

/*
   EXAMPLE_START(LECentral): LE Central

   @text Compared with the other examples, the LE Central is
   a bit more complex. This is because it performs multiple
   steps in sequence as it is common with GATT Client APIs.

   It shows how to first scan for other
   devices and then connect to one. When connected, a series of
   GATT Client operations are performed: first the list of
   GATT Services is queried. If a particular service is found,
   the list of its GATT Characteristics is retrieved and a set
   of known Characteristics are cached for later access.
*/

/*
   @section Characteristic Summary
   @text As multiple Characteristics need to be found, a custom
   struct is used to collect all information about it. This allows
   to define the list of necessary characteristics in the
   characteristics[] array
*/
/* LISTING_START(LECentralSummary): Characteristic Summary */

// BLE Shield Service V2 incl. used Characteristics
UUID bleShieldServiceV2UUID("B8E06067-62AD-41BA-9231-206AE80AB551");


typedef struct characteristic_summary {
  UUID         uuid;
  const char * name;
  bool         found;
  BLECharacteristic characteristic;
} characteristic_summary_t;

typedef enum characteristicIDs {
  str,
  hex,
  numCharacteristics  /* last one */
} characteristicIDs_t;

characteristic_summary characteristics[] = {
  { UUID("F897177B-AEE8-4767-8ECC-CC694FD5FCEF"), "STRING", false, BLECharacteristic() },
  { UUID("F897177B-AEE8-4767-8ECC-CC694FD5FCE0"), "HEX", false, BLECharacteristic() }
};

/* LISTING_END(LECentralSummary): Characteristic Summary */

// Application state
BLEDevice  myBLEDevice;
BLEService myBLEService;
bool serviceFound;
bool sendCounter = false;
int xPos;
int yPos;
bool trigger;

int counter = 0;
char counterString[20];

// static btstack_timer_source_t heartbeat;

/*
   @section Setup
   @text In the setup, various callbacks are registered. After that
   we start scanning for other devices
*/
/* LISTING_START(LECentralSetup): LE Central Setup */
void setup(void) {
  Serial.begin(9600);
  
  BTstack.addGATTService(new UUID("B8E06067-62AD-41BA-9231-206AE80AB551"));
  uint16_t c1 = BTstack.addGATTCharacteristicDynamic(new UUID("f897177b-aee8-4767-8ecc-cc694fd5fcef"), ATT_PROPERTY_WRITE, 3); //value_handle=3
  uint16_t c2 = BTstack.addGATTCharacteristicDynamic(new UUID("f897177b-aee8-4767-8ecc-cc694fd5fce0"), ATT_PROPERTY_WRITE, 5); //value_handle=5
  uint16_t c3 = BTstack.addGATTCharacteristicDynamic(new UUID("f897177b-aee8-4767-8ecc-cc694fd5fcee"), ATT_PROPERTY_WRITE, 7); //value_handle=7

  //Callbacks
  BTstack.setBLEAdvertisementCallback(advertisementCallback);
  BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
  BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
  BTstack.setGATTServiceDiscoveredCallback(gattServiceDiscovered);
  BTstack.setGATTCharacteristicDiscoveredCallback(gattCharacteristicDiscovered);
  BTstack.setGATTCharacteristicNotificationCallback(gattCharacteristicNotification);
  BTstack.setGATTCharacteristicIndicationCallback(gattCharacteristicIndication);
  BTstack.setGATTCharacteristicReadCallback(gattReadCallback);
  BTstack.setGATTCharacteristicWrittenCallback(gattWrittenCallback);
  BTstack.setGATTCharacteristicSubscribedCallback(gattSubscribedCallback);
  BTstack.setGATTCharacteristicUnsubscribedCallback(gattUnsubscribeCallback);
  BTstack.setGATTCharacteristicWrite(gattWriteCallback);
  BTstack.setGATTCharacteristicRead(gattReadCallback);
        
  BTstack.setup();
  BTstack.startAdvertising();
}
/* LISTING_END(LECentralSetup): LE Central Setup */

/*
   @section Loop

   @text In the standard Arduino loop() function, BTstack's loop() is called first
   If we're connected, we send the string "BTstack" plus a counter as fast as possible.
   As the Bluetooth module might be busy, it's important to check the result of the
   writeCharacteristicWithoutResponse() call. If it's not ok, we just try again in the
   next loop iteration.
*/
/* LISTING_START(LECentralLoop): Loop */
void loop(void) {
  BTstack.loop();
  
}
/* LISTING_END(LECentralLoop): Loop */

/*
   @section Advertisement Callback

   @text When an Advertisement is received, we check if it contains
   the UUID of the service we're interested in. Only a single service
   with a 128-bit UUID can be contained in and Advertisement and not
   all BLE devices provides this. Other options are to match on the
   reported device name or the BD ADDR prefix.

   If we found an interesting device, we try to connect to it.
*/
/* LISTING_START(LECentralAdvertisementCallback): Advertisement Callback */
void advertisementCallback(BLEAdvertisement *bleAdvertisement) {
  String s = "28:CD:C1:06:8B:FC";
  const char* myCharArray = bleAdvertisement->getBdAddr()->getAddressString();
  String myString(myCharArray);
  if (myString==s) {
    Serial.print("\nBLE ShieldService V2 found:");
    Serial.println(myString);
    BTstack.bleStopScanning();
    BTstack.bleConnect(bleAdvertisement, 10000);  // 10 s
  }
}
/* LISTING_END(LECentralAdvertisementCallback): Advertisement Callback */

/*
   @section Device Connected Callback

   @text At the end of bleConnect(), the device connected callback is callec.
   The status argument tells if the connection timed out, or if the connection
   was established successfully.

   On a successful connection, a GATT Service Discovery is started.
*/
/* LISTING_START(LECentralDeviceConnectedCallback): Device Connected Callback */
void deviceConnectedCallback(BLEStatus status, BLEDevice *device) {
  switch (status) {
    case BLE_STATUS_OK:
      Serial.println("Device connected!");
      myBLEDevice = *device;
      counter = 0;
      myBLEDevice.discoverGATTServices();
      break;
    case BLE_STATUS_CONNECTION_TIMEOUT:
      Serial.println("Error while Connecting the Peripheral");
      BTstack.bleStartScanning();
      break;
    default:
      break;
  }
}
/* LISTING_END(LECentralDeviceConnectedCallback): Device Connected Callback */

/*
   @section Device Disconnected Callback

   @text If the connection to a device breaks, the device disconnected callback
   is called. Here, we start scanning for new devices again.
*/
/* LISTING_START(LECentralDeviceDisconnectedCallback): Device Disconnected Callback */
void deviceDisconnectedCallback(BLEDevice * device) {
  (void) device;
  Serial.println("Disconnected, starting over..");
  sendCounter = false;
  BTstack.bleStartScanning();
}
int gattWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {
  // Temporarily add a null terminator to the buffer
 
  char tempBuffer[size + 1];
  memcpy(tempBuffer, buffer, size);
  tempBuffer[size] = '\0'; // Null-terminate the string
  
  Serial.print("Write value: ");
  for (int i = 0; i < size; i++) {
      Serial.print(buffer[i]);
      Serial.print(" ");
  }
  Serial.print(" for handle(");
  Serial.print(value_handle, HEX);
  Serial.println(")");
  xPos = buffer[0];
  yPos = buffer[1];
  trigger = buffer[2];
  
  return 0;
}

uint16_t gattReadCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {
  // Temporarily add a null terminator to the buffer
 
  char tempBuffer[size + 1];
  memcpy(tempBuffer, buffer, size);
  tempBuffer[size] = '\0'; // Null-terminate the string
  
  Serial.print("Read value: ");
  for (int i = 0; i < size; i++) {
      Serial.print(buffer[i]);
      Serial.print(" ");
  }
  Serial.print(" for handle(");
  Serial.print(value_handle, HEX);
  Serial.println(")");
  xPos = buffer[0];
  yPos = buffer[1];
  trigger = buffer[2];
  
  return 0;
}

/* LISTING_END(LECentralDeviceDisconnectedCallback): Device Disconnected Callback */

/*
   @section Service Discovered Callback

   @text The service discovered callback is called for each service and after the
   service discovery is complete. The status argument is provided for this.

   The main information about a discovered Service is its UUID.
   If we find our service, we store the reference to this service.
   This allows to discover the Characteristics for our service after
   the service discovery is complete.
*/
/* LISTING_START(LECentralServiceDiscoveredCallback): Service Discovered Callback */
void gattServiceDiscovered(BLEStatus status, BLEDevice *device, BLEService *bleService) {
  switch (status) {
    case BLE_STATUS_OK:
      Serial.print("Service Discovered: :");
      Serial.println(bleService->getUUID()->getUuidString());
      if (bleService->matches(&bleShieldServiceV2UUID)) {
        serviceFound = true;
        Serial.println("Our service located!");
        myBLEService = *bleService;
      }
      break;
    case BLE_STATUS_DONE:
      Serial.println("Service discovery finished");
      if (serviceFound) {
        device->discoverCharacteristicsForService(&myBLEService);
      }
      break;
    default:
      Serial.println("Service discovery error");
      break;
  }
}
/* LISTING_END(LECentralServiceDiscoveredCallback): Service Discovered Callback */

/*
   @section Characteristic Discovered Callback

   @text Similar to the Service Discovered callback, the Characteristic Discovered
   callback is called for each Characteristic found and after the discovery is complete.

   The main information is again its UUID. If we find a Characteristic that we're
   interested in, it's name is printed and a reference stored for later.

   On discovery complete, we subscribe to a particular Characteristic to receive
   Characteristic Value updates in the Notificaation Callback.
*/
/* LISTING_START(LECentralCharacteristicDiscoveredCallback): Characteristic Discovered Callback */
void gattCharacteristicDiscovered(BLEStatus status, BLEDevice *device, BLECharacteristic *characteristic) {
  switch (status) {
    case BLE_STATUS_OK:
      Serial.print("Characteristic Discovered: ");
      Serial.print(characteristic->getUUID()->getUuidString());
      Serial.print(", handle 0x");
      Serial.println(characteristic->getCharacteristic()->value_handle, HEX);
      int i;
      for (i = 0; i < numCharacteristics; i++) {
        if (characteristic->matches(&characteristics[i].uuid)) {
          Serial.print("Characteristic found: ");
          Serial.println(characteristics[i].name);
          characteristics[i].found = 1;
          characteristics[i].characteristic = *characteristic;
          break;
        }
      }
      break;
    case BLE_STATUS_DONE:
      Serial.print("Characteristic discovery finished, status ");
      Serial.println(status, HEX);
      if (characteristics[str].found) {
        device->subscribeForNotifications(&characteristics[str].characteristic);
        Serial.print(characteristics[str].name);
        Serial.println(" subscribed!");
      }
      if (characteristics[hex].found) {
        device->subscribeForNotifications(&characteristics[hex].characteristic);
        Serial.print(characteristics[hex].name);
        Serial.println(" subscribed!");
      }
      break;
    default:
      Serial.println("Characteristics discovery error");
      break;
  }
}
/* LISTING_END(LECentralCharacteristicDiscoveredCallback): Characteristic Discovered Callback */

/*
   @section Subscribed Callback

   @text After the subscribe operation is complete, we get notified if it was
   successful. In this example, we read the Characteristic that contains the
   BD ADDR of the other device. This isn't strictly necessary as we already
   know the device address from the Advertisement, but it's a common pattern
   with iOS as the device address is hidden from applications.
*/
/* LISTING_START(LECentralSubscribedCallback): Subscribed Callback */
void gattSubscribedCallback(BLEStatus status, BLEDevice * device) {
  (void) status;
  device->readCharacteristic(&characteristics[str].characteristic);
}
/* LISTING_END(LECentralSubscribedCallback): Subscribed Callback */

/*
   @section Read Callback

   @text The Read callback is called with the result from a read operation.
   Here, we write to the TX Characteristic next.
*/
/* LISTING_START(LECentralReadCallback): Read Callback */
void gattReadCallback(BLEStatus status, BLEDevice *device, uint8_t *value, uint16_t length) {
  (void) status;
  (void) length;
  Serial.print("Read callback: ");
  Serial.println((const char *)value);
  device->writeCharacteristic(&characteristics[str].characteristic, (uint8_t*) "Hello!", 6);
}
/* LISTING_END(LECentralReadCallback): Read Callback */

/*
   @section Written Callback

   @text After the write operation is complete, the Written Callback is callbed with
   the result in the status argument. As we're done with the initial setup of the remote
   device, we set the flag to write the test string as fast as possible.
*/
/* LISTING_START(LECentralWrittenCallback): Written Callback */
void gattWrittenCallback(BLEStatus status, BLEDevice *device) {
  (void) status;
  (void) device;
  sendCounter = true;
  Serial.println("written");
}

void gattUnsubscribeCallback(BLEStatus status, BLEDevice *device) {
  (void) status;
  (void) device;
  sendCounter = true;
  Serial.println("Unsubscribed");
}
/* LISTING_END(LECentralWrittenCallback): Written Callback */

/*
   @section Notification Callback

   @text Notifications for Characteristic Value Updates are delivered via the
   Notification Callback. Whesn more than one Characteristic is subscribed,
   the value handle can be used to distinguish between them. The
   BLECharacteristic.isValueHandle(int handle) allows to test if a value handle
   belongs to a particular Characteristic.
*/
/* LISTING_START(LECentralNotificationCallback): Notification Callback */
void gattCharacteristicNotification(BLEDevice *device, uint16_t value_handle, uint8_t *value, uint16_t length) {
  (void) device;
  (void) value_handle;
  (void) length;
  Serial.print("Notification: ");
  Serial.println((const char *)value);
}
/* LISTING_END(LECentralNotificationCallback): Notification Callback */

void gattCharacteristicIndication(BLEDevice *device, uint16_t value_handle, uint8_t *value, uint16_t length) {
  (void) device;
  (void) value_handle;
  (void) length;
  Serial.print("gattCharacteristicIndication: ");
  Serial.println((const char *)value);
}
