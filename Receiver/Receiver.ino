#include <BTstackLib.h>
#include <SPI.h>
#define DEBUG 0  // Change to 0 to disable debug

// Application state
int xPos;
int yPos;
bool trigger;
const int digitalPin = 7;
const int firePin = 6;  // Digital output pin for the fire action
struct {

  int width = 47;
  int height = 239;
  int x = 25;
  int y = 125;
  int vSyncOffset = 240;
  int hSyncCounter = 0;
} screen;
int lastButtonState;
const byte tempTrig = 4;
const byte vSyncPin = 3;
const byte hSyncPin = 2;

// static btstack_timer_source_t heartbeat;

/*
   @section Setup
   @text In the setup, various callbacks are registered. After that
   we start scanning for other devices
*/
/* LISTING_START(LECentralSetup): LE Central Setup */
void setup(void) {
  Serial.begin(9600);
  uint16_t c2 = BTstack.addGATTCharacteristicDynamic(new UUID("f897177b-aee8-4767-8ecc-cc694fd5fcef"), ATT_PROPERTY_WRITE, 3);  //value_handle=3
  BTstack.enableDebugLogger();
  BTstack.enablePacketLogger();
  
  static const char* MY_SERVICE_UUID_STR = "b48e423a-a7ac-4d81-af24-8c4accdcf550";
  UUID myServiceUUID(MY_SERVICE_UUID_STR);
  BTstack.addGATTService(&myServiceUUID);
  UUID charUUID("61d292f5-e8bd-476a-82d7-d59eb5653241");
  uint16_t c1 = BTstack.addGATTCharacteristicDynamic(&charUUID, ATT_PROPERTY_WRITE, 3);
  BTstack.setBLEAdvertisementCallback(advertisementCallback);
  BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
  BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
  BTstack.setGATTCharacteristicWrite(gattWriteCallback);

  BTstack.setup();
  Serial.print("Broadcasting service UUID: ");
  Serial.println(myServiceUUID.getUuid128String());
  
  BTstack.startAdvertising();
  delay(5000);
  BTstack.stopAdvertising();
  delay(5000);
  BTstack.bleStartScanning();

  //Interrupt for Vertical and Horizontal sync.
  pinMode(vSyncPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(vSyncPin), vSync, FALLING);
  pinMode(hSyncPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hSyncPin), hSync, RISING);

  pinMode(tempTrig, INPUT_PULLUP);
}
void vSync() {
  screen.hSyncCounter = screen.vSyncOffset;
}

void hSync() {
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
  debug();
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
  const char *myCharArray = bleAdvertisement->getBdAddr()->getAddressString();
  String myString(myCharArray);
  if (myString == s) {
    Serial.println("\nBLE ShieldService V2 found!\n");
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
      Serial.print("Device connected: ");
      Serial.println(device->getHandle());
      device->discoverGATTServices();
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
void deviceDisconnectedCallback(BLEDevice *device) {
  (void)device;
  Serial.println("Disconnected, starting over..");
  BTstack.bleStartScanning();
}

int gattWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {
  char tempBuffer[size + 1];
  memcpy(tempBuffer, buffer, size);
  tempBuffer[size] = '\0';  // Null-terminate the string

  Serial.print("Received value: ");
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
#if DEBUG
void debug() {
{
    delay(10);
    screen.y++;
    if (screen.x == screen.width) screen.x = 0;
    if (screen.y >= screen.height) {
        screen.y = 0;
        screen.x++;
    }
}
#else
void debug() {}
#endif
