# Area-51-Arcade-IR-Gun
A bluetooth IR gun replacement for the Area 51 Arcade cabinet that has upgraded to an LCD screen.  
## Hardware
- 2 X Raspberry Pi Pico W

## How to build
- TBD

## How it works
I am reading the vertical sync and horizontal sync video data to recognize what part of the screen is being rendered. When at the right location I am bringing the aimpin low, which simulates the light gun seeing a white pixel.

## To do
- Upgrade to using wireless PI pico via bluetooth.

## Program flow

<pre>
+---------------------------------------------------+
|                     Start                        |
+---------------------------------------------------+
              |
              V
+-------------------------------+
| Initialize libraries:          |
| - Wire                        |
| - DFRobotIRPosition           |
| - BTstackLib                  |
+-------------------------------+
              |
              V
+-------------------------------+
| Setup:                        |
| - Configure SDA and SCL pins  |
| - Start communication with    |
|   Wire1 and IR camera         |
| - Initialize Serial (9600)    |
| - Set up BTstack and GATT     |
| - Start Bluetooth advertising |
+-------------------------------+
              |
              V
+---------------------------------------------------+
|                   Main Loop                       |
+---------------------------------------------------+
              |
              V
+-------------------------------+
| Check Bluetooth status:       |
| - If disconnected, continue   |
| - If connected:               |
|   - Call triggerCheck()       |
|   - Call setPosition()        |
+-------------------------------+
              |
              V
+-------------------------------+
| In setPosition():             |
| - Request and read IR LED     |
|   positions                   |
| - Assign quadrant coordinates |
|   to coords[] array           |
| - Update posCounter           |
+-------------------------------+
              |
              V
+-------------------------------+
| If posCounter == 4:           |
| - Calculate perspective matrix|
|   with calculatePerspective   |
|   Transform() and             |
|   gaussianElimination()       |
+-------------------------------+
              |
              V
+-------------------------------+
| Apply perspective transform   |
| - Transform IR points         |
| - Transform default camera    |
|   coordinate                  |
| - Update xPos and yPos        |
+-------------------------------+
              |
              V
+-------------------------------+
| Debug:                        |
| - Print posCounter            |
| - Print coords and tCoords    |
| - Print cameraCoord           |
+-------------------------------+
              |
              V
+-------------------------------+
| Repeat the loop               |
+-------------------------------+

</pre>

