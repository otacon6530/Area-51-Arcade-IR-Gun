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
| Wire, DFRobotIRPosition, etc.  |
+-------------------------------+
              |
              V
+-------------------------------+
| Setup:                        |
| - Set SDA and SCL pins         |
| - Start communication with     |
|   Wire1 and DFRobotIRPosition  |
| - Begin Serial communication   |
+-------------------------------+
              |
              V
+---------------------------------------------------+
|                   Main Loop                       |
+---------------------------------------------------+
              |
              V
+-------------------------------+
| Call setPosition() to update   |
| coordinates from the sensor    |
| - Check if posCounter == 4     |
|   (4 positions received)       |
+-------------------------------+
              |
              V
+-------------------------------+
| Calculate perspective matrix   |
| using calculatePerspectiveTransform()             |
| and Gaussian Elimination                            |
+-------------------------------+
              |
              V
+-------------------------------+
| Apply the perspective transform|
| to all four points and the     |
| defaultCameraCoord             |
| using applyPerspectiveTransform() |
+-------------------------------+
              |
              V
+-------------------------------+
| Debug the results:             |
| - Print IR sensor positions    |
| - Print calculated coordinates |
| - Print transformed points     |
+-------------------------------+
              |
              V
+-------------------------------+
| Repeat the loop                |
+-------------------------------+
</pre>

