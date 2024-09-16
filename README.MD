# Area-51-Arcade-IR-Gun
A usb mouse adapter for the Area 51 Arcade cabinet. This allows you to replace a light gun with a mouse or IR gun that emulates a mouse like Sinden, Gun4IR, or OpenFire.

## Hardware
- Arduino Uno
- USB Host Shield
- Usb Mouse or IR light gun.

## How to build
- Put the USB Host Shield on the Uno
- Connect vSync to Pin 3
- Connect hSync to Pin 2
- Connect lightgun sensor pin to pin 7
- Connect lightgun trigger pin to pin 6
- Upload the Mouse2Arcade.ino file to the arduino uno via the arduino ide.

Power on the arcade in service mode, since you can't see the crosshairs in normal arcade mode for calibration of your IR gun.

## How it works
I am reading the vertical sync and horizontal sync video data to recognize what part of the screen is being rendered. When at the right location I am bringing the aimpin low, which simulates the light gun seeing a white pixel.

## To do
- Add auto reload option.
- Add turbo fire option.

