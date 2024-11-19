// Example by Tom Igoe
// Modified for https://www.dfrobot.com by Lumi, Jan. 2014

/*
   This code should show one colored blob for each detected IR source (max four) at the relative position to the camera.
*/

import processing.serial.*;

int lf = 10;    // Linefeed in ASCII

String myString = null;
String port = "COM12";
Serial myPort;  // The serial port

void setup() {
  // List all the available serial ports
  println(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, port, 19200);
  myPort.clear();
  // Throw out the first reading, in case we started reading
  // in the middle of a string from the sender.
  myString = myPort.readStringUntil(lf);
  myString = null;
  size(50,50);
  fullScreen();
  //frameRate(30);
}

void draw() {
  background(77);
  //while (myPort.available() > 0) {
    myString = myPort.readStringUntil(lf);
    if (myString != null) {
      int[] output = int (split(myString, ','));

      println(myString); // display the incoming string

      int xx = output[0]/2;
      int yy = output[1]/2;

      int ww = output[2]/2;
      int zz = output[3]/2;

      int xxx = output[4]/2;
      int yyy = output[5]/2;

      int www = output[6]/2;
      int zzz = output[7]/2;

      ellipseMode(RADIUS);  // Set ellipseMode to RADIUS
      fill(255, 0, 0);  // Set fill to white
      ellipse(xx, yy, 20, 20);
      ellipseMode(RADIUS);  // Set ellipseMode to RADIUS
      fill(0, 255, 0);  // Set fill to white
      ellipse(ww, zz, 20, 20);

      ellipseMode(RADIUS);  // Set ellipseMode to RADIUS
      fill(0, 0, 255);  // Set fill to white
      ellipse(xxx, yyy, 20, 20);
      ellipseMode(RADIUS);  // Set ellipseMode to RADIUS
      fill(255);  // Set fill to white
      ellipse(www, zzz, 20, 20);

    }
}
