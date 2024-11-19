#include <Wire.h>
#include <DFRobotIRPosition.h>
#include <math.h>
#define upperLeft 2
#define upperRight 3
#define lowerLeft 0
#define lowerRight 1

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

void setup() {

    Serial.begin(9600); 

    //Setup IR Camera
    Wire.setSDA(16);
    Wire.setSCL(17);
    Wire1.begin();
    myDFRobotIRPosition.begin();

}

void loop() {
    setPosition();

    // Calculate the perspective transform matrix
    if(posCounter==4){
      debug();
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

      // Apply the transformation to a point
      debug();

      // Print the transformed point
      //Serial.println("Transformed Point:");
      //Serial.print(transformedPoint[0], 4);
      //Serial.print(", ");
      //Serial.println(transformedPoint[1], 4);
    }
    
}

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

void applyPerspectiveTransform(float matrix[3][3], float src[2], float dst[2]) {
    float x = src[0];
    float y = src[1];

    float w = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2];
    dst[0] = (matrix[0][0] * x + matrix[0][1] * y + matrix[0][2]) / w;
    dst[1] = (matrix[1][0] * x + matrix[1][1] * y + matrix[1][2]) / w;
}

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

