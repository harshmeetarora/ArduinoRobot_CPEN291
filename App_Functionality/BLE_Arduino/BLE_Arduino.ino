#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <Servo.h>
//#include <math.h>

//Initialize the pins for functionality 1
#define SERVOPIN 1
#define TRIG 2
#define ECHO 3
#define LM A0

// functionality 1 constants
#define MAXDISTANCE 400 // in cm
#define MINDISTANCE 10  // in cm

// Switches for reading mode:
#define SWITCH1 8   
#define SWITCH2 9


 
// Modes: 
#define OFF 0   // 0 is "off"
#define PF1 1   // 1 for principle functionality 1
#define PF2 2   // 2 for principle functionality 2
#define BT  3    // 3 for the remote control with the bluetooth application

//Initialize the motor PWM speed control ports
#define rightMotor 4
#define enableRightMotor 5
#define enableLeftMotor 6
#define leftMotor 7 
#define topSpeed 255

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

//Initialize the global variables
int mode = 0;
int currentSpeed = 0;
int pos; // servo arm angle
float distance; // distance read by the rangefinder
float robotLinearSpeed = 0;
float speedSlope = topSpeed/(MAXDISTANCE-MINDISTANCE); 

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

int xCoordinate;
int yCoordinate;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  xCoordinate = 0;
  yCoordinate = 0;
  
  BTLEserial.begin();
}

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop() {
  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Read every 4 values (negativeX, negativeY, x, y)
    // negativeX and negativeY are flags indicating x and y should be read as negative
    
    if (BTLEserial.available() == 4) {
      int negativeX = BTLEserial.read();
      int negativeY = BTLEserial.read();
      
      xCoordinate = BTLEserial.read();
      yCoordinate = BTLEserial.read();
        
      if (negativeX) {
        xCoordinate = -xCoordinate;
      }  
      
      if (negativeY) {
        yCoordinate = -yCoordinate;
      }
    }
  }
}

void convertXYandDrive() {
  int maxSpeed = 255; // max y value we will read from Bluetooth

  // circle radius vector has magnitude root(x^2 + y^2).
  // will use this vector magnitude as the speed. 
  // take the max of this magnitude and 255 which is robot max speed
  robotLinearSpeed = (float) max(255.0, sqrt(pow((double) xCoordinate, 2.0) + pow((double) yCoordinate, 2.0))); 
  
  drive(robotLinearSpeed, atan(yCoordinate/xCoordinate));
}

// Motors read global speed/direction variables
// angle -90 to 90
// angle < 0 is left
// angle > 0 is right
void drive(float forwardSpeed, int angle){
  
}

