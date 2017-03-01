#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <math.h>
#include <Servo.h>

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
float robotAngularSpeed; // Ask Jamie... 

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

byte negative_data_count;

int xCoordinate;
int yCoordinate;

byte xread;
byte yread;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  negative_data_count = 0; // keep track of negative x and y values

  xread = 0;
  yread = 0;

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
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      //Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      int byte1 = (int) BTLEserial.read();
      if (c == 255) { 
        negative_data_count++;
      } else {
        if (negative_data_count != 0) { // if we have received a leading negative byte from the BLE connection
          c = -c; // convert this value to its negative
          negative_data_count--; // decrement # of leading negative bytes we still have to process (should be 0, 1 or 2)
          allocateCToXYCoordinates(c);
        } else {
          allocateCToXYCoordinates(c);
        }
        Serial.println(c);
      }
    }
  }
}

void allocateCToXYCoordinates(int c) {
  if (!xread && !yread) { // if we haven't read any values yet
    xCoordinate = -c;
    xread = !xread; // we read x coordinate first to establish convention
  } else if (xread && !yread) {
    // we just read a x value
    yCoordinate = -c;
    yread = !yread;
    xread = !xread; 
  } else if (!xread && yread) {
    // we just read a y value
    xCoordinate = -c;
    xread = !xread;
    yread = !yread;
  } else {  
    Serial.println("ERROR!"); // we're out of order
  }
}

void convertXYandDrive() {
  int maxSpeed = 250; // max y value we will read from Bluetooth
  robotLinearSpeed = (yCoordinate / maxSpeed) * 100;
  
  float circleRadius = sqrt(pow((double) xCoordinate, 2.0) + pow((double) yCoordinate, 2.0))  ;
  robotAngularSpeed = xCoordinate * cos(circleRadius);
  drive(robotLinearSpeed, robotAngularSpeed);
}

// Motors read global speed/direction variables
// angle -90 to 90
// angle < 0 is left
// angle > 0 is right
void drive(float forwardSpeed, int angle){
  
}

// Sets motor direction
void setMotorDirection(int right, int left)
{
  if(right) {
    digitalWrite(enableRightMotor, HIGH); 
  } else {
    digitalWrite(enableRightMotor, LOW); 
  }
  if(left) {
    digitalWrite(enableLeftMotor, HIGH); 
  } else {
    digitalWrite(enableLeftMotor, LOW); 
  }
}

int mapFloatToInt(float x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

