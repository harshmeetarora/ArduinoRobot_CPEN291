#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <Servo.h>
#include <math.h>
#include <LiquidCrystal.h>

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

// App constants
#define FORWARD_ANGLE 90 // 90 degrees is forward (corresponds to the y-axis in the app axes)
#define JOYSTICK_ANGLE_MARGIN 5 // angle error margin (in degrees) 
 
// Modes: 
#define OFF 0   // 0 is "off"
#define PF1 1   // 1 for principle functionality 1
#define PF2 2   // 2 for principle functionality 2
#define BT  3    // 3 for the remote control with the bluetooth application

#define LCD_ENABLE_PIN 8

//Initialize the motor PWM speed control ports
#define enableRightMotor 4
#define rightMotor 5
#define enableLeftMotor 7
#define leftMotor 6
#define topSpeed 250

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

//Initialize the global variables
int mode = 0;
int currentSpeed = 0;
int pos; // servo arm angle
float distance; // distance read by the rangefinder
float speedSlope = topSpeed/(MAXDISTANCE-MINDISTANCE); 


/* Function Prototypes */
void updateLCD();
void displayMode();
void displaySpeed();

int i;

//LiquidCrystal lcd();
LiquidCrystal lcd(mode);

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST, mode);

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

int xCoordinate;
int yCoordinate;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (mode == 1) {
    //pinMode(LCD_ENABLE_PIN, OUTPUT);
    //digitalWrite(LCD_ENABLE_PIN, LOW);
    lcd.updatePins(13, 8, 12, 11, 10, 9);
    lcd.begin(16,2);
    lcd.clear();
    lcd.updatePins(8,7,6,5,4,3);
    pinMode(LCD_ENABLE_PIN, OUTPUT);
    digitalWrite(LCD_ENABLE_PIN, LOW);
    xCoordinate = 0;
    yCoordinate = 0;
    setupBLE();
  } else {
    lcd.updatePins(13, 8, 12, 11, 10, 9);
    lcd.begin(16,2);
    i = 0;
  }
  
}

void setupBLE() {
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001"));
  BTLEserial.begin();
}

void loop() {
  if (mode == 1) {
    updateBLE();
  } else {
   updateLCD(); 
  }
}

void updateBLE() {
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out
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
    // Connected!
    readBLESerialAndDrive();
    //Serial.println("hi");
  }
}

void readBLESerialAndDrive() {
  // Read every 4 values (negativeX, negativeY, x, y)
  // the first 2 bytes sent over Bluetooth are flags indicating x and y should be read as negative  
  // we save these flags in negativeX and negativeY
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

      // we'll set the robot speed to be equal to its y coordinate in the app axes
      // and get the angle we want to drive at with tan^-1(y/x), saving it in degrees
      int robotLinearSpeed = min(255, sqrt(pow(xCoordinate,2) + pow(yCoordinate,2))); 
      int robotAngle = (atan2((double) yCoordinate, (double) xCoordinate)) * (180 / PI); 

      // drive at the given speed and angle
      driveViaJoyStick(robotLinearSpeed, robotAngle);
    }
}

void driveViaJoyStick(int forwardSpeed, int angle){
  // if the angle we should be driving at is within the error margin of 
  // our predefined forward angle, we should not turn and instead go in a straight line
  // otherwise turn
  if (abs(abs(angle) - FORWARD_ANGLE) <= JOYSTICK_ANGLE_MARGIN) {
    writeToMotorsForBLE(forwardSpeed, 0);
  } else {
    writeToMotorsForBLE(forwardSpeed, angle);
  }
}

void writeToMotorsForBLE(int motorSpeed, int angle) {
  if (motorSpeed < 0) {
    // we should reverse, so take absolute value of motorSpeed
    // and reverse the enable bits on the wheels
    motorSpeed = -motorSpeed;
    setMotorDirection(1,1);
  } else {
    setMotorDirection(0,0);
  }

  if (angle == 0) {
    // drive in a straight line forwards (or backwards)
    analogWrite(rightMotor, motorSpeed);
    analogWrite(leftMotor, motorSpeed);
    Serial.print("right wheel: ");Serial.println(motorSpeed);
    Serial.print("left wheel: ");Serial.println(motorSpeed);
  } else if (abs(angle) > FORWARD_ANGLE) {
    // desired turning angle is greater than forward angle, so turn left if going forward
    // or reverse to the left if moving backward
    analogWrite(rightMotor, (int) abs(motorSpeed * sin(angle * (PI / 180))));
    analogWrite(leftMotor, motorSpeed);
    Serial.print("left wheel: ");Serial.println((int) abs(motorSpeed * sin(angle * (PI / 180))));
    Serial.print("right wheel: ");Serial.println(motorSpeed);
  } else  {
    // desired turning angle is less than forward angle, so turn right if going forward
    // or reverse to the right if moving backward
    analogWrite(rightMotor, motorSpeed);
    analogWrite(leftMotor, (int) abs(motorSpeed * sin(angle * (PI / 180))));
    Serial.print("left wheel: ");Serial.println(motorSpeed);
    Serial.print("right wheel: ");Serial.println((int) abs(motorSpeed * sin(angle * (PI / 180))));
  } 
}

// Sets the motor direction bits
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


/* Updates the LCD with current mode and speed values */
void updateLCD(){
  lcd.setCursor(0, 0);
  // print the number of seconds since reset:
  lcd.print("hello, world!");
  /*lcd.clear();
  displayMode();
  //displaySpeed();*/
}

/* Displays current mode to top row of LCD */
void displayMode(){
  //int mode = (SWITCH) ? 1 : 2;
  i++;
  lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.write(i);
}

/* Displays current speed to bottom row of LCD */
void displaySpeed(){
  lcd.setCursor(0,1);
  lcd.write("Speed = shut up");
}
