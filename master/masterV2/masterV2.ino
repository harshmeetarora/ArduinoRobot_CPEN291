/*
 * Part of this code references the code present in 
 * https://www.dfrobot.com/wiki/index.php/Arduino_Motor_Shield_(L298N)_(SKU:DRI0009)
 */
//include the libraries we need
#include <Servo.h>
#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <math.h>
#include <LiquidCrystal.h>

// define the pins for functionality 1
#define SERVOPIN 1
#define TRIG 3
#define ECHO A4

// define functionality 1 constants
#define MAXDISTANCE 200.0 // in cm
#define MINDISTANCE 10.0  // in cm

// Switches for reading mode:
#define SWITCH1 3   
#define SWITCH2 8

// Modes: 
#define OFF 0   // 0 is "off"
#define PF1 1   // 1 for principle functionality 1
#define PF2 2   // 2 for principle functionality 2
#define BT  3   // 3 for the remote control with the bluetooth application

//Initialize the motor PWM speed control pins and constants
#define enableRightMotor 5 // motor analog pins
#define enableLeftMotor 6

#define rightMotor 4 // motor direction (digital) pins
#define leftMotor 7

// define motor constants
#define topSpeed 35.0
#define maxSpeed 152
#define minSpeed 10.0 // TODO: Adjust this based on testing
#define FWD LOW
#define REV HIGH
#define maxRPM 60
#define powerIncrement 10

// Hall Effect Sensor Pins
#define rightHallPin A5
#define leftHallPin  A0

// LCD Pins
#define LCD_ENABLE_PIN 8
#define lcd1      9
#define lcd2      10
#define lcd3      11
#define lcd4      12
#define lcd5      13

// App constants
#define FORWARD_ANGLE 90 // 90 degrees is forward (corresponds to the y-axis in the app axes)
#define JOYSTICK_ANGLE_MARGIN 5 // angle error margin (in degrees) 

// Bluetooth Pins
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

 
//Initialize the global variables: 
int mode = 0; // mode variable
int pos; // servo arm angle
float distance; // distance read by the rangefinder
int robotLinearSpeed = 0; // Desired linear speed of the robot
float speedSlope = (topSpeed-minSpeed)/(MAXDISTANCE-MINDISTANCE);

/****************** Optical Sensor Variables *********************/
// Optical Sensor Pins
int infraPins[4] = {A1,A2,A3,A4};

// The following are for PF2:
int infraSensorAnalog[4] = {0,0,0,0}; // Storing optical sensor values
int infraSensorDigital[4] = {0,0,0,0};
int infraSensors = B0000;  // binary representation of sensors
int angleCorrection = 0; // goes from 0 to 180, 90 is straight ahead
int lastAngle = 0;  // Keep track of the optical sensors' last angle in case line is lost


//********************* Global Variables used for Hall Effect Readings ***********************/

// flags indicating that this is the first reading when the wheels begin to move
char firstReadingLeft;
char firstReadingRight;

// flag indicating that the old magnet on the given wheel has gone by
// and that the new magnet is ready to appear
char readyToReadRight = 1;
char readyToReadLeft = 1;

// time since last interrupt on the given wheel
unsigned long lastInterruptRight = 0;
unsigned long lastInterruptLeft = 0;

// Flag indicating we received an updated speed value as 
// a result of this reading
char newLeftSpeedFlag = 1;
char newRightSpeedFlag = 1;

float tireSpeedLeft = 0.0;
float tireSpeedRight = 0.0;

// This is the speed that is mapped to a PWM value to 
// control the wheels
int controlSpeedRight = 0;
int controlSpeedLeft = 0;

// PWM value between 0 and 255 that is actually written to motors
int PMWDriveSignalLeft;
int PMWDriveSignalRight;

// The speed the robot desires to go at, as a result of 
// reading its surroundings
float desiredSpeed = 55.0;


/*********************** App Constants ***************************/
// in-app user touch coordinates
int xCoordinate;
int yCoordinate;

// Construct a datatype for the Bluetooth app, to be updated later with pins
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(0);

// initialize Bluetooth status to Disconnected
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;


/********************* Shared Objects *****************************/
// initialize an lcd object, to be updated later with pins
LiquidCrystal lcd(0);

//Initialize a servo motor object
Servo servo;

void setup()
{
  // start a serial connection
  Serial.begin (9600);

  // set pinmodes for the switches
  pinMode(SWITCH1, INPUT);
  pinMode(SWITCH2, INPUT);
  
  // set motor pinmodes
  pinMode(rightMotor, OUTPUT);
  pinMode(leftMotor, OUTPUT);
  
  // set up optical sensor pins
  pinMode(infraPins[0], INPUT);
  pinMode(infraPins[1], INPUT);
  pinMode(infraPins[2], INPUT);
  pinMode(infraPins[3], INPUT);
  
  // set up Hall effect pins
  pinMode(leftHallPin, INPUT_PULLUP);
  pinMode(rightHallPin, INPUT_PULLUP);
    
  // acquire the mode from buttons 
  acquireMode();

  // Bluetooth configuration (happens only when bluetooth is used
  if(mode==BT){ // Bluetooth configuration
    xCoordinate = 0;
    yCoordinate = 0;
    turnOffLCD();
    setupBLE();  
   } else { // LCD and rangefinder configuration
    lcd.updatePins(13, 8, 12, 11, 10, 9);
    lcd.begin(16,2);
    //set rangefinder pinmodes 
    //attach the servo motor to its pin
    servo.attach(SERVOPIN);
    //set initial direction of sevo
    servo.write(90);
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
  }

  // initialize Hall effect flags
  firstReadingLeft = 1;
  firstReadingRight = 1;
}

void loop()
{
  // check state of 2 buttons for 4 modes
  if (mode == PF1)
  {
    // perform functionality 1
    functionality1();
    if(PMWDriveSignalRight < 100)
    {
      PMWDriveSignalRight = 0;
    }
    if(PMWDriveSignalLeft <100)
    {
      PMWDriveSignalLeft = 0;
    }
    
    drive(PMWDriveSignalLeft, PMWDriveSignalRight);
    updateLCD(); 
  }
  else if (mode == PF2)
  {
    functionality2();
    lineFollow();
    updateLCD(); 
  }
  else if (mode == BT)
  {
    updateBLE();
  }
}
/*
 * Moves robot in a straight line until an object is encountered 
 * Then, turns robot left or right depending on the space available
 * Finally, proceeds in the new direction
 */
void functionality1()
{
  // initialize the beginning distance
  distance = readDistance();

  // determine speed and turning of robot
  if(distance>=MAXDISTANCE)
  {
    
    // here we want speed to be maximal
    desiredSpeed = topSpeed;
    checkWheels();
  } 
  else if (distance < MAXDISTANCE && distance >= 10) 
  { 
    desiredSpeed = speedSlope*(distance - MINDISTANCE) + minSpeed;
    //desiredSpeed = map(robotLinearSpeed,minSpeed,topSpeed,0,65);
    // rightRPM = map(robotLinearSpeed,minSpeed,topSpeed,0,maxRPM);
    checkWheels();
  } 
  else 
  {
    // case where we need spped to be 0   
    desiredSpeed = 0;
    fullStop();
    char newDirection = servoScan();
    // multuplexer to pick turning direction
    newDirection == 'L' ? turnLeft() : turnRight();
    // placeholder variables
    firstReadingLeft = 1;
    firstReadingRight = 1;
  }
  setMotorDirection(FWD,FWD);
}

/*
 * Moves the robot on a flat surface to follow a strip of black 
 * electrical tape, turning or proceeding accordingly
 */
void functionality2()
{
  lineScan();
  updateDirection();
  lineFollow();
}

/*
 * Reads 2 switches and sets mode accordingly    (complete this by Friday)
 * 11b: mode 3 / 01b: mode 2 / 10b:mode 1 / 00b: stand-by mode (do nothing)
 */
void acquireMode()
{ 
  // Commented out just for testing
  //Serial.println("Please enter a mode. 1 for PF1, 2 for PF2, 3 for BT");
  // print start up info to LCD. Wait for user to input mode.
  lcd.updatePins(13, 8, 12, 11, 10, 9);
  lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Choose mode");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1 = PF1, 2 = PF2");
  lcd.setCursor(0,1);
  lcd.print("3 = BT");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("waiting..");
  while (!digitalRead(SWITCH1) && !digitalRead(SWITCH2)) {
    
  }

  delay(2000);
  int sw1 = digitalRead(SWITCH1);
  int sw2 = digitalRead(SWITCH2);

  if (sw1 && sw2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("mode = 3");
    delay(2000);
    mode = 3;
  } else if (sw1 && !sw2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("mode = 3");
    delay(2000);
    mode = 2;
  } else if (!sw1 && sw2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("mode = 3");
    delay(2000);
    mode = 1;
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("mode = 0. Standby");
    delay(2000);
    mode = 0;
    //Serial.println("Mode is 0");
  }

  Serial.println("waiting for user to return switches to off");
  while(digitalRead(SWITCH1) || digitalRead(SWITCH2)) {
    // wait
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Return switches");
    lcd.setCursor(0, 1);
    lcd.print("to off position");
  }
  Serial.print("mode = ");Serial.println(mode);
}

/*
 * Sets motor direction depending on the parameter inputs
 */
void setMotorDirection(int left, int right)
{
  if (left) { // left wheel should move forward
    digitalWrite(leftMotor, FWD);   
  } else {
    digitalWrite(leftMotor, REV);   
  }
  
  if (right) { // left wheel should move forward
   digitalWrite(rightMotor, FWD);  
  } else {
    digitalWrite(rightMotor, REV);
  }
}

/*
 * Checks the actual wheel speed and adjusts the power levels accordingly
 */
void checkWheels() {
  double change; 
  
  double intervalLeft = millis() - lastInterruptLeft;
  double intervalRight = millis() - lastInterruptRight;
  // we have not read an interrupt in 1.5 seconds, we are stopped!
  if (intervalLeft > 1500) {
    tireSpeedLeft = 0;
  }
  if (intervalRight > 1500){
    
    tireSpeedRight = 0;
  }
  
  if((analogRead(leftHallPin) < 50) && readyToReadLeft){
    readyToReadLeft = 0;
    
    if (firstReadingLeft){
      lastInterruptLeft = millis();
      firstReadingLeft = 0;
    } else {
      tireSpeedLeft = 0.25 * PI * 6.5 / (millis() - lastInterruptLeft / 1000.00) ; //devided by 1000 to translate millis into seconds       
      lastInterruptLeft = millis();
      newLeftSpeedFlag = 1;
    }
  } else if( (analogRead(leftHallPin) > 50) && !readyToReadLeft){
    readyToReadLeft = 1;    
  }
  
  if( (analogRead(rightHallPin) < 50) && readyToReadRight ){
    readyToReadRight = 0;
    
    if (firstReadingRight){
      lastInterruptRight = millis();
      firstReadingRight = 0;
    } else {
      tireSpeedRight = 0.25 * PI * 6.5 / ((millis() - lastInterruptRight) / 1000.00) ; //devided by 1000 to translate millis into seconds
      lastInterruptRight = millis();
      newRightSpeedFlag = 1;
    }
  } else if ((analogRead(rightHallPin) > 50) && !readyToReadRight){
    readyToReadRight = 1;
  }
  if (newLeftSpeedFlag){
    change = (desiredSpeed - tireSpeedLeft) ;
    
    controlSpeedLeft = max(controlSpeedLeft + change, 0);
     if (controlSpeedLeft > 65) {
      controlSpeedLeft = 65;
    }
    PMWDriveSignalLeft = map(controlSpeedLeft, minSpeed, 110, 90, 255);  
    newLeftSpeedFlag = 0;
  }
  
  if (newRightSpeedFlag){
  change = (desiredSpeed - tireSpeedRight) ;
  
  controlSpeedRight = max(controlSpeedRight + change, 0);
   if (controlSpeedRight > 65) {
    controlSpeedRight = 65;
  }
  PMWDriveSignalRight = map(controlSpeedRight, minSpeed, 110, 90, 255);  
  newRightSpeedFlag = 0;
  } 
}

/*
 * function to read and update the distance to the Arduino
 */
float readDistance()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); // start signal
  delayMicroseconds(10);  // delay for 10 us pulse
  digitalWrite(TRIG, LOW); //stop signal
  float EchoWidth = pulseIn(ECHO, HIGH);
//  float temp = readTemperature();
  float speedOfSound = 10000.0/341.0;  //(331.5 + (0.6 * temp)); // in [m/s]
  // speedOfSound = (1/speedOfSound)*10000.0; // in [cm/us]
  return EchoWidth / (speedOfSound * 2.0);
}

/*
 * Scans left and right to determine which offers more space
 */
char servoScan()
{ 
  // Check left distace
  servo.write(0);
  delay(800);
  float newDist1 = readDistance();

  // Check right distance
  servo.write(180);
  delay(1200);
  float newDist2 = readDistance();

  // Return to starting position and pick the direction to head into
  servo.write(90);
  return (newDist1 >= newDist2) ? 'L' : 'R';
}
 
/*
 * Turns robot 90* left
 */
void turnLeft()
{
  setMotorDirection(FWD,REV);
  analogWrite(enableLeftMotor, topSpeed-100);
  analogWrite(enableRightMotor, topSpeed-100);
  delay(550);
  fullStop();
}

/*
 * Turns robot 90* right
 */
void turnRight()
{
  setMotorDirection(REV,FWD);
  analogWrite(enableLeftMotor, topSpeed-100);
  analogWrite(enableRightMotor, topSpeed-100);
  delay(550);
  fullStop();
}

/*
 * Motors read global speed/direction variables
 */
void drive(int leftSpeed, int rightSpeed)
{
  analogWrite(enableLeftMotor, leftSpeed);
  analogWrite(enableRightMotor, rightSpeed);
}

/*
 * Stops both motors from spinning completely
 */
void fullStop()
{
  digitalWrite(leftMotor, LOW);
  digitalWrite(rightMotor, LOW);
  analogWrite(enableLeftMotor, 0);
  analogWrite(enableRightMotor, 0);

  // reset hall effect flags to recalibrate speed
  firstReadingRight = 1;
  firstReadingLeft = 1;
}

/*
 * Scans the line under the light sensors and stores the line data
 */
void lineScan()
{   
  infraSensors = B0000;
  int i; 
  for (i = 0; i < 4; i++) {
    infraSensorAnalog[i] = analogRead(infraPins[i]);
    if (infraSensorAnalog[i] > 350 && infraSensorAnalog[i] < 950) {
        infraSensorDigital[i] = 1;
    }
    else {infraSensorDigital[i] = 0;}
    int b = 3-i;
    infraSensors = infraSensors + (infraSensorDigital[i]<<b);
    }    
}

/*
 * Constantly updates the direction of the robot
 */
void updateDirection() 
{
  lastAngle = angleCorrection;
  switch (infraSensors) {
    case B0000:
       if (lastAngle < 90) { angleCorrection = 0;}
       else if (lastAngle > 90) {angleCorrection = 180;}
       break;     
     case B1000: // leftmost sensor on the line- SHARP LEFT
       angleCorrection = 180;
       break;  
     case B0100: 
       angleCorrection = 135;
       break;
     case B0010: 
       angleCorrection = 45;
       break;
     case B0001:  
       angleCorrection = 0;
       break;     
     case B0110: 
       angleCorrection = 90;
       break;
     case B1100: 
       angleCorrection = 180;
       break; 
      case B0011: 
       angleCorrection = 0;
       break;
   default:
     angleCorrection = lastAngle;
  }
}

/*
 * Function that follows a line and updates direction based or the angles of the curves
 */
void lineFollow()
{
  if(angleCorrection==90){     
     setMotorDirection(REV ,REV);
     drive(maxSpeed, maxSpeed);
    }
  if(angleCorrection==0){
     setMotorDirection(REV,REV);
     drive(maxSpeed-110, maxSpeed-50);
    }
   if(angleCorrection==45){     
     setMotorDirection(REV,REV);
     drive(maxSpeed-70, maxSpeed-10);
    }
  if(angleCorrection==135){     
     setMotorDirection(REV,REV);
     drive(maxSpeed-10, maxSpeed-70);
    }
  if(angleCorrection==180){ 
     setMotorDirection(REV,REV);
     drive(maxSpeed-50, maxSpeed-110);
    }
}

/*
 * Function to turn off an LCD completely
 */
void turnOffLCD() {
  lcd.updatePins(lcd5, LCD_ENABLE_PIN, lcd4, lcd3, lcd2, lcd1);
  lcd.begin(16,2);
  lcd.clear();
  lcd.updatePins(0,0,0,0,0,0);
  pinMode(LCD_ENABLE_PIN, OUTPUT);
  digitalWrite(LCD_ENABLE_PIN, LOW);
}

/*
 * Sets up the bluetooth functionality
 */
void setupBLE() {
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001"));
  BTLEserial.initWithPins(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
  BTLEserial.begin();
}

/*
 * Moves according to commands sent from a bluetooth iOS application
 */
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
      int forwardSpeed = min(230, sqrt(pow(xCoordinate,2) + pow(yCoordinate,2))); 
      int robotAngle = 0;
      if (xCoordinate != 0) {
        robotAngle = (atan2((double) yCoordinate, (double) xCoordinate)) * (180 / PI); 
      }
    
      // if the angle we should be driving at is within the error margin of 
      // our predefined forward angle, we should not turn and instead go in a straight line
      // otherwise turn
      configureMotorsForBLE(forwardSpeed, robotAngle);
    }
}

void configureMotorsForBLE(int motorSpeed, int angle) {
  
  if (angle < 0) {
    // we should reverse, so take absolute value of motorSpeed
    // and reverse the enable bits on the wheels
    motorSpeed = -motorSpeed;
    setMotorDirection(0,0);
  } else {
    setMotorDirection(1,1);
  }
  
  if (abs(abs(angle) - FORWARD_ANGLE) <= JOYSTICK_ANGLE_MARGIN) {
    //Serial.println("angle: ");Serial.print(angle);
    // drive in a straight line forwards (or backwards)
    drive(abs(motorSpeed), abs(motorSpeed));
  } else if (abs(angle) > FORWARD_ANGLE) {
    // desired turning angle is greater than forward angle, so turn left if going forward
    // or reverse to the left if moving backward
    drive((int) abs(motorSpeed * sin(angle * (PI / 180))), abs(motorSpeed));
  } else  {
    // desired turning angle is less than forward angle, so turn right if going forward
    // or reverse to the right if moving backward
    drive(abs(motorSpeed), (int) abs(motorSpeed * sin(angle * (PI / 180))));
  } 
}

/* Updates the LCD with current mode and speed values */
void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("Turtle in mode ");
  lcd.print(mode);
  displaySpeed();
}

/* Displays current speed to bottom row of LCD */
void displaySpeed() {
  lcd.setCursor(0,1);
  lcd.print("Speed is ");
  if (mode == 1) {
    float avgRPM = map((PMWDriveSignalLeft+PMWDriveSignalRight)/2.0, 70, 255, 0, 35);
    lcd.print(avgRPM);  
    lcd.print("RPM");
  } else if (mode == 2) {
    float pf2speed = map(maxSpeed, 70, 255, 0, 35);
    lcd.print(pf2speed);  
    lcd.print("RPM");
  }
  
}
