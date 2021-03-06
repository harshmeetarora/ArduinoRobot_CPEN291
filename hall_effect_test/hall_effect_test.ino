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

//The pins for functionality 1
#define SERVOPIN 1
#define TRIG 3
#define ECHO A4

// functionality 1 constants
#define MAXDISTANCE 200.0 // in cm
#define MINDISTANCE 10.0  // in cm

// Switches for reading mode:
#define SWITCH1 3  
#define SWITCH2 8

// Modes: 
#define OFF 0   // 0 is "off"
#define PF1 1   // 1 for principle functionality 1
#define PF2 2   // 2 for principle functionality 2
#define BT  3    // 3 for the remote control with the bluetooth application

//Initialize the motor PWM speed control pins and constants
#define enableRightMotor 5
#define rightMotor 4
#define enableLeftMotor 6
#define leftMotor 7
#define topSpeed 35.0
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

// Constructing datatypes for App
LiquidCrystal lcd(0);
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(0);
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

// Optical Sensor Pins
int infraPins[4] = {A1,A2,A3,A4};
 
//Initialize the global variables: 

int mode = 0; // 4 modes
int pos; // servo arm angle
float distance; // distance read by the rangefinder
int robotLinearSpeed = 0; // Desired linear speed of the robot
float speedSlope = (topSpeed-minSpeed)/(MAXDISTANCE-MINDISTANCE);
// The following are for PF2:
int infraSensorAnalog[4] = {0,0,0,0}; // Storing optical sensor values
int infraSensorDigital[4] = {0,0,0,0};
int infraSensors = B0000;  // binary representation of sensors
int angleCorrection = 0; // goes from 0 to 180, 90 is straight ahead
int lastAngle = 0;  // Keep track of the optical sensors' last angle in case line is lost
// For Hall-effect correction:
float leftRPM;
float rightRPM;
char firstReadingLeft;
char firstReadingRight;
char readyToReadRight = 1;
char readyToReadLeft = 1;
unsigned long lastInterruptRight = 0;
unsigned long lastInterruptLeft = 0;
int leftCorrectionOffset = 0;
int rightCorrectionOffset = 0;
char newLeftSpeedFlag = 1;
char newRightSpeedFlag = 1;
float tireSpeedLeft = 0.0;
float tireSpeedRight = 0.0;
int controlSpeedRight = 0;
int controlSpeedLeft = 0;
int PMWDriveSignalLeft;
int PMWDriveSignalRight;
float desiredSpeed = 55.0;
float LrobotLinearSpeed;
float RrobotLinearSpeed;

// App Variables:
int xCoordinate;
int yCoordinate;

//Initialize a servo motor object
Servo servo;

void setup()
{
  //start a serial connection
  Serial.begin (9600);

  pinMode(SWITCH1, INPUT);
  pinMode(SWITCH2, INPUT);
  
  // set motor pinmodes
  pinMode(rightMotor, OUTPUT);
  pinMode(leftMotor, OUTPUT);
  //pinMode(enableRightMotor, OUTPUT);
  //pinMode(enableLeftMotor, OUTPUT);

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
  //mode = BT;
  
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
  firstReadingLeft = 1;
  firstReadingRight = 1;
}

void loop()
{
  // check state of 2 buttons for 4 modes
  if (mode == PF1)
  {
    functionality1();
    if(PMWDriveSignalRight < 100){
      PMWDriveSignalRight = 0;
    }
    if(PMWDriveSignalLeft <100){
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
 * Then, turns robot left or right
 * Finally, proceeds in the new direction
 */
void functionality1()
{
  distance = readDistance();
  
  if(distance>=MAXDISTANCE){
   
    desiredSpeed = topSpeed;
    checkWheels();
    
  } else if (distance < MAXDISTANCE && distance >= 10) { 

    desiredSpeed = speedSlope*(distance - MINDISTANCE) + minSpeed;
    //desiredSpeed = map(robotLinearSpeed,minSpeed,topSpeed,0,65);
    // rightRPM = map(robotLinearSpeed,minSpeed,topSpeed,0,maxRPM);
    checkWheels();
  } else {
    desiredSpeed = 0;
    // rightRPM = 0;
    fullStop();
    char newDirection = servoScan();
//    Serial.println(newDirection);
    newDirection == 'L' ? turnLeft() : turnRight();
    firstReadingLeft = 1;
    firstReadingRight = 1;
  }
  setMotorDirection(FWD,FWD);
}

/*
 * Moves the robot on a flat surface to follow 
 * a strip of black electrical tape
 */
void functionality2()
{
  lineScan();
  updateDirection();
  lineFollow();
}

// Reads 2 switches and sets mode accordingly
void acquireMode()
{ /*
  (SWITCH1 && SWITCH2) ? mode = 3 : SWITCH1 ? mode = 1 : 
    SWITCH2 ? mode = 2 : mode = 0;
   */ // Commented out just for testing
   Serial.println("Please enter a mode. 1 for PF1, 2 for PF2, 3 for BT");
  while (!digitalRead(SWITCH1) && !digitalRead(SWITCH2)) {
    
  }

  delay(2000);
  int sw1 = digitalRead(SWITCH1);
  int sw2 = digitalRead(SWITCH2);

  if (sw1 && sw2) {
    mode = 3;
  } else if (sw1 && !sw2) {
    mode = 2;
  } else if (!sw1 && sw2) {
    mode = 1;
  } else {
    mode = 0;
    Serial.println("Mode is 0");
  }

  Serial.println("waiting for user to return switches to off");
  while(digitalRead(SWITCH1) || digitalRead(SWITCH2)) {
    // wait
    
  }
  Serial.print("mode = ");Serial.println(mode);
}

// Sets motor direction
void setMotorDirection(int left, int right)
{
  if (left) {
    digitalWrite(leftMotor, FWD);   
  } else {
    digitalWrite(leftMotor, REV);   
  }
  
  if (right) {
   digitalWrite(rightMotor, FWD);  
  } else {
    digitalWrite(rightMotor, REV);
  }
}

// check actual wheel speed and adjust power levels accordingly
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
  //     Serial.println(tireSpeedLeft);
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

// Scans left and right to determine which offers more space
char servoScan()
{
  servo.write(0);
  delay(800);
  float newDist1 = readDistance();
  servo.write(180);
  delay(1200);
  float newDist2 = readDistance();
  servo.write(90);
  return (newDist1 >= newDist2) ? 'L' : 'R';
}

// Turns robot 90* left
void turnLeft()
{
  setMotorDirection(FWD,REV);
  analogWrite(enableLeftMotor, topSpeed-100);
  analogWrite(enableRightMotor, topSpeed-100);
  delay(550);
  fullStop();
}

// Turns robot 90* right
void turnRight()
{
  setMotorDirection(REV,FWD);
  analogWrite(enableLeftMotor, topSpeed-100);
  analogWrite(enableRightMotor, topSpeed-100);
  delay(550);
  fullStop();
}

// Motors read global speed/direction variables
void drive(int leftSpeed, int rightSpeed)
{
  //Serial.print("leftSpeed: ");Serial.println(leftSpeed);
  //Serial.print("rightSpeed: ");Serial.println(rightSpeed);
  analogWrite(enableLeftMotor, leftSpeed);
  analogWrite(enableRightMotor, rightSpeed);
}

void fullStop()
{
  digitalWrite(leftMotor, LOW);
  digitalWrite(rightMotor, LOW);
  analogWrite(enableLeftMotor, 0);
  analogWrite(enableRightMotor, 0);
  firstReadingRight = 1;
  firstReadingLeft = 1;
}

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

void lineFollow()
{
  if(angleCorrection==90){     
     setMotorDirection(FWD,FWD);
     drive(topSpeed, topSpeed);
    }
  if(angleCorrection==0){
     setMotorDirection(FWD,FWD);
     drive(topSpeed-150, topSpeed-50);
    }
   if(angleCorrection==45){     
     setMotorDirection(FWD,FWD);
     drive(topSpeed-10, topSpeed-60);
    }
  if(angleCorrection==135){     
     setMotorDirection(FWD,FWD);
     drive(topSpeed-60, topSpeed-10);
    }
  if(angleCorrection==180){ 
     setMotorDirection(FWD,FWD);
     drive(topSpeed-50, topSpeed-150);
    }
}

void turnOffLCD() {
  lcd.updatePins(13, 8, 12, 11, 10, 9);
  lcd.begin(16,2);
  lcd.clear();
  lcd.updatePins(8,7,6,5,4,3);
  pinMode(LCD_ENABLE_PIN, OUTPUT);
  digitalWrite(LCD_ENABLE_PIN, LOW);
}

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
      int forwardSpeed = min(250, sqrt(pow(xCoordinate,2) + pow(yCoordinate,2))); 
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
    //Serial.println("angle: ");Serial.print(angle);
  } else {
    setMotorDirection(1,1);
    //Serial.println("forward");
  }
  
  if (abs(abs(angle) - FORWARD_ANGLE) <= JOYSTICK_ANGLE_MARGIN) {
    //Serial.println("angle: ");Serial.print(angle);
    // drive in a straight line forwards (or backwards)
    drive(abs(motorSpeed), abs(motorSpeed));
    //analogWrite(rightMotor, motorSpeed);
   // analogWrite(leftMotor, motorSpeed);
  } else if (abs(angle) > FORWARD_ANGLE) {
    // desired turning angle is greater than forward angle, so turn left if going forward
    // or reverse to the left if moving backward
    drive((int) abs(motorSpeed * sin(angle * (PI / 180))), abs(motorSpeed));
    //Serial.println("motorSpeed: ");
    //Serial.println((int) abs(motorSpeed * sin(angle * (PI / 180))));
  } else  {
    // desired turning angle is less than forward angle, so turn right if going forward
    // or reverse to the right if moving backward
    drive(abs(motorSpeed), (int) abs(motorSpeed * sin(angle * (PI / 180))));
    //Serial.println("motorSpeed: ");
    //Serial.println((int) abs(motorSpeed * sin(angle * (PI / 180))));
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
  float avgRPM = map((PMWDriveSignalLeft+PMWDriveSignalRight)/2.0, 70, 255, 0, 35);
  lcd.print(avgRPM);  
  lcd.print("RPM");
}
