#include <SPI.h>
#include <LiquidCrystal.h>
#include "Adafruit_BLE_UART.h"
//#include <math.h>
#include <Servo.h>

#define SERVOPIN 2
#define TRIG 4
#define ECHO 5
#define LM A0
#define MAXSPEED 100
#define MAXDISTANCE 100 // in cm
#define MINDISTANCE 10  // in cm
#define SWITCH 1   // Used to pick with mode

#define threshold 700

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

//Initialize the pins for functionality 1
#define SERVOPIN 10
#define TRIG 2
#define ECHO 3
#define LM A5

// functionality 1 constants
#define MAXDISTANCE 400.0 // in cm
#define MINDISTANCE 10.0  // in cm

// Switches for reading mode:
#define SWITCH1 8   
#define SWITCH2 9
 
// Modes: 
#define OFF 0   // 0 is "off"
#define PF1 1   // 1 for principle functionality 1
#define PF2 2   // 2 for principle functionality 2
#define BT  3    // 3 for the remote control with the bluetooth application

//Initialize the motor PWM speed control ports
#define enableRightMotor 4
#define rightMotor 5
#define enableLeftMotor 7
#define leftMotor 6
#define topSpeed 255.0

/** ===== Library Data Types ===== */
Servo servo;
LiquidCrystal lcd(12, 11, 5, 4 , 3, 2);
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

int pos; // servo arm angle
float distance; // distance read by the rangefinder
float robotSpeed;
float speedSlope = MAXSPEED/(MAXDISTANCE-MINDISTANCE);
int robotDirection;

const byte motorLPin1 = 2;
const byte motorLPin2 = 3;
const byte motorLEnable = 5;

const byte motorRPin1 = 4;
const byte motorRPin2 = 5;
const byte motorREnable = 7;

int infraPins[4] = {A0,A1,A2,A3};
int infraSensorAnalog[4] = {0,0,0,0};
int infraSensorDigital[4] = {0,0,0,0};// sensor limit 
//int limit = 700;

// binary representation of sensors
int infraSensors = B0000;

// detecting line 
int count =0;

// a score to determine deviation from the line [-180 ; +180]. Negative means the robot is left of the line.
int angle = 0;

//  store the last value of error
int lastAngle = 0;  
//correctipon to speed
int speedcheck=0;

// negativei s left, positive is right
int deviation =0;

// store last deviation 
int deviationLast =0;

// correction to be made based on deviation
int correction =0;

// lap counter
int lap =0;

// set max speed for PWM
int maxSpeed = 255;

// variables to keep track of current speed of motors
int motorSpeed = 0;
//int motorRSpeed = 0;

int xCoordinate;
int yCoordinate;

void setup() {
  Serial.begin(9600);

  /* ============= Servo setup ============ */
  servo.attach(SERVOPIN);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT); 
  servo.write(90);
  pinMode(SWITCH, INPUT);



  /* ========= Bluetooth setup ============ */
  // the next 2 lines might need to be removed 
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  xCoordinate = 0;
  yCoordinate = 0;
  
  BTLEserial.begin();

  /* ======== IR optical sensor setup ======*/
  /* Set-up IR sensor pins as input */ 
  pinMode(infraPins[1], INPUT);
  pinMode(infraPins[2], INPUT);
  pinMode(infraPins[3], INPUT);
  pinMode(infraPins[4], INPUT);

  
   /* Set up motor controll pins as output */
  pinMode(motorLPin1,OUTPUT);        
  pinMode(motorLPin2,OUTPUT);
  pinMode(motorLEnable,OUTPUT);
  
  pinMode(motorRPin1,OUTPUT);        
  pinMode(motorRPin2,OUTPUT);
  pinMode(motorREnable,OUTPUT);

  /* ====== LCD Setup ====== */
  lcd.begin(20,4);
  lcd.print("Hello.  My name is Saul.");
  
}

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop() {
  digitalRead(SWITCH) ? functionality1() : function2();

  /* Continually update LCD */
  updateLCD();
  delay(5);
  
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
  robotSpeed = (float) max(255.0, sqrt(pow((double) xCoordinate, 2.0) + pow((double) yCoordinate, 2.0))); 
  
  drive(robotSpeed, atan(yCoordinate/xCoordinate));
}

// Motors read global speed/direction variables
// angle -90 to 90
// angle < 0 is left
// angle > 0 is right
void drive(float forwardSpeed, float angle){
  if (angle >= -90.0 && angle <= 90.0) 
  {
    
    float motorSpeed = map(forwardSpeed, 0, 255, 0, 61);
    //find the arc length -- the diameter of the robot is 17cm
    float distance = abs((angle/360.0)) * (PI*17.0);
    float travelTime = (distance) / motorSpeed;
    Serial.println("travelTime: ");
    Serial.println(travelTime);

    Serial.println("distance: ");
    Serial.println(distance);

     
    writeToMotors(forwardSpeed, angle);
  
    // time = distance / speed. so, delay to allow full turn
    delay(travelTime * 1000); //now in milliseconds
    
    //delay(5000); //now in milliseconds
  } 
  else 
  {
    Serial.print("The angle is out of the set range\n");
  }
}

// Sets motor direction
void setMotorDirection(int right, int left)
{
  if(right) {
    digitalWrite(rightMotor, HIGH); 
  } else {
    digitalWrite(rightMotor, LOW); 
  }
  if(left) {
    digitalWrite(leftMotor, HIGH); 
  } else {
    digitalWrite(leftMotor, LOW); 
  }   
}

/*
 * Turns one motor only at the given speed according to the given angle
 */
void writeToMotors(float turnSpeed, int angle){
  if (angle < 0){
    setMotorDirection(0,1);
    analogWrite(leftMotor, 0);
    analogWrite(rightMotor, turnSpeed);
    //delay(1000);
   // fullStop();
  } else if (angle > 0){
    setMotorDirection(1,0);
    analogWrite(rightMotor, turnSpeed);
    analogWrite(leftMotor, 0);
    //delay(1000);
   // fullStop();
  } else {
    // proceed with both motors at the required speed
    setMotorDirection(1,1);
    analogWrite(leftMotor, turnSpeed);
    analogWrite(rightMotor, turnSpeed);
  }
}

/*
 * Moves robot in a straight line until an object is encountered 
 * Then, turns robot left or right
 * Finally, proceeds in the new direction
 */
void functionality1(){
  distance = readDistance();
  Serial.println(distance);
  if(distance>=400){
    robotSpeed = topSpeed;
  } else { 
    robotSpeed = speedSlope*(distance - MINDISTANCE);
  }
  Serial.println(robotSpeed);
 if (robotSpeed < 0.1){
    robotSpeed = 0;
    fullStop();
    char newDirection = servoScan();
    Serial.println(newDirection);
    newDirection == 'L' ? turnLeft() : turnRight();
  }
}

// Scans left and right to determine which offers more space
char servoScan(){
  servo.write(0);
  delay(200);
  float newDist1 = readDistance();
  servo.write(180);
  delay(400);
  float newDist2 = readDistance();
  servo.write(90);
  return (newDist1 >= newDist2) ? 'L' : 'R';
}

float readDistance(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); // start signal
  delayMicroseconds(10);  // delay for 10 us pulse
  digitalWrite(TRIG, LOW); //stop signal

  float EchoWidth = pulseIn(ECHO, HIGH);
  float temp = readTemperature();
  float speedOfSound = (331.5 + (0.6 * temp)); // in [m/s]
  speedOfSound = (1/speedOfSound)*10000; // in [cm/us]
  
  return EchoWidth / (speedOfSound * 2);
}

// Read value from LM35 and convert to degrees Celsius
float readTemperature() {
  float t = analogRead(LM);
  return 0.48828125 * t;
}

// Turns robot 90* left
void turnLeft(){
  setMotorDirection(0,1);
  analogWrite(enableLeftMotor, 80);
  analogWrite(enableRightMotor, 80);
  delay(1800);
  fullStop();
}

// Turns robot 90* right
void turnRight(){
  setMotorDirection(1,0);
  analogWrite(enableLeftMotor, 80);
  analogWrite(enableRightMotor, 80);
  delay(1800);
  fullStop();
}

void fullStop() {
  setMotorDirection(1,1);
  analogWrite(enableLeftMotor, 0);
  analogWrite(enableRightMotor, 0);
}

void function2(){
  
}

void Scan() {
 
  count = 0;
  
  infraSensors = B000000;
    
  for (int i = 0; i < 4; i++) {
    infraSensorAnalog[i] = analogRead(infraPins[i]);

    if (infraSensorAnalog[i] >= threshold) {
        infraSensorDigital[i] = 1;
    }
    else {infraSensorDigital[i] = 0;}
    Serial.print(infraSensorAnalog[i]);
    Serial.print("|");
    count += infraSensorDigital[i];
    int b = 3-i;
    infraSensors = infraSensors + (infraSensorDigital[i]<<b);
    }    
}

/*For sharp turns (when the robot “loses” the line temporarily) the robot checks the last reading of the sensors, before it lost the line
*/


void UpdateDeviation() {
  
  lastAngle = angle;
  speedcheck=0;  
  
  switch (infraSensors) {
     
    case B0000:
       if (lastAngle < 90) { angle = 0;}
       else if (lastAngle > 90) {angle = 180;}
       break;
       
     //count=1
     
     case B1000: // leftmost sensor on the line- SHARP LEFT
       angle = 180;
       speedcheck=-95;
       break;
      
     case B0100: 
       angle = 135;
       speedcheck=-95;
       break;

     case B0010: 
       angle = 45;
       speedcheck=-95;
       break;

     case B0001:  
       angle = 0;
       speedcheck=-95;
       break;

     //count=2
     
     case B0110: 
       angle = 90;
       speedcheck=-95;
       break;
       
     case B1100: 
       angle = 165;
       speedcheck=-95;
       break; 
       
      case B0011: 
       angle = 15;
       speedcheck=-95;
       break;

 default:
     angle = lastAngle;
     speedcheck=0;
  }
}
void Drive() {
  motorSpeed=MAXSPEED+speedcheck;
//  drive(angle,motorSpeed);
}

/* Updates the LCD with current mode and speed values */
void updateLCD(){
  displayMode();
  displaySpeed();
}

/* Displays current mode to top row of LCD */
void displayMode(){
  //int mode = getMode();
  int mode = 1;
  lcd.setCursor(0,0);
  lcd.write("MODE %d", mode);
}

/* Displays current speed to bottom row of LCD */
void displaySpeed(){
  //double speed = getSpeed();
  double speed = 2.5;
  lcd.setCursor(0,1);
  lcd.write("Speed = %lf", speed);
}

