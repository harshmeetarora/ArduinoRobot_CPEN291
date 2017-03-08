/*
 * Part of this code references the code present in 
 * https://www.dfrobot.com/wiki/index.php/Arduino_Motor_Shield_(L298N)_(SKU:DRI0009)
 */

//include the servo motor library 
#include <Servo.h>

//The pins for functionality 1
#define SERVOPIN A0
#define TRIG A5
#define ECHO A4

// functionality 1 constants
#define MAXDISTANCE 400.0 // in cm
#define MINDISTANCE 10.0  // in cm

// Switches for reading mode:
#define SWITCH1 0   
#define SWITCH2 1

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
#define topSpeed 255.0
#define minSpeed 100.0 // TODO: Adjust based on testing
#define FWD LOW
#define REV HIGH

// Hall Effect Sensor Pins
#define rightHallPin 2
#define leftHallPin  3

// LCD Pins
#define lcdEnable 8
#define lcd1      9
#define lcd2      10
#define lcd3      11
#define lcd4      12
#define lcd5      13

// Optical Sensor Pins
int infraPins[4] = {A1,A2,A3,A4};
 
//Initialize the global variables: 
int mode = 0; // 4 modes
int currentSpeed = 0; // For Hall-effect correction
int pos; // servo arm angle
float distance; // distance read by the rangefinder
int robotLinearSpeed = 0; // Desired linear speed of the robot
float speedSlope = (topSpeed-minSpeed)/(MAXDISTANCE-MINDISTANCE);
int robotAngle;
int infraSensorAnalog[4] = {0,0,0,0}; // Storing optical sensor values
int infraSensorDigital[4] = {0,0,0,0};
int infraSensors = B0000;  // binary representation of sensors
int angleCorrection = 0; // goes from 0 to 180, 90 is straight ahead
int lastAngle = 0;  // Keep track of the optical sensors' last angle in case line is lost

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
  pinMode(enableRightMotor, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  
  //set rangefinder pinmodes 
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  //attach the servo motor to its pin
  servo.attach(SERVOPIN);
  //set initial direction of sevo
  servo.write(90);

  // set up optical sensor pins
  pinMode(infraPins[0], INPUT);
  pinMode(infraPins[1], INPUT);
  pinMode(infraPins[2], INPUT);
  pinMode(infraPins[3], INPUT);

  // acquire the mode from buttons 
  acquireMode();

  if(mode==BT){
    // bluetooth pin configuration here
  } else {
    // LCD pin configuration here
  }
}

void loop()
{
  // check state of 2 buttons for 4 modes
  if (mode == PF1)
  {
    functionality1();
    drive();
  }
  else if (mode == PF2)
  {
    functionality2();
    lineFollow();
  }
  else if (mode == BT)
  {
    functionality3();
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
  Serial.println(distance);
  if(distance>=MAXDISTANCE){
    robotLinearSpeed = topSpeed;
  } else { 
    robotLinearSpeed = speedSlope*(distance - MINDISTANCE);
  }
  Serial.println(robotLinearSpeed);
  if (robotLinearSpeed < 0.1){
    robotLinearSpeed = 0;
    fullStop();
    char newDirection = servoScan();
    Serial.println(newDirection);
    newDirection == 'L' ? turnLeft() : turnRight();
  }
}

/*
 * Moves the robot on a flat surface to follow 
 * a strip of black electrical tape
 */
void functionality2()
{
  // Add Sahil's code here:
  lineScan();
  updateDirection();
  lineFollow();
}

/*
 * Moves according to commands sent from a bluetooth iOS application
 */
void functionality3()
{
  //add bluetooth functionality here
}

// Reads 2 switches and sets mode accordingly
void acquireMode()
{ /*
  (SWITCH1 && SWITCH2) ? mode = 3 : SWITCH1 ? mode = 1 : 
    SWITCH2 ? mode = 2 : mode = 0;
   */ // Commented out just for testing PF1
   mode = PF1;
}

// Sets motor direction
void setMotorDirection(int right, int left)
{
  if(right) {
    digitalWrite(rightMotor, FWD); 
  } else {
    digitalWrite(rightMotor, REV); 
  }
  if(left) {
    digitalWrite(leftMotor, FWD); 
  } else {
    digitalWrite(leftMotor, REV); 
  }   
}

void evaluateHallSensors()
{
  // check actual wheel speed and adjust power levels accordingly
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
  setMotorDirection(0,1);
  analogWrite(enableLeftMotor, topSpeed-80);
  analogWrite(enableRightMotor, topSpeed-80);
  delay(800);
  fullStop();
}

// Turns robot 90* right
void turnRight()
{
  setMotorDirection(1,0);
  analogWrite(enableLeftMotor, topSpeed-80);
  analogWrite(enableRightMotor, topSpeed-80);
  delay(800);
  fullStop();
}

// Motors read global speed/direction variables
// angle -90 to 90
// angle < 0 is left
// angle > 0 is right
void drive()
{
  digitalWrite(rightMotor, FWD); 
  digitalWrite(leftMotor, FWD);
  analogWrite(enableLeftMotor, robotLinearSpeed);
  analogWrite(enableRightMotor, robotLinearSpeed);
}

void fullStop()
{
  digitalWrite(leftMotor, LOW);
  digitalWrite(rightMotor, LOW);
  analogWrite(enableLeftMotor, 0);
  analogWrite(enableRightMotor, 0);
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
     digitalWrite(rightMotor, LOW );
     digitalWrite(leftMotor, LOW); 
     analogWrite(enableLeftMotor, topSpeed);
     analogWrite(enableRightMotor, topSpeed);  
    }
  if(angleCorrection==0){
    digitalWrite(rightMotor, LOW ); 
    digitalWrite(leftMotor, LOW);  
     analogWrite(enableLeftMotor, topSpeed-150);
     analogWrite(enableRightMotor, topSpeed-50);      
    }
   if(angleCorrection==45){     
    digitalWrite(rightMotor, LOW);
     digitalWrite(leftMotor, LOW); 
     analogWrite(enableLeftMotor, topSpeed-10);
     analogWrite(enableRightMotor, topSpeed-60);   
    }
  if(angleCorrection==135){     
    digitalWrite(rightMotor, LOW);
     digitalWrite(leftMotor, LOW); 
     analogWrite(enableLeftMotor, topSpeed-60);
     analogWrite(enableRightMotor, topSpeed-10); 
    }
  if(angleCorrection==180){ 
    digitalWrite(leftMotor, LOW);    
    digitalWrite(rightMotor, LOW);
     analogWrite(enableLeftMotor, topSpeed-50);
     analogWrite(enableRightMotor, topSpeed-150);
    }
}
