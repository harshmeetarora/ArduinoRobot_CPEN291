/*
 * Part of this code references the code present in 
 * https://www.dfrobot.com/wiki/index.php/Arduino_Motor_Shield_(L298N)_(SKU:DRI0009)
 */

//include the servo motor library 
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

//Initialize the global variables
int mode = 0;
int currentSpeed = 0;
int pos; // servo arm angle
float distance; // distance read by the rangefinder
float robotLinearSpeed = 0;
float speedSlope = topSpeed/(MAXDISTANCE-MINDISTANCE);
int robotAngularSpeed; // Ask Jamie... 

//Initialize a servo motor object
Servo servo;

void setup()
{
  //start a serial connection
  Serial.begin (9600);
  
  // acquire pinmodes of all sensors
  // acquire motor pinmodes
  pinMode(rightMotor, OUTPUT);
  pinMode(leftMotor, OUTPUT);

  //acquire sensor pinmodes 
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  //attach the servo motor to its pin
  servo.attach(SERVOPIN);
  //set initial direction of sevo
  servo.write(90);

  // acquire the mode from buttons 
  acquireMode();
}

void loop()
{
  // acquire the mode from buttons
  // check state of 2 buttons for 4 modes
  acquireMode();
  if (mode == PF1)
  {
    functionality1();
  }
  else if (mode == PF2)
  {
    functionality2();
  }
  else if (mode == BT)
  {
    functionality3();
  }
  drive();
}

/*
 * Moves robot in a straight line until an object is encountered 
 * Then, turns robot left or right
 * Finally, proceeds in the new direction
 */
void functionality1(){
  distance = readDistance();
  if(distance>=400){
    robotLinearSpeed = topSpeed;
  } else { 
    robotLinearSpeed = speedSlope*(distance - MINDISTANCE);
  }
  if (robotLinearSpeed < 0.01){
    robotLinearSpeed = 0;
    char newDirection = servoScan();
    newDirection == 'L' ? turnLeft() : turnRight();
  }
}

/*
 * Moves the robot on a flat surface while following a strip
 * of black electrical tape, turning if necessary
 */
void functionality2()
{
  
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
{
  (SWITCH1 && SWITCH2) ? mode = 3 : SWITCH1 ? mode = 1 : 
    SWITCH2 ? mode = 2 : mode = 0;
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

void runTopSpeed()
{
  currentSpeed = topSpeed;
  analogWrite(rightMotor, topSpeed);
  analogWrite(leftMotor, topSpeed);
  evaluateHallSensors();
}

void evaluateHallSensors()
{
  // check actual wheel speed and adjust power levels accordingly
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

// Turns robot 90* left
void turnLeft(){
  setMotorDirection(0,1);
  analogWrite(leftMotor, 50);
  analogWrite(leftMotor, 50);
  delay(1000);
  fullStop();
}

// Turns robot 90* right
void turnRight(){
  setMotorDirection(1,0);
  analogWrite(leftMotor, 50);
  analogWrite(leftMotor, 50);
  delay(1000);
  fullStop();
}

// Motors read global speed/direction variables
// and are written to accordingly
void drive(){
  
}

void fullStop(){
  analogWrite(leftMotor, 0);
  analogWrite(rightMotor, 0);
}


