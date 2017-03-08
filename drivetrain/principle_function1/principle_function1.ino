/*
 * Part of this code references the code present in 
 * https://www.dfrobot.com/wiki/index.php/Arduino_Motor_Shield_(L298N)_(SKU:DRI0009)
 */

//include the servo motor library 
#include <Servo.h>

//#define PI 3.14159265358979323846

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
  drive(0, 0);
}

void loop()
{
  drive(50, 90);
  drive(50, 0);
  
  delay(500000);
  // acquire the mode from buttons
  // check state of 2 buttons for 4 modes
//  acquireMode();
//  if (mode == PF1)
//  {
//    functionality1();
//  }
//  else if (mode == PF2)
//  {
//    functionality2();
//  }
//  else if (mode == BT)
//  {
//    functionality3();
//  }
//  drive();
}

/*
 * Moves robot in a straight line until an object is encountered 
 * Then, turns robot left or right
 * Finally, proceeds in the new direction
 */
void functionality1(){
  distance = readDistance();
  Serial.println(distance);
  if(distance >= 400){
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
{ /*
  (SWITCH1 && SWITCH2) ? mode = 3 : SWITCH1 ? mode = 1 : 
    SWITCH2 ? mode = 2 : mode = 0;
   */ // Commented out just for testing PF1
   mode = 1;
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
  delay(400);
  float newDist1 = readDistance();
  servo.write(180);
  delay(800);
  float newDist2 = readDistance();
  servo.write(90);
  return (newDist1 >= newDist2) ? 'L' : 'R';
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


///*
// * Turns one motor only at the given speed according to the given angle
// */
//void writeToMotors(float turnSpeed, int angle){
//  if (angle < 0){
//    setMotorDirection(0,1);
//    analogWrite(leftMotor, 0);
//    analogWrite(rightMotor, turnSpeed);
//    //delay(1000);
//   // fullStop();
//  } else if (angle > 0){
//    setMotorDirection(1,0);
//    analogWrite(rightMotor, turnSpeed);
//    analogWrite(leftMotor, 0);
//    //delay(1000);
//   // fullStop();
//  } else {
//    // proceed with both motors at the required speed
//    setMotorDirection(1,1);
//    analogWrite(leftMotor, turnSpeed);
//    analogWrite(rightMotor, turnSpeed);
//  }
//}


