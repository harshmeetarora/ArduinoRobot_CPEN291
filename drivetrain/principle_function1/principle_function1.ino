/*
 * Part of this code references the code present in 
 * https://www.dfrobot.com/wiki/index.php/Arduino_Motor_Shield_(L298N)_(SKU:DRI0009)
 */

//include the servo motor library 
#include <Servo.h>

//Initialize the ports for functionality 1
#define SERVOPIN 2
#define TRIG 4     //change
#define ECHO 5     //change
#define LM A0
#define MAXSPEED 100
#define MAXDISTANCE 400 // in cm
#define MINDISTANCE 10  // in cm
//#define SWITCH 1   // Used to pick with mode    //change to keypad input
 
//Initialize the motor PWM speed control ports
#define M1 4
#define E1 5
#define E2 6
#define M2 7 
#define topSpeed 255

//Initialize the global variables
int mode = 0;
int currentSpeed = 0;
int pos; // servo arm angle
float distance; // distance read by the rangefinder
float robotSpeed = 0;
float speedSlope = MAXSPEED/(MAXDISTANCE-MINDISTANCE);
int robotDirection;

//Initialize a servo motor object
Servo servo;

void setup()
{
  //start a serial connection
  Serial.begin (9600);
  
  // acquire pinmodes of all sensors
  // acquire motor pinmodes
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  //acquire sensor pinmodes 
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  //attach the servo motor to its pin
  servo.attach(SERVOPIN);
  //set initial direction of sevo
  servo.write(90);

  // acquire the mode from keypad 
  mode = acquireMode();
}

void loop()
{
  // acquire the mode from keypad
  // if a button (decide on it later) is pressed, reaquire mode 
  mode = acquireMode();
  
  // 1 for principle functionality 1
  // 2 for principle functionality 2
  // 3 for the remote control with the bluetooth application
  if (mode == 1)
  {
    functionality1();
  }
  else if (mode == 2)
  {
    functionality2();
  }
  else if (mode == 3)
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
  // enable and run the motors in a straight line at top speed 
  enableMotors();
  runTopSpeed();
  if (detectObject()){
    // slow down the robot gradually
    //slowDown();
  }
  
  
}

/*
 * Moves the robot on a flat surface while following a strip
 * of black electrical tape, turning if necessary
 */
void functionality2()
{
  enableMotors();
  
}

/*
 * Moves according to commands sent from a bluetooth iOS application
 */
void functionality3()
{
  enableMotors();
  //add bluetooth functionality here
}

int acquireMode()
{
  // add the keypad and LCD prompt here
}

void enableMotors()
{
  digitalWrite(E1, HIGH);
  digitalWrite(E2, HIGH);
}

void runTopSpeed()
{
  currentSpeed = topSpeed;
  analogWrite(M1, topSpeed);
  analogWrite(M2, topSpeed);
  evaluateHallSensors();
}

int detectObject()
{
  //detect if there's an object using the ultrasonic sensor  
}
void evaluateHallSensors()
{
  // check hall effect and adjust accordingly
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

}

// Turns robot 90* right
void turnRight(){
  
}


