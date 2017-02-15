#include <Servo.h>
#define SERVOPIN 2
#define TRIG 4
#define ECHO 5
#define LM A0
#define MAXSPEED 100
#define MAXDISTANCE 400 // in cm
#define MINDISTANCE 10  // in cm

Servo servo;

int pos; // servo arm angle
float distance; // distance read by the rangefinder
float robotSpeed;
float speedSlope = MAXSPEED/(MAXDISTANCE-MINDISTANCE);
int robotDirection;

void setup() {
  Serial.begin (9600);
  servo.attach(SERVOPIN);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT); 
  servo.write(90);
}

void loop() {
  
}

// Defines behaviour as detailed in lab handout
// 
void function1(){
  distance = readDistance();
  if(distance>=400){
    robotSpeed = MAXSPEED;
  } else { 
    robotSpeed = speedSlope*(distance - MINDISTANCE);
  }
  if (robotSpeed < 0.01){
    robotSpeed = 0;
    char newDirection = servoScan();
    newDirection == L ? turnLeft() : turnRight();
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
  return (newDist1 >= newDist2) ? L : R;
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

