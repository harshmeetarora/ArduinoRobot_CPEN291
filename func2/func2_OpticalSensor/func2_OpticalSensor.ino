const int motorLPin1 = 2;
const int motorLPin2 = 3;
const int motorLEnable = 5;

const int motorRPin1 = 4;
const int motorRPin2 = 5;
const int motorREnable = 7;

int infraPins[6] = {A0,A1,A2,A3,A4,A5};
int infraSensorAnalog[6] = {0,0,0,0,0,0};
int infraSensorDigital[6] = {0,0,0,0,0,0};

// sensor limit 
int limit = 700;

// binary representation of sensors
int irSensors = B000000;

// detecting line 
int count =0;

// variable to detect deviation from line (-180;+180)
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
int motorLSpeed = 0;
int motorRSpeed = 0;

void setup() {

  /* Set-up IR sensor pins as input */ 
  pinMode(infraPins[1], INPUT);
  pinMode(infraPins[2], INPUT);
  pinMode(infraPins[3], INPUT);
  pinMode(infraPins[4], INPUT);
  pinMode(infraPins[5], INPUT);
  pinMode(infraPins[6], INPUT);
  
   /* Set up motor controll pins as output */
  pinMode(motorLPin1,OUTPUT);        
  pinMode(motorLPin2,OUTPUT);
  pinMode(motorLEnable,OUTPUT);
  
  pinMode(motorRPin1,OUTPUT);        
  pinMode(motorRPin2,OUTPUT);
  pinMode(motorREnable,OUTPUT);
   
}

void loop() {
//  detect();
//  updateDeviation();
//  updateCorrection();
//  execute();
}







