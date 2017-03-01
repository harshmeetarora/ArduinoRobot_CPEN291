#define MAXSPEED 100
#define threshold 700

const int motorLPin1 = 2;
const int motorLPin2 = 3;
const int motorLEnable = 5;

const int motorRPin1 = 4;
const int motorRPin2 = 5;
const int motorREnable = 7;

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

void setup() {

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
   
}

void loop() {
//  detect();

//  updateDeviation();

//  updateCorrection();

//  execute();

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

