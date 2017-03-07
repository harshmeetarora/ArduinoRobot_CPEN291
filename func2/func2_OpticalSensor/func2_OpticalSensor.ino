#define MAXSPEED 255
#define threshold 600


const int motorLEnable = 6;
const int motorREnable = 5;
const int rightMotor = 4;
const int leftMotor = 7; 

int infraPins[4] = {A1,A2,A3,A4};
int infraSensorAnalog[4] = {0,0,0,0};
int infraSensorDigital[4] = {0,0,0,0};// sensor limit 
//int limit = 150;

// binary representation of sensors
int infraSensors = B0000;

// detecting line 
int count =0;

// a score to determine deviation from the line [+180 ; 0]. Negative means the robot is left of the line.
int angle = 0;

//  store the last value of error
int lastAngle = 0;  
//correctipon to speed
int speedcheck=0;


// lap counter
int lap =0;

// set max speed for PWM
int maxSpeed = 250;

// variables to keep track of current speed of motors
int motorSpeed = 0;
void setup() {
  
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  /* Set-up IR sensor pins as input */ 
  pinMode(infraPins[1], INPUT);
  pinMode(infraPins[2], INPUT);
  pinMode(infraPins[3], INPUT);
  pinMode(infraPins[4], INPUT);

  
   /* Set up motor controll pins as output */

  pinMode(motorLEnable,OUTPUT);
  pinMode(motorREnable,OUTPUT);
  pinMode(rightMotor,OUTPUT);        
  pinMode(leftMotor,OUTPUT);

   
}

void loop() {
scan();

updateDirection();

drive();

delay(100);

}
void scan() {
 
  count = 0;
  
  infraSensors = B0000;
    
  for (int i = 0; i < 4; i++) {
    infraSensorAnalog[i] = analogRead(infraPins[i]);

    if (infraSensorAnalog[i] > 350 && infraSensorAnalog[i] < 950) {
        infraSensorDigital[i] = 1;
    }
    else {infraSensorDigital[i] = 0;}
    Serial.print(infraSensorAnalog[i]);
    Serial.print("|");
    count += infraSensorDigital[i];
    int b = 3-i;
    infraSensors = infraSensors + (infraSensorDigital[i]<<b);
    }    
     Serial.println("**");
}

/*For sharp turns (when the robot “loses” the line temporarily) the robot checks the last reading of the sensors, before it lost the line
*/


void updateDirection() {
  
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
       speedcheck=-200;
       break;
      
     case B0100: 
       angle = 135;
       speedcheck=-100;
       break;

     case B0010: 
       angle = 45;
       speedcheck=-100;
       break;

     case B0001:  
       angle = 0;
       speedcheck=-200;
       break;

     //count=2
     
     case B0110: 
       angle = 90;
       speedcheck=0;
       break;
       
     case B1100: 
       angle = 180;
       speedcheck=-100;
       break; 
       
      case B0011: 
       angle = 0;
       speedcheck=-100;
       break;

 default:
     angle = lastAngle;
     speedcheck=0;
  }
}
void drive() {
  //motorSpeed=MAXSPEED+speedcheck;
  
  if(angle==90){     
     digitalWrite(rightMotor, LOW );
     digitalWrite(leftMotor, LOW); 
     analogWrite(motorLEnable, MAXSPEED);
     analogWrite(motorREnable, MAXSPEED);
   
    }
  if(angle==0){
    digitalWrite(rightMotor, LOW ); 
    digitalWrite(leftMotor, LOW);  
     analogWrite(motorLEnable, MAXSPEED-150);
     analogWrite(motorREnable, MAXSPEED-50);
      
    }
   if(angle==45){     
    digitalWrite(rightMotor, LOW);
     digitalWrite(leftMotor, LOW); 
     analogWrite(motorLEnable, MAXSPEED-10);
     analogWrite(motorREnable, MAXSPEED-60);
   
    }
  if(angle==135){     
    digitalWrite(rightMotor, LOW);
     digitalWrite(leftMotor, LOW); 
     analogWrite(motorLEnable, MAXSPEED-60);
     analogWrite(motorREnable, MAXSPEED-10);
   
    }
  if(angle==180){ 
    digitalWrite(leftMotor, LOW);    
    digitalWrite(rightMotor, LOW);
     analogWrite(motorLEnable, MAXSPEED-50);
     analogWrite(motorREnable, MAXSPEED-150);

   
    }
  

}
