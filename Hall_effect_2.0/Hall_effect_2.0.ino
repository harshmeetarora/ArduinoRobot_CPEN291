int E1 = 5;  
int M1 = 4; 
int E2 = 6;                      
int M2 = 7;  
char firstReadingLeft;
char firstReadingRight;
double tireSpeedLeft;
double tireSpeedRight;
double controlSpeedLeft;
double controlSpeedRight;
double desiredSpeedRight;
double desiredSpeedLeft;

unsigned long lastInterruptRight = 0;
unsigned long lastInterruptLeft = 0;

/* IMPORTANT:
update lastInterrupt to 0 when stopped
*/

void setup() {
   Serial.begin (9600);
   pinMode(M1, OUTPUT);   
   pinMode(M2, OUTPUT);
   pinMode(2, INPUT);
   pinMode(3, INPUT);
   
   attachInterrupt(digitalPinToInterrupt(2), checkRightHallEffectSensor, RISING);
   attachInterrupt(digitalPinToInterrupt(3), checkLeftHallEffectSensor, RISING);

   

   firstReadingLeft = 1;
   firstReadingRight = 1;
   controlSpeedLeft = 0;
   controlSpeedRight = 0;

   
  digitalWrite(M1,LOW);   
   digitalWrite(M2, LOW);       
   analogWrite(E1, 255);   //PWM Speed Control
   analogWrite(E2, 100);   //PWM Speed Control
}

void loop() {  

  

}


void checkLeftHallEffectSensor(){
  
    
    if ( firstReadingLeft){
      lastInterruptLeft = millis();
      firstReadingLeft = 0;
    } else {
      tireSpeedLeft = 0.25 * PI * 6.5 / ((millis() - lastInterruptLeft) / 1000) ; //devided by 1000 to translate millis into seconds       
      lastInterruptLeft = millis();
      updateLeftTireSpeed(); // optional
    }
    
   
}

void checkRightHallEffectSensor(){
 
    if ( firstReadingRight){
      lastInterruptRight = millis();
      firstReadingRight = 0;
    } else {
      tireSpeedRight = 0.25 * PI * 6.5 / ((millis() - lastInterruptRight) / 1000) ; //devided by 1000 to translate millis into seconds
      lastInterruptRight = millis();
      updateRightTireSpeed(); // optional 
    }
    
  
}


void updateLeftTireSpeed(){
    double change;
    double PMWDriveSignal;
    
    change = (desiredSpeedLeft - tireSpeedLeft) ;
    controlSpeedLeft = max(controlSpeedLeft + change, 0);
    if (controlSpeedLeft > 65) {
      controlSpeedLeft = 65;
    }
    PMWDriveSignal = map(controlSpeedLeft , 0, 65, 0, 255);      
    digitalWrite(M2, LOW);       
    analogWrite(E2, PMWDriveSignal );   //PWM Speed Control
   /* if (PMWDriveSignal < 20){    // update tire speed to 0 if DriveSignal is too small to make the tire turn 
      tireSpeedLeft = 0;  
    } // DO THIS SOMEWHERE ELSE */
  
}

void updateRightTireSpeed(){
    double change;
    double PMWDriveSignal;
    
    change = desiredSpeedRight - tireSpeedRight;
    controlSpeedRight = max(controlSpeedRight + change, 0);
    if (controlSpeedRight > 65) {
      controlSpeedRight = 65;
    }
    PMWDriveSignal = map(controlSpeedRight , 0, 65, 0, 255);
    digitalWrite(M1, HIGH);       
    analogWrite(E1, PMWDriveSignal);   //PWM Speed Control  
    /*if (PMWDriveSignal < 20){ // update tire speed to 0 if DriveSignal is too small to make the tire turn 
      tireSpeedLeft = 0; 
    } // DO THIS SOMEWHERE ELSE */
  
}

/*
    Serial.print("desiredSpeed : "); Serial.println(desiredSpeed);
    Serial.print("tireSpeedLeft : "); Serial.println(tireSpeedLeft);
    Serial.print("change : "); Serial.println(change);
    Serial.print("controlSpeedLeft : "); Serial.println(controlSpeedLeft);
    Serial.print("PMW signal : "); Serial.println(PMWDriveSignal);
    Serial.println("-----------------------------------------------");
    */
