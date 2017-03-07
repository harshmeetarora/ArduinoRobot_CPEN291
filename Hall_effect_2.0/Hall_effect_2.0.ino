int E1 = 5;  
int M1 = 4; 
int E2 = 6;                      
int M2 = 7;  
char firstReadingLeft;
char firstReadingRight;
double halfRotationTimeLeft;
double halfRotationTimeRight;
double tireSpeedLeft;
double tireSpeedRight;
double controlSpeedLeft;
double controlSpeedRight;
double desiredSpeed = 15;

unsigned long lastInterruptRight = 0;
unsigned long lastInterruptLeft = 0;

void setup() {
   Serial.begin (9600);
   pinMode(M1, OUTPUT);   
   pinMode(M2, OUTPUT);
   pinMode(A0, INPUT_PULLUP);
   pinMode(A1, INPUT_PULLUP);

   

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

  checkLeftHallEffectSensor();
  

}


void checkLeftHallEffectSensor(){
  
  if ( ((millis() - lastInterruptLeft) >  200)  && (analogRead(A0) < 100) ){  
    
    if ( firstReadingLeft){
      lastInterruptLeft = millis();
      halfRotationTimeLeft = lastInterruptLeft;
      firstReadingLeft = 0;
    } else {
      lastInterruptLeft = millis();
      tireSpeedLeft = 0.5 * PI * 6.5 / ((millis() - halfRotationTimeLeft) / 1000) ; //devided by 1000 to translate millis into seconds
      firstReadingLeft = 1;
      updateLeftTireSpeed();
    }
    
  } else if ( (millis() - lastInterruptLeft) > 1000){
    updateLeftTireSpeed();
  }
   
}

void checkRightHallEffectSensor(){
  
  if ( ((millis() - lastInterruptRight) >  1000)  && (analogRead(A5) < 100) ){
    
    if ( firstReadingRight){
      halfRotationTimeRight = millis();
      firstReadingRight = 0;
    } else {
      lastInterruptRight = millis();
      tireSpeedRight = 0.5 * PI * 6.5 / ((millis() - halfRotationTimeRight) / 1000) ; //devided by 1000 to translate millis into seconds
      firstReadingRight = 1;
      updateRightTireSpeed(); 
    }
    
  } else if ( (millis() - lastInterruptRight) > 1000){
     updateRightTireSpeed();
  }
  
}


void updateLeftTireSpeed(){
    double change;
    double PMWDriveSignal;
    
    change = (desiredSpeed - tireSpeedLeft) ;
    controlSpeedLeft = max(controlSpeedLeft + change, 0);
    if (controlSpeedLeft > 65) {
      controlSpeedLeft = 65;
    }
    PMWDriveSignal = map(controlSpeedLeft , 0, 65, 0, 255);      
    digitalWrite(M2, LOW);       
    analogWrite(E2, PMWDriveSignal );   //PWM Speed Control
    if (PMWDriveSignal < 20){    // update tire speed to 0 if DriveSignal is too small to make the tire turn 
      tireSpeedLeft = 0;  
    }
  
}

void updateRightTireSpeed(){
    double change;
    double PMWDriveSignal;
    
    change = desiredSpeed - tireSpeedRight;
    controlSpeedRight = max(controlSpeedRight + change, 0);
    if (controlSpeedRight > 65) {
      controlSpeedRight = 65;
    }
    PMWDriveSignal = map(controlSpeedRight , 0, 65, 0, 255);
    digitalWrite(M1, HIGH);       
    analogWrite(E1, PMWDriveSignal);   //PWM Speed Control  
    if (PMWDriveSignal < 20){ // update tire speed to 0 if DriveSignal is too small to make the tire turn 
      tireSpeedLeft = 0;  
    }
  
}

/*
    Serial.print("desiredSpeed : "); Serial.println(desiredSpeed);
    Serial.print("tireSpeedLeft : "); Serial.println(tireSpeedLeft);
    Serial.print("change : "); Serial.println(change);
    Serial.print("controlSpeedLeft : "); Serial.println(controlSpeedLeft);
    Serial.print("PMW signal : "); Serial.println(PMWDriveSignal);
    Serial.println("-----------------------------------------------");
    */
