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

   
//  digitalWrite(M1,LOW);   
//   digitalWrite(M2, LOW);       
//   analogWrite(E1, 140);   //PWM Speed Control
//   analogWrite(E2, 0);   //PWM Speed Control
}

void loop() {  

  checkLeftHallEffectSensor();

  

}


void checkLeftHallEffectSensor(){
  int i;
  i = analogRead(A0);
//  Serial.println(i);
  if ( ((millis() - lastInterruptLeft) >  1000)  && (i < 100) ){
    if ( firstReadingLeft){
      halfRotationTimeLeft = millis();
      firstReadingLeft = 0;
      Serial.println("first detect");
    } else {
      lastInterruptLeft = millis();
      tireSpeedLeft = 0.5 * PI * 6.5 / ((millis() - halfRotationTimeLeft) / 1000) ; //devided by 1000 to translate millis into seconds
      firstReadingLeft = 1;
      Serial.println("second detect");
      Serial.println(tireSpeedLeft);

      
      //updateLeftTireSpeed(desiredSpeed);
      // possibly update speed here
      
    }

    Serial.println(" once or twice");
  }

  
  
}

void checkRightHallEffectSensor(){
  int i;
  i = analogRead(A1);
  if ( ((millis() - lastInterruptRight) >  1000)  && (i < 100) ){
    if ( firstReadingRight){
      halfRotationTimeRight = millis();
      firstReadingRight = 0;
    } else {
      lastInterruptRight = millis();
      tireSpeedRight = 0.5 * PI * 6.5 / ((millis() - halfRotationTimeRight) / 1000) ; //devided by 1000 to translate millis into seconds
      firstReadingRight = 1;
      //updateRightTireSpeed(desiredSpeed);
      // possibly update speed here
      
    }
  }
  
}


void updateLeftTireSpeed(double desiredSpeed){
    double change;
    double PMWDriveSignal;
    
  
    change = (desiredSpeed - tireSpeedLeft) ;
    controlSpeedLeft = max(controlSpeedLeft + change, 0);
    if (controlSpeedLeft > 65) {
      controlSpeedLeft = 65;
    }
        PMWDriveSignal = map(controlSpeedLeft , 0, 65, 0, 255);  
    Serial.print("desiredSpeed : "); Serial.println(desiredSpeed);
    Serial.print("tireSpeedLeft : "); Serial.println(tireSpeedLeft);
    Serial.print("change : "); Serial.println(change);
    Serial.print("controlSpeedLeft : "); Serial.println(controlSpeedLeft);
    Serial.print("PMW signal : "); Serial.println(PMWDriveSignal);
    Serial.println("-----------------------------------------------");
     
    digitalWrite(M2, HIGH);       
    analogWrite(E2, PMWDriveSignal );   //PWM Speed Control
    if (PMWDriveSignal < 20){    // update tire speed to 0 if DriveSignal is too small to make the tire turn 
      tireSpeedLeft = 0;  
    }
  
}

void updateRightTireSpeed(double desiredSpeed){
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
