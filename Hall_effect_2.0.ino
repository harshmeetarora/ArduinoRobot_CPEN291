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
   pinMode(2, INPUT_PULLUP);
   pinMode(M1, OUTPUT);   
   pinMode(M2, OUTPUT);
   pinMode(2, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(2), readHallSpeedRight, RISING);
   attachInterrupt(digitalPinToInterrupt(2), readHallSpeedLeft, RISING);
   

   firstReadingLeft = 1;
   firstReadingRight = 1;
   controlSpeedLeft = 0;
   controlSpeedRight = 0;
}

void loop() {
int i;

//  digitalWrite(M1,LOW);   
//   digitalWrite(M2, LOW);       
//   analogWrite(E1, 140);   //PWM Speed Control
//   analogWrite(E2, 120);   //PWM Speed Control
  

    

}


void updateLeftTireSpeed(double desiredSpeed){
    double change;
    double PMWDriveSignal;
    
  
    change = (desiredSpeed - tireSpeedLeft) / 2;
    controlSpeedLeft = max(controlSpeedLeft + change, 0);
    if (controlSpeedLeft > 65) {
      controlSpeedLeft = 65;
    }
        PMWDriveSignal = map(controlSpeedLeft , 0, 65, 0, 255);  
 /*   Serial.print("desiredSpeed : "); Serial.println(desiredSpeed);
    Serial.print("tireSpeedLeft : "); Serial.println(tireSpeedLeft);
    Serial.print("change : "); Serial.println(change);
    Serial.print("controlSpeedLeft : "); Serial.println(controlSpeedLeft);
    Serial.print("PMW signal : "); Serial.println(PMWDriveSignal);
    Serial.println("-----------------------------------------------");
  */   
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
    
    controlSpeedRight = min(controlSpeedRight + change, 0);
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

double readHallSpeedRight(){
  
//  char i = 0;
//  while(digitalRead(2) == 1){ //didnt work
//    i++;

  if ( (millis() - lastInterruptLeft) >  200){
    if ( firstReadingRight){
      halfRotationTimeRight = millis();
      firstReadingRight = 0;
    } else {
      lastInterruptLeft = millis();
      tireSpeedRight = 0.5 * PI * 6.5 / ((millis() - halfRotationTimeRight) / 1000) ; //devided by 1000 to translate millis into seconds
      firstReadingRight = 1;
    }
  }

  
  
}


double readHallSpeedLeft(){

    if ( (millis() - lastInterruptRight) >  200){
      if ( firstReadingLeft){
        halfRotationTimeLeft = millis();
        lastInterruptRight = halfRotationTimeLeft; //xxxxxx
        firstReadingLeft = 0;
      } else {
        lastInterruptRight = millis(); // xxxxxx
        tireSpeedLeft = 0.5 * PI * 6.5 / ((millis() - halfRotationTimeLeft) / 1000) ; //devided by 1000 to translate millis into seconds * circumference of half the tire
        firstReadingLeft = 1;
        
      }
    }
}
