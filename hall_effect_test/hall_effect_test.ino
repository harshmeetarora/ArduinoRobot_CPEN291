#define hallEffect1 2

void setup() {
   Serial.begin (9600);
   pinMode(hallEffect1, INPUT_PULLUP);

}

void loop() {
  

}

void readHallSpeed(){
  float halfRotationTime;

  halfRotationTime = millis();
  while(digitalRead(hallEffect1) != 1);
  halfRotationTime = millis();
  while(digitalRead(hallEffect1) == 1);
  while(digitalRead(hallEffect1) == 0);
  halfRotationTime = (millis() - halfRotationTime) / 1000; //devided by 1000 to translate millis into seconds

  return (0.5 * (PI * 6.5) ) / halfRotationTime;
  
}
