/*
 * Part of this code references the code present in 
 * https://www.dfrobot.com/wiki/index.php/Arduino_Motor_Shield_(L298N)_(SKU:DRI0009)
 */
 
//Initialize the motor PWM speed control ports
#define M1 4
#define E1 5
#define E2 6
#define M2 7 
#define topSpeed 255

int mode = 0;

void setup()
{
  // acquire pinmodes of all sensors
  // acquire motor pinmodes
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
}

void loop()
{
  // acquire the mode from keypad
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

void functionality1()
{
  enableMotors();
  runTopSpeed();
}

void functionality2()
{
  enableMotors();
}

void functionality3()
{
  enableMotors();
}

void enableMotors()
{
  digitalWrite(E1, HIGH);
  digitalWrite(E2, HIGH);
}

void runTopSpeed()
{
  analogWrite(M1, topSpeed);
  analogWrite(M2, topSpeed);
}



