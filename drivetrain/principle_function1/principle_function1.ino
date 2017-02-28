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
int currentSpeed = 0;

void setup()
{
  // acquire pinmodes of all sensors
  // acquire motor pinmodes
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  // acquire the mode from keypad 
  mode = acquireMode();
}

void loop()
{
  // acquire the mode from keypad
  // if a button (decide on it later) is pressed, reaquire mode 
  mode = acquireMode();
  
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

/*
 * Moves robot in a straight line until an object is encountered 
 * Then, turns robot left or right
 * Finally, proceeds in the new direction
 */
void functionality1()
{
  // enable and the motors in a straight line at top speed 
  enableMotors();
  runTopSpeed();
  if (detectObject()){
    // slow down the robot gradually
    
  }
  
  
}

/*
 * Moves the robot on a flat surface while following a strip
 * of black electrical tape, turning if necessary
 */
void functionality2()
{
  enableMotors();
  
}

/*
 * Moves according to commands sent from a bluetooth iOS application
 */
void functionality3()
{
  enableMotors();
  //add bluetooth functionality here
}

int acquireMode()
{
  // add the keypad and LCD prompt here
}

void enableMotors()
{
  digitalWrite(E1, HIGH);
  digitalWrite(E2, HIGH);
}

void runTopSpeed()
{
  currentSpeed = topSpeed;
  analogWrite(M1, topSpeed);
  analogWrite(M2, topSpeed);
  evaluateHallSensors();
}

int detectObject()
{
  //detect if there's an object using the ultrasonic sensor  
}
void evaluateHallSensors()
{
  // check hall effect and adjust accordingly
}




