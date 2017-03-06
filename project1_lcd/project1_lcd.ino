#include <LiquidCrystal.h>
#include <SPI.h>

/* Function Prototypes */
void updateLCD();
void displayMode();
void displaySpeed();

LiquidCrystal lcd(13, 12, 11, 8, 1, 0);

//Sets up LCD display
void setup() {
  lcd.begin(20,4);
  lcd.print("Hello.  My name is Saul.");
}

void loop() {
  /* Continually update LCD */
  updateLCD();
  delay(5);
}

/* Updates the LCD with current mode and speed values */
void updateLCD(){
  displayMode();
  displaySpeed();
}

/* Displays current mode to top row of LCD */
void displayMode(){
  int mode = (SWITCH) ? 1 : 2;
  lcd.setCursor(0,0);
  lcd.write("MODE %d", mode);
}

/* Displays current speed to bottom row of LCD */
void displaySpeed(){
  lcd.setCursor(0,1);
  lcd.write("Speed = %lf", robotSpeed);
}
