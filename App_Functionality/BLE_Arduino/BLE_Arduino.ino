#include <SPI.h>
#include "Adafruit_BLE_UART.h"
/*#define ADAFRUITBLE_RST 9
#define interrupt_time 15
#define CLOCK_SPEED 16000000 // BLE clock

#define BLE_MODE 2

const int rdyPin = 6;

byte BLEChar;

char mode;

SPISettings mySetting(CLOCK_SPEED, LSBFIRST, SPI_MODE0);*/

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

byte negative_data_count;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  // BTLEserial.setDeviceName("NEWNAME"); /* 7 characters max! */
  negative_data_count = 0;

  BTLEserial.begin();
}

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop() {
  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      //Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      int c = (int) BTLEserial.read();
      if (c == 255) {
        negative_data_count++;
      } else {
        if (negative_data_count != 0) {
          c = -c;
          negative_data_count--;
        }
        Serial.println(c);
      }
    }

    // Next up, see if we have any data to get from the Serial console

    if (Serial.available()) {
      // Read a line from Serial
      Serial.setTimeout(100); // 100 millisecond timeout
      String s = Serial.readString();

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
  }
}

/*void ISR(void){
  // write REQ pin low to begin transaction
  digitalWrite(slaveSelectPin, LOW);
  //  send in the address and value via SPI:
  - Byte 1 (debug byte) from nRF8001 is an internal debug byte and the application
  controller discards it.
  - Byte 2 (length byte) from nRF8001 defines the length of the message.
  - Byte 3 (ACI byte1) is the first byte of the ACI data.
  - Byte N is the last byte of the ACI data.
  // write REQ pin low to end transaction
  digitalWrite(slaveSelectPin, HIGH);
}

void readBLEData(void) {
  if(timer>0 && timer<interrupt_time){
    BLEChar = digitalRead(A0);
  } else {
    photocellValue = OFF_THRESHOLD+1;
  }
}*/
