// main.ino

#include "ESP32SoftwareSerial.h"

#define HEATER_PIN 32  // Using GPIO 35 for half-duplex communication with heater

ESP32SoftwareSerial sOne(HEATER_PIN); // Declare the instance of ESP32SoftwareSerial

unsigned long lasttime;   // used to calculate inter character delay

void setup() 
{
  // initialize listening serial port with SoftwareSerial
  // 25000 baud, Tx and Rx channels of Chinese heater comms interface:
  sOne.begin(25000);   
  
  // initialise serial monitor on serial port 0
  Serial.begin(115200);

  // prepare for detecting a long delay
  lasttime = millis();
}

void loop() 
{
  static byte Data[48];
  static bool RxActive = false;
  static int count = 0;

  // read from SoftwareSerial, the "Tx Data" (to heater), send to the serial monitor:
  if (sOne.available()) {
    // calc elapsed time since last rx’d byte to detect start of frame sequence
    unsigned long timenow = millis();
    unsigned long diff = timenow - lasttime;
    lasttime = timenow;
    if(diff > 100) {       // this indicates the start of a new frame sequence
      RxActive = true;
    }
    int inByte = sOne.read(); // read hex byte from SoftwareSerial
    if(RxActive) {
      Data[count++] = inByte;
      if(count == 48) {
        RxActive = false;
      }
    }
  }
  if(count == 48) {  // filled both frames – dump
    count = 0;
    char str[16];
    sprintf(str, "%08d  ", lasttime);
    Serial.print(str);                 // print timestamp
    for(int i=0; i<48; i++) {
      if(i == 0)
        Serial.print("Tx ");           // insert Tx marker on first pass
      if(i == 24)
        Serial.print("Rx ");           // insert Rx marker after first 24 bytes
      sprintf(str, "%02X ", Data[i]);  // make 2 dig hex values
      Serial.print(str);               // and print                         
    }
    Serial.println();                  // newline and done
  }  // count == 48
}  // loop