#include <SoftwareSerialWithHalfDuplex.h>

SoftwareSerialWithHalfDuplex sOne(2, 2); // NOTE TX & RX are set to same pin for half duplex operation

int optionalEnableDisableSwitch = 8;
unsigned long flasherLastTime;
unsigned long rxLastTime;
unsigned long writerLastTime;
String heaterState[] = {"Off", "Starting", "Pre-Heat", "Failed Start - Retrying", "Ignition - Now heating up", "Running Normally", "Stop heaterCommand Received", "Stopping", "Cooldown"};
int heaterStateNum = 0;
int heaterError = 0;

// Default enable thermostat mode at start up
int controlEnable = 1;

// set the currentTemperature default to something outside the usual range, this will help detect correctly reading the values.
float currentTemperature = -200;

float setTemperature = 0;
float heaterCommand = 0;
void setup()
{
  pinMode(optionalEnableDisableSwitch, INPUT_PULLUP);
  // initialize listening serial port
  // 25000 baud, Tx and Rx channels of Chinese heater comms interface:
  // Tx/Rx data to/from heater, special baud rate for Chinese heater controllers
  sOne.begin(25000);
  // initialise serial monitor on serial port 0 - use 115200
  // lower baud rates for debugging can slow down the diesel heater tx/rx
  Serial.begin(115200);
  // prepare for detecting a long delay
  flasherLastTime = millis();
  rxLastTime = millis();
  writerLastTime = millis();
    // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);      
}

void loop()
{ 
  static byte Data[48];
  static bool RxActive = false;
  static int count = 0;
  // Default heartbeat LED flash length milliseconds
  static long flashLength = 800;

  unsigned long flasherTimeNow = millis();
  unsigned long flasherDiff = flasherTimeNow - flasherLastTime;

  // LED Heartbeat
  // flash speed changes depending
  // 1) standard heart beat when nothing is active
  // 2) fast heart beat when heater has been triggered to turn on
  // 3) slow heart beat when heater has been triggered to turn off
  // These can be used for testing if you do not want to transmit changes to the heater.
  if (controlEnable == 0) {
    flashLength = 800;
  }
  if (flasherDiff > flashLength) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if (flasherDiff > flashLength * 2) {
    digitalWrite(LED_BUILTIN, LOW);
    //Serial.println("Heartbeat");
    flasherLastTime = flasherTimeNow;
  }

  // read from serial on D2
  if (sOne.available()) {
    // calc elapsed time since last rx’d byte to detect start of frame sequence
    unsigned long rxTimeNow = millis();
    unsigned long rxDiff = rxTimeNow - rxLastTime;
    rxLastTime = rxTimeNow;

    if (rxDiff > 100) { // this indicates the start of a new frame sequence
      RxActive = true;
    }
    int inByte = sOne.read(); // read hex byte
    if (RxActive) {
      Data[count++] = inByte;
      if (count == 48) {
        RxActive = false;
      }
    }
  }

  if (count == 48) { // filled both frames – dump
    unsigned long writerTimeNow = millis();
    unsigned long writerDiff = writerTimeNow - writerLastTime;

    count = 0;
    heaterCommand = int(Data[2]);
    currentTemperature = int(Data[3]);
    setTemperature = int(Data[4]);
    heaterStateNum = int(Data[26]);
    heaterError = int(Data[41]);

    // Adjust temperature integer for negative numbers (-1 = 255, -2 = 254)
    // The basis of this code will use operating range -100'c to 155'c, which should be way outside the operating range of the heater
    if (currentTemperature > 155 && currentTemperature <= 255) {
      // currentTemperature will now give an accurate negative value
      currentTemperature = currentTemperature - 256;
      //  0 = 0
      // -1 = 255, so 255-256=-1
      // -2 = 254, so 254-256=-2
    }

    // Data[29] looks like it contains voltage 121 = 12.1v
    // Voltage dips significantly during glowplug use
    // So low voltage detector would need to accomadate this
    // Or thicker wires to the heater maybe required. 

    // For debugging
//    for (int i = 0; i <= 47; i++) {
//      Serial.print("Item ");
//      Serial.print(i);
//      Serial.print(": ");
//      Serial.println(int(Data[i]));
//    }
//    Serial.println();
//    Serial.println("--- Heater Status ---");
//    Serial.print("Command          ");
//    Serial.println(int(heaterCommand));
//    Serial.print("Heater Status    ");
//    if (heaterStateNum >= 0 && heaterStateNum <= 8)
//    {
//      Serial.println(heaterState[heaterStateNum]);
//    }
//    Serial.print("Error Code       ");
//    Serial.println(heaterError);
//    Serial.print("Current Temp     ");
//    Serial.println(currentTemperature);
//    Serial.print("Set Temp         ");
//    Serial.println(setTemperature);
//    Serial.print("Heater State #   ");
//    Serial.println(heaterStateNum);
//    Serial.println();
//    Serial.print("System Enabled : ");
//    Serial.println(controlEnable);

    // On button pressed on unit or remote.
    if (int(heaterCommand) == 160) {
      //Serial.println("Start command seen from controller - Enabling Auto");
      controlEnable = 1;
    }
    // Off button pressed on unit or remote.
    if (int(heaterCommand) == 5) {
      //Serial.println("Stop command seen from controller - Disabling Auto");
      // disable termostat control until next on command seen.
      controlEnable = 0;
    }

    // Make sure variables are being populated by the heater
    // Sometimes the current temp takes a while to establish
    // Use writerDiff to wait 30 seconds between issuing commands
    // Optional Switch is in correct position
    if (controlEnable == 1 && currentTemperature > -100 && setTemperature > 0 && writerDiff > 30000 && digitalRead(optionalEnableDisableSwitch) == HIGH)
    {
      // Inside this block only gets run every 30 seconds
      writerLastTime = writerTimeNow;
      // Start heater if set temp is 2 degree higher than current temp by more than 1 degree and heater state is "[0] stopped"
      if (setTemperature >= (currentTemperature + 2) && heaterError <= 1 && heaterStateNum == 0)  {
        //Serial.println("*** Temp Below Set Limit - Starting Heater ***");
        uint8_t data1[24] = {0x78, 0x16, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x32, 0x08, 0x23, 0x05, 0x00, 0x01, 0x2C, 0x0D, 0xAC, 0x8D, 0x82};
        delay(50);
        sOne.write(data1, 24);
        // Now use 0.1 Second LED Heart Beat
        flashLength = 100;
      }

      // shutdown if current temp is 2 degree warmer than set temp
      if (setTemperature <= (currentTemperature - 2) && heaterStateNum == 5) {
        //Serial.println("*** Temperature Above Upper Limit - Stopping Heater ***");
        uint8_t data1[24] = {0x78, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x32, 0x08, 0x23, 0x05, 0x00, 0x01, 0x2C, 0x0D, 0xAC, 0x61, 0xD6};
        delay(50);
        sOne.write(data1, 24);
        // Now use 3 Second LED Heart Beat
        flashLength = 3000;
      }
    }

  }
} // loop
