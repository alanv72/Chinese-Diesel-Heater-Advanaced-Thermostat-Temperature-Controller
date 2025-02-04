// main.ino

#include "ESP32SoftwareSerial.h"

#define HEATER_PIN 32  // Using GPIO 32 for half-duplex communication with heater
#define LED_BUILTIN 2  // GPIO pin for the blue LED on ESP32 Dev Board

ESP32SoftwareSerial sOne(HEATER_PIN); // Declare the instance of ESP32SoftwareSerial

unsigned long flasherLastTime;
unsigned long rxLastTime;
unsigned long writerLastTime;
String heaterState[] = {"Off/Stand-by", "Starting", "Pre-Heat", "Failed Start - Retrying", "Ignition - Now heating up", "Running Normally", "Stop heaterCommand Received", "Stopping", "Cooldown"};
int heaterStateNum = 0;
int heaterError = 0;
int heaterinternalTemp = 0;
unsigned long lastSendTime = 0;  // New variable to track last send time

// Default enable thermostat mode at start up
int controlEnable = 1;

// set the currentTemperature default to something outside the usual range, this will help detect correctly reading the values.
float currentTemperature = -200;

float setTemperature = 0;
float heaterCommand = 0;

void setup()
{
  pinMode(HEATER_PIN, INPUT); // Set pin to input mode with pull-down for heater communication
  sOne.begin(25000);

  // Serial debug on UART0 (GPIO1 TX, GPIO3 RX)
  Serial.begin(115200); // Initialize serial communication for debugging
  Serial.println("Heater Controller Starting...");
  
  flasherLastTime = millis();
  rxLastTime = millis();
  writerLastTime = millis();
  pinMode(LED_BUILTIN, OUTPUT);      
}

void loop()
{
  static byte Data[48];
  static bool RxActive = false;
  static int count = 0;
  static long flashLength = 800;

  unsigned long currentMillis = millis();
  unsigned long flasherTimeNow = millis();
  unsigned long flasherDiff = flasherTimeNow - flasherLastTime;

  // LED Heartbeat
  if (controlEnable == 0) {
    flashLength = 800;
  }
  if (flasherDiff > flashLength) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if (flasherDiff > flashLength * 2) {
    digitalWrite(LED_BUILTIN, LOW);
    flasherLastTime = flasherTimeNow;
  }

  // Read from serial (heater communication)
  if (sOne.available()) {
    unsigned long rxTimeNow = millis();
    unsigned long rxDiff = rxTimeNow - rxLastTime;
    rxLastTime = rxTimeNow;
    
    int inByte = sOne.read(); // Declare inByte here
    if (inByte == 0x76 && rxDiff > 150) { // Start of new frame with correct declaration
      RxActive = true;
      //Serial.println("New frame detected");
    }
    if (RxActive) {
      Data[count++] = inByte;
      if (count == 48) {
        RxActive = false;
        //Serial.println("Frame complete, processing data...");
      }
    }
  }

  if (count == 48) { // Process data when complete frame received
    unsigned long writerTimeNow = millis();
    unsigned long writerDiff = writerTimeNow - writerLastTime;

    count = 0;
    heaterCommand = int(Data[2]);
    currentTemperature = int(Data[3]);
    setTemperature = int(Data[4]);
    heaterStateNum = int(Data[26]);
    heaterinternalTemp = (int(Data[34]) << 8) | int(Data[35]);
    int glowPlugCurrent = (int(Data[38]) << 8) | int(Data[39]);
    float glowPlugCurrent_Amps = glowPlugCurrent / 100.0; // Convert from 10mA per digit to Amps
    heaterError = int(Data[41]);

    // Adjust temperature for negative values
    if (currentTemperature > 155 && currentTemperature <= 255) {
      currentTemperature -= 256;
    }

    // Format output for better readability
    if (currentMillis - lastSendTime >= 5000) {
      lastSendTime = currentMillis;
      Serial.println("Heater Status:");
      Serial.printf("  Command: %d\n", heaterCommand);
      Serial.printf("  Current Temp: %.2f°C\n", currentTemperature);
      Serial.printf("  Set Temp: %.2f°C\n", setTemperature);
      Serial.printf("  Heater Internal Temp: %d°C\n", heaterinternalTemp);
      Serial.printf("  Glow Plug Current: %.2f A\n", glowPlugCurrent_Amps);
      //Serial.printf("  State: %d - ", heaterStateNum);
      if (heaterStateNum >= 0 && heaterStateNum <= 8) {
        Serial.printf("  State: %s\n", heaterState[heaterStateNum]);
      } else {
        Serial.println("Unknown");
      }
      Serial.printf("  Error: %d\n", heaterError);
    }
    // Handle heater commands
    if (int(heaterCommand) == 160) {
      controlEnable = 1;
      Serial.println("  Heater ON command detected");
    }
    if (int(heaterCommand) == 5) {
      controlEnable = 0;
      Serial.println("  Heater OFF command detected");
    }

    // Control heater based on temperature and conditions
    if (controlEnable == 1 && currentTemperature > -100 && setTemperature > 0 && writerDiff > 30000)
    {
      writerLastTime = writerTimeNow;
      if (setTemperature >= (currentTemperature + 2) && heaterError <= 1 && heaterStateNum == 0) {
      //  uint8_t data1[24] = {0x78, 0x16, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x32, 0x08, 0x23, 0x05, 0x00, 0x01, 0x2C, 0x0D, 0xAC, 0x8D, 0x82};
        uint8_t data1[24] = {0x76, 0x16, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x06, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00};
        sendData(data1, 24);
        flashLength = 100;
        Serial.println("  Starting Heater");
      }
      
      if (setTemperature <= (currentTemperature - 2) && (heaterStateNum == 5 || heaterStateNum == 2)) {
        //uint8_t data1[24] = {0x78, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x32, 0x08, 0x23, 0x05, 0x00, 0x01, 0x2C, 0x0D, 0xAC, 0x61, 0xD6};
        uint8_t data1[24] = {0x76, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x06, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00};
        sendData(data1, 24);
        flashLength = 3000;
        Serial.println("  Stopping Heater");
      }
    }
  }
}

uint16_t calculateCRC16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void sendData(const uint8_t *data, size_t length) {
  uint8_t dataToSend[length];
  memcpy(dataToSend, data, length - 2); // Copy all but the CRC bytes
  uint16_t crc = calculateCRC16(dataToSend, length - 2);
  dataToSend[length - 2] = (crc >> 8) & 0xFF; // MSB
  dataToSend[length - 1] = crc & 0xFF;        // LSB

  for (int i = 0; i < 3; ++i) {
    for (size_t i = 0; i < length; i++) {
      sOne.write(dataToSend[i]); // Write each byte
    }
    Serial.printf("  Sent %d bytes to heater (Attempt %d)\n", length, i+1);
    delay(400);
  }
}
