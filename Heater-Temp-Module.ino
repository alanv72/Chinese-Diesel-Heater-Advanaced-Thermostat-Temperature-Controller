// main.ino
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include <Preferences.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <index_html.h>
#include "ESP32SoftwareSerial.h"
#include <OneWire.h>           // Add OneWire library for DS18B20
#include <DallasTemperature.h> // Add DallasTemperature library for DS18B20


#define HEATER_PIN 32
#define LED_BUILTIN 2
// Define the pins for increase and decrease temperature actions
const int increaseTempPin = 25;
const int decreaseTempPin = 26;
const unsigned long buttonPressDuration = 250;  // milliseconds for button press
unsigned long lastAdjustmentTime = 0;
const unsigned long adjustmentInterval = 1000;  // 1 second interval between adjustments

// New pin definitions for 5V relays and DY-HFT sensor
#define DUCT_FAN_RELAY_PIN 33  // Pin for duct fan relay
#define WALL_FAN_RELAY_PIN 27  // Pin for wall fan relay
#define DS18B20_SENSOR_PIN 14   // Pin for DY-HFT sensor


ESP32SoftwareSerial sOne(HEATER_PIN);

// Setup OneWire and DallasTemperature for DS18B20
OneWire oneWire(DS18B20_SENSOR_PIN);
DallasTemperature sensors(&oneWire);

// Wi-Fi credentials
const char* primarySSID = "freedom";
const char* primaryPassword = "ontheroadagain!";
const char* fallbackSSID = "littlesugar";
const char* fallbackPassword = "netgearsucks!";

// WiFi connection timing
const unsigned long PRIMARY_CONNECT_TIME = 10000;
const unsigned long FALLBACK_CONNECT_TIME = 20000;
unsigned long wifiConnectStartMillis = 0;
bool tryingPrimary = true;
bool connectedToAnyNetwork = false;
unsigned long bootTime = millis();
unsigned long lastMemoryCheckTime = 0;

// NTP Client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org",-21600, 10800000);

// Your BLE name or hostname for mDNS
String currentBLEName = "RV-DHeat";

AsyncWebServer server(80);
AsyncEventSource events("/events");
Preferences preferences;
unsigned long heaterRunTime = 0;
bool eventen = true;
float fuelConsumption = 0.0;                  // Lifetime fuel consumption in ml
float tankRuntime = 0.0;                      // Runtime of the current tank in seconds
float tankSizeGallons = 0.0;                  // Size of the tank in gallons, user-settable
float tankConsumption = 0.0;
float totalTankTime = 0.0;                  // Fuel consumption since last tank reset in ml
const float ML_TO_GALLON = 0.000264172;       // Conversion factor from ml to gallons
const float PUMP_FLOW_PER_1000_PUMPS = 22.0;  // ml per 1000 pumps
float pumpHz = 0.0;                           // Real-time pump frequency
// Global variable for uptime
unsigned long uptime = 0;
float avgGallonPerHour = 0.0;    // Average gallons per hour
float currentGPH = 0.0;
float rollingAvgGPH = 0.0; // New variable for rolling average GPH
const float alpha = 0.1;   // Smoothing factor for EMA, you can adjust this
unsigned long totalRuntime = 0;  // Total runtime in seconds to calculate average
int fanSpeed = 0;                // Fan speed in RPM
float supplyVoltage = 0.0;       // Supply voltage in volts
bool frostModeEnabled = false;
float glowPlugHours = 0.0;  // Hours of glow plug operation
float glowPlugCurrent_Amps = 0.0;
bool voltagegood = true;
unsigned long ductfandelay = 0;
unsigned long wallfandelay = 0;
bool ductfan = 0;
bool wallfan = 0;
bool cshut = 0;
String message = "";
#define TEMP_HISTORY_SIZE 720 // 12 hours * (60 minutes / 5 minutes per update) = 720 entries
#define INTERVAL_BETWEEN_SAVES 300000
float tempHistory[TEMP_HISTORY_SIZE];
unsigned long tempTimestamps[TEMP_HISTORY_SIZE];
int tempIndex = 0;
#define VOLTAGE_HISTORY_SIZE 720  // Same as temperature: 12 hours of data
float voltageHistory[VOLTAGE_HISTORY_SIZE];
unsigned long voltageTimestamps[VOLTAGE_HISTORY_SIZE];
int voltageIndex = 0;
// #define SAMPLE_SIZE 480 // 8 hours * 60 samples/hour
// float gphSamples[SAMPLE_SIZE] = {0};
// unsigned long sampleTimestamps[SAMPLE_SIZE] = {0};
// int sampleIndex = 0;
// unsigned long lastSampleTime = 0;

unsigned long flasherLastTime;
unsigned long rxLastTime;
unsigned long writerLastTime;
String heaterState[] = { "Off/Stand-by", "Starting", "Pre-Heat", "Retry Start", "Ramping Up", "Heating", "Stop Received", "Shutting Down", "Cooldown"};
int heaterStateNum = 0;
String heaterError[] = {
  "No Error",                     // 0 - No Error (0 - 1 = -1, but we treat 0 as no error)
  "No Error, running",        // 1 - No Error, but started (1 - 1 = 0)
  "Voltage too low",              // 2 - Voltage too low (2 - 1 = 1)
  "Voltage too high",             // 3 - Voltage too high (3 - 1 = 2)
  "Ignition plug failure",        // 4 - Ignition plug failure (4 - 1 = 3)
  "Pump Failure – over current",  // 5 - Pump Failure (5 - 1 = 4)
  "Overheating",                  // 6 - Too hot (6 - 1 = 5)
  "Motor Failure",                // 7 - Motor Failure (7 - 1 = 6)
  "Comms lost",       // 8 - Serial connection lost (8 - 1 = 7)
  "Fire Out",         // 9 - Fire is extinguished (9 - 1 = 8)
  "Temp sensor failed",    // 10 - Temperature sensor failure (10 - 1 = 9)
  "Multiple Starts Failed. Check Fuel." // Fuel failure. Empty or restricted. (11 - 1 = 10)
};
int heaterErrorNum = 0;
int heaterinternalTemp = 0;
unsigned long lastSendTime = 0;

#define FRAME_SIZE 24           // Frames are 24 bytes
#define COMBINED_FRAME_SIZE 48  // 2 frames combined
uint8_t frameBuffer[FRAME_SIZE] = { 0 };
uint8_t combinedFrame[COMBINED_FRAME_SIZE] = { 0 };  // To store both frames
int frameIndex = 0;
bool RxActive = false;
bool commandFrameReceived = false;  // Track if we've received a command frame

// Function prototypes
bool validateCRC(uint8_t* frame);
uint16_t calculateCRC16(const uint8_t* data, size_t length);
void processFrame(uint8_t* frame);

// Default enable thermostat mode at start up
int controlEnable = 1;

// set the currentTemperature default to something outside the usual range
float currentTemperature = -200.0f;

float setTemperature = 0;
float heaterCommand = 0;
float targetSetTemperature = 0;  // Global variable to store the target temperature set from the web interface
bool temperatureChangeByWeb = false;

// Variable to store DS18B20 temperature
float walltemp = 0.0;
float walltemptrigger = 100;

// Convert Celsius to Fahrenheit
float celsiusToFahrenheit(float celsius) {
  return (celsius * 9.0 / 5.0) + 32.0;
}

// Convert Fahrenheit to Celsius for heater command
float fahrenheitToCelsius(float fahrenheit) {
  return (fahrenheit - 32) * 5.0 / 9.0;
}

String getFormattedDate() {
  time_t rawtime = timeClient.getEpochTime();
  struct tm* ti;
  ti = localtime(&rawtime);

  uint16_t year = ti->tm_year + 1900;
  String yearStr = String(year);
  uint8_t month = ti->tm_mon + 1;
  String monthStr = month < 10 ? "0" + String(month) : String(month);
  uint8_t day = ti->tm_mday;
  String dayStr = day < 10 ? "0" + String(day) : String(day);

  return monthStr + "/" + dayStr + "/" + yearStr;
}

// WiFi connection function
void connectToWiFi() {
  WiFi.begin(primarySSID, primaryPassword);
  Serial.println("Connecting to primary WiFi...");
  wifiConnectStartMillis = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiConnectStartMillis < PRIMARY_CONNECT_TIME) {
    delay(1000);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nPrimary WiFi connection failed, trying fallback...");
    WiFi.begin(fallbackSSID, fallbackPassword);
    wifiConnectStartMillis = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiConnectStartMillis < FALLBACK_CONNECT_TIME) {
      delay(1000);
      Serial.print(".");
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    timeClient.begin();
    connectedToAnyNetwork = true;
    if (!MDNS.begin(currentBLEName.c_str())) {
      Serial.println("Error setting up MDNS responder!");
    } else {
      Serial.println("mDNS responder started");
      MDNS.addService("http", "tcp", 80);
    }
  } else {
    Serial.println("Failed to connect to any WiFi network.");
    connectedToAnyNetwork = false;
  }
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting reconnection...");
    connectToWiFi();
  }
}

void sendData(const uint8_t* data, size_t length) {
  uint8_t dataToSend[length];
  memcpy(dataToSend, data, length - 2);
  uint16_t crc = calculateCRC16(dataToSend, length - 2);
  dataToSend[length - 2] = (crc >> 8) & 0xFF;
  dataToSend[length - 1] = crc & 0xFF;

  for (int i = 0; i < 3; ++i) {
    for (size_t i = 0; i < length; i++) {
      sOne.write(dataToSend[i]);
    }
    Serial.printf("  Sent %d bytes to heater (Attempt %d)\n", length, i + 1);
    delay(400);
  }
}

// File upload handler
void handleUpload(AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
  String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
  Serial.println(logmessage);

  if (!index) {
    logmessage = "Upload Start: " + String(filename);
    request->_tempFile = SPIFFS.open("/" + filename, "w");
    Serial.println(logmessage);
  }

  if (len) {
    request->_tempFile.write(data, len);
    logmessage = "Writing file: " + String(filename) + " index=" + String(index) + " len=" + String(len);
    Serial.println(logmessage);
  }

  if (final) {
    logmessage = "Upload Complete: " + String(filename) + ",size: " + String(index + len);
    request->_tempFile.close();
    Serial.println(logmessage);
    request->redirect("/");
  }
}

void setup() {
  esp_task_wdt_deinit();  //wdt is initialized by default. disable and reconfig
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 120000,                             // 2m timeout
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // All cores
    .trigger_panic = false                            // Do not trigger panic on timeout, just warn
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
 
  for (int i = 0; i < TEMP_HISTORY_SIZE; i++) {
    tempHistory[i] = -200.0; // Initialize with an out-of-range value
    tempTimestamps[i] = 0;
  }

  for (int i = 0; i < VOLTAGE_HISTORY_SIZE; i++) {
    voltageHistory[i] = -1.0;  // Initialize with an invalid value
    voltageTimestamps[i] = 0;
  }

  pinMode(increaseTempPin, OUTPUT);
  pinMode(decreaseTempPin, OUTPUT);
  // Ensure pins are not connected to ground at start
  digitalWrite(increaseTempPin, HIGH);
  digitalWrite(decreaseTempPin, HIGH);

  // Setup new pins for relays and DS18B20 sensor
  pinMode(DUCT_FAN_RELAY_PIN, OUTPUT);
  pinMode(WALL_FAN_RELAY_PIN, OUTPUT);
  digitalWrite(DUCT_FAN_RELAY_PIN, LOW); // Initially off (assuming LOW turns relay off)
  digitalWrite(WALL_FAN_RELAY_PIN, LOW); // Initially off
  sensors.begin();
  pinMode(HEATER_PIN, INPUT);
  sOne.begin(25000);

  Serial.begin(115200);
  Serial.println("Heater Controller Starting...");

  flasherLastTime = millis();
  rxLastTime = millis();
  writerLastTime = millis();
  pinMode(LED_BUILTIN, OUTPUT);

  connectToWiFi();
  timeClient.begin();

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  Serial.println("SPIFFS mounted successfully");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index_html", "text/html");
  });

  server.on("/fallback", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/toggleThermostat", HTTP_POST, [](AsyncWebServerRequest* request) {
    String enable = request->arg("enable");
    controlEnable = (enable == "true") ? 1 : 0;
    preferences.putInt("controlEnable", controlEnable);  // Save state
    request->send(200, "text/plain", "Thermostat control toggled");

    // Optionally send an update immediately to reflect the change
    DynamicJsonDocument jsonDoc(1024);
    jsonDoc["controlEnable"] = controlEnable;
    // ... other status data ...
    String jsonString;
    serializeJson(jsonDoc, jsonString);
    String eventString = "event: heater_update\ndata: " + jsonString + "\n\n";
    if (eventen) {
      events.send(eventString.c_str());
    }
  });

  server.on("/setWallTempTrigger", HTTP_POST, [](AsyncWebServerRequest* request) {
      String temp = request->arg("trigger");
      float triggerTempDeltaF = temp.toFloat();

      // Convert delta from Fahrenheit to Celsius
      float triggerTempDeltaC = triggerTempDeltaF * 5.0 / 9.0;
      walltemptrigger = triggerTempDeltaC;
      // Store the delta in Celsius for internal use
      preferences.putFloat("walltemptrigger", triggerTempDeltaC); // Save the delta in Celsius

      request->send(200, "text/plain", "Wall temperature trigger delta updated");
  });
  
  server.on("/settemp", HTTP_POST, [](AsyncWebServerRequest* request) {
    String temp = request->arg("temp");
    float targetTempF = temp.toFloat();

    // Convert to Celsius, round to the nearest whole number, and store as an int
    int targetTempC = round(fahrenheitToCelsius(targetTempF));

    // Update targetSetTemperature, which is now an integer
    targetSetTemperature = targetTempC;

    // Flag that this change was initiated by the web interface
    temperatureChangeByWeb = true;
    controlEnable = 1;
    request->send(200, "text/plain", "New target temperature set.");
  });

  server.on(
    "/upload", HTTP_POST, [](AsyncWebServerRequest* request) {
      request->send(200);
    },
    handleUpload);

  server.on("/events", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse(200, "text/event-stream", "");
    response->addHeader("Cache-Control", "no-cache");
    response->addHeader("Connection", "keep-alive");
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
    Serial.println("Event stream response sent...");
  });

  server.on("/primepump", HTTP_POST, [](AsyncWebServerRequest* request) {
    String action = request->arg("action");
    if (action == "start") {
      uint8_t data1[24] = { 0x76, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x5A, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
      sendData(data1, 24);
      request->send(200, "text/plain", "Pump priming started.");
    } else if (action == "stop") {
      uint8_t data1[24] = { 0x76, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
      sendData(data1, 24);
      request->send(200, "text/plain", "Pump priming stopped.");
    } else {
      request->send(400, "text/plain", "Invalid action");
    }
  });

  server.on("/frostMode", HTTP_POST, [](AsyncWebServerRequest* request) {
    String enable = request->arg("enable");
    frostModeEnabled = (enable == "true");
    preferences.putBool("frostMode", frostModeEnabled);  // Save immediately
    request->send(200, "text/plain", "Frost Mode updated");
  });

  server.on("/listfiles", HTTP_GET, [](AsyncWebServerRequest* request) {
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    String fileListJSON = "{\"files\":[";
    bool first = true;

    while (file) {
      if (!first) {
        fileListJSON += ",";
      } else {
        first = false;
      }
      fileListJSON += "{\"name\":\"" + String(file.name()) + "\",\"size\":" + String(file.size()) + "}";
      file = root.openNextFile();
    }
    fileListJSON += "]}";

    request->send(200, "application/json", fileListJSON);
  });

  server.on("/setTankSize", HTTP_POST, [](AsyncWebServerRequest* request) {
    String sizeStr = request->arg("size");
    tankSizeGallons = sizeStr.toFloat();
    preferences.putFloat("tankSize", tankSizeGallons);
    request->send(200, "text/plain", "Tank size updated");
  });

  server.on("/resetTank", HTTP_POST, [](AsyncWebServerRequest* request) {
    tankRuntime = 0;
    tankConsumption = 0;
    controlEnable = 1;
    totalTankTime = 0;
    preferences.putFloat("tankRuntime", tankRuntime);
    preferences.putFloat("tankConsumption", tankConsumption);
    request->send(200, "text/plain", "Tank reset");
  });

  server.on("/resetTotals", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("confirm")) { // Check if the 'confirm' parameter is present
    // Reset the variables
    fuelConsumption = 0.0;
    glowPlugHours = 0.0;
    heaterRunTime = 0;

    // Save the reset values to persistent storage
    preferences.putFloat("fuelConsumption", fuelConsumption);
    preferences.putFloat("glowPlugHours", glowPlugHours);
    preferences.putULong("runtime", heaterRunTime);

    // Send a response
    request->send(200, "text/plain", "Fuel consumption, glow plug hours, and heater runtime reset");
    } else {
      // Send an error response if 'confirm' is not provided
      request->send(400, "text/plain", "Confirmation required.");
    }
  });

  server.on("/shutdownHeater", HTTP_POST, [](AsyncWebServerRequest *request){
    // Turn off the heater here by setting controlEnable to 0 or similar logic
    controlEnable = 0;
    uint8_t data1[24] = { 0x76, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
    sendData(data1, 24); // Assuming sendData is your function to send commands to the heater
    cshut = 1;
    Serial.println("Heater shutdown command received");
    request->send(200, "text/plain", "Heater shutdown command processed");
  });

  server.onNotFound([](AsyncWebServerRequest *request){
    if (request->method() == HTTP_GET) {
      String path = request->url();
      if (SPIFFS.exists(path)) {
        request->send(SPIFFS, path, String(), false);
      } else {
        request->send(404, "text/plain", "File Not Found");
      }
    }
  });

  ElegantOTA.begin(&server);
  ElegantOTA.onStart([]() {
    Serial.println("OTA Update Start");
    eventen = false;
    // Stop listening on sOne if necessary
    pinMode(HEATER_PIN, INPUT_PULLDOWN);  // Keep the heater pin low during OTA update
  });

  ElegantOTA.onEnd([](bool success) {
    pinMode(HEATER_PIN, INPUT);  // Assuming it should go back to being an input
    eventen = true;
    Serial.println("OTA Update End");
    Serial.print("Update ");
    Serial.print(success ? "Succeeded" : "Failed");
    Serial.println();
  });

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n",
      client->lastId());
    }
  // send event with message "hello!", id current millis
  // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });

  server.addHandler(&events);
  server.begin();
  Serial.println("HTTP server started");

  preferences.begin("heater", false);
  heaterRunTime = preferences.getULong("runtime", 0);
  fuelConsumption = preferences.getFloat("fuelConsumption", 0);
  tankRuntime = preferences.getFloat("tankRuntime", 0);
  tankSizeGallons = preferences.getFloat("tankSize", 0);
  tankConsumption = preferences.getFloat("tankConsumption", 0);
  avgGallonPerHour = preferences.getFloat("avgGallonPerHour", 0.0);
  totalRuntime = preferences.getULong("totalRuntime", 0);
  glowPlugHours = preferences.getFloat("glowPlugHours", 0.0);
  frostModeEnabled = preferences.getBool("frostMode", false);
  walltemptrigger = preferences.getFloat("walltemptrigger", 0);
  totalTankTime = preferences.getFloat("totalTankTime", 0.0);
  rollingAvgGPH = preferences.getFloat("rollingAvgGPH", 0.0);
  // preferences.begin("tempHistory", false);  // false for read/write mode
  // loadTempHistory();
}

void loop() {
  if (esp_task_wdt_status(NULL) == ESP_ERR_TIMEOUT) {
    Serial.println("Warning: Task Watchdog Timer timeout detected!");
    // Reset the WDT to prevent repeated warnings
    esp_task_wdt_reset();
  } else {
    // Reset WDT if it hasn't triggered
    esp_task_wdt_reset();
  }

  ElegantOTA.loop();

  checkWiFiConnection();
  timeClient.update();
  static bool firstRun = true;
  uptime = millis() - bootTime;
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

  unsigned long tankcurrentMillis = millis();
  static unsigned long prevMillis = 0;
  float elapsedSeconds = (tankcurrentMillis - prevMillis) / 1000.0;
  prevMillis = tankcurrentMillis;

  totalTankTime += elapsedSeconds;

  static byte Data[48];  // For compatibility with existing code
  static int count = 0;

  if (sOne.available() && eventen) {
    // unsigned long rxTimeNow = millis();
    // unsigned long rxDiff = rxTimeNow - rxLastTime;
    // rxLastTime = rxTimeNow;
    int inByte = sOne.read();
    if (inByte == 0x76) {
      RxActive = true;
      frameIndex = 0;  // Reset frame index
    }
    if (RxActive) {
      frameBuffer[frameIndex++] = inByte;
      if (frameIndex == FRAME_SIZE) {
        // Print the frame buffer
        // Serial.print("Frame Received: ");
        // for (int i = 0; i < FRAME_SIZE; i++) {
        //   if (frameBuffer[i] < 0x10) Serial.print("0");  // Add leading zero for single digit hex
        //   Serial.print(frameBuffer[i], HEX);
        //   Serial.print(" ");
        // }
        // Serial.println();  // New line for better readability

        processFrame(frameBuffer);  // Process the frame
        RxActive = false;
        frameIndex = 0;  // Reset for next frame
      }
    }
  }

  if (combinedFrame[0] == 0x76 && combinedFrame[FRAME_SIZE] == 0x76) {  // Check if we have both frames
    memcpy(Data, combinedFrame, COMBINED_FRAME_SIZE);                   // Copy into Data for existing logic
    count = COMBINED_FRAME_SIZE;                                        // Indicate we have a full combined frame
    unsigned long writerTimeNow = millis();
    unsigned long writerDiff = writerTimeNow - writerLastTime;
    //count = 0;
    heaterCommand = int(Data[2]);
    currentTemperature = int(Data[3]);
    setTemperature = int(Data[4]);
    heaterStateNum = int(Data[26]);
    fanSpeed = (int(Data[30]) << 8) | int(Data[31]);
    supplyVoltage = ((int(Data[28]) << 8) | int(Data[29])) * 0.1;
    heaterinternalTemp = (int(Data[34]) << 8) | int(Data[35]);
    int glowPlugCurrent = (int(Data[38]) << 8) | int(Data[39]);
    glowPlugCurrent_Amps = glowPlugCurrent / 100.0;
    pumpHz = int(Data[40] * 0.1);  //Convert to Hz
    heaterErrorNum = int(Data[27]);

    memset(combinedFrame, 0, COMBINED_FRAME_SIZE);
    static unsigned long lastMeterUpdate = 0;
    
    if (heaterErrorNum > 1 && controlEnable ) {
      controlEnable = 0;  // Disable control
      Serial.println("Heater control turned off due to error.");
      message = "Heater control turned off due to error.";
      // Set LED to constant ON to indicate low voltage shutdown
      flashLength = 1000000;  // A very long time to make it effectively constant ON
      digitalWrite(LED_BUILTIN, HIGH);
    }

    if (heaterStateNum >= 6 && !cshut && controlEnable) {
      controlEnable = 0;  // Disable control
      Serial.println("Heater shutdown unexpectedly. Disabling Thermostat.");
      message = "Heater shutdown unexpectedly. Disabling Thermostat.";
      flashLength = 1000000;  // A very long time to make it effectively constant ON
      digitalWrite(LED_BUILTIN, HIGH);
    }

    // Check supplyVoltage
    if (supplyVoltage >= 11.2) {
        voltagegood = true;  // Only go true once voltage is above 11.5
    } else if (supplyVoltage <= 10.5) {
        voltagegood = false;  // Go false if voltage dips below or equal to 10.5
    }

    // Adjust temperature for negative values
    if (currentTemperature > 155.0f && currentTemperature <= 255.0f) {
      currentTemperature -= 256.0f;
    }
    if (firstRun) {
      targetSetTemperature = setTemperature;
      firstRun = false;
    }

    if (frostModeEnabled) {
      float currentTempF = celsiusToFahrenheit(currentTemperature);  // Assuming currentTemperature is in Celsius
      if (currentTempF < 35.0) {
        // Set the temperature to the minimum setting (assuming min is 46°F from your slider)
        setTemperature = fahrenheitToCelsius(46.0);  // Convert min Fahrenheit to Celsius
        temperatureChangeByWeb = true;
        // Turn on the heater if it's not already on
        if (heaterStateNum == 0) {  // If heater is off or in standby
          uint8_t data1[24] = { 0x76, 0x16, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
          sendData(data1, 24);
          Serial.println("Frost Mode: Starting Heater");
          message = "Front Mode Start";
        }
      }
    }
    
    static float lastSetTemperature = setTemperature;
    if (setTemperature != lastSetTemperature) {
      lastSetTemperature = setTemperature;
      // If setTemperature changed from the controller, reset the flag
      if (!temperatureChangeByWeb) {
        // Reset targetSetTemperature to match controller if not changed by web
        targetSetTemperature = (int)setTemperature;  // Assuming setTemperature might be float
      }
    }

    // Format output for better readability
    if (currentMillis - lastSendTime >= 5000) {
      lastSendTime = currentMillis;
      Serial.println("Heater Status:");
      Serial.printf("  Current Time: %s\n",timeClient.getFormattedTime());
      Serial.printf("  Date: %s\n",getFormattedDate());
      Serial.printf("  Command: %d\n", heaterCommand);
      Serial.printf("  Current Temp: %.2f°C\n", currentTemperature);
      Serial.printf("  Set Temp: %.2f°C\n", setTemperature);
      Serial.printf("  Heater Internal Temp: %d°C\n", heaterinternalTemp);
      Serial.printf("  Glow Plug Current: %.2f A\n", glowPlugCurrent_Amps);
      Serial.printf("  Heater Hour Meter: %.2f Hrs\n", heaterRunTime / 3600.0);
      Serial.printf("  Pump Hz: %.2f Hz\n", pumpHz);
      Serial.printf("  Fan Speed: %d RPM\n", fanSpeed);
      Serial.printf("  Supply Voltage: %.1f V\n", supplyVoltage);
//      Serial.printf("  Glow Plug Hours: %.2f Hrs\n", glowPlugHours);
      if (heaterStateNum >= 0 && heaterStateNum <= 8) {
        Serial.printf("  State: %s\n", heaterState[heaterStateNum]);
      } else {
        Serial.println("Unknown");
      }
      Serial.printf("  Error: %s\n", heaterError[heaterErrorNum]);
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
    if (controlEnable == 1 && currentTemperature > -100 && setTemperature > 0 && writerDiff > 30000) {
      writerLastTime = writerTimeNow;
      if (setTemperature >= (currentTemperature + 2) && heaterErrorNum <= 1 && heaterStateNum == 0) {
        uint8_t data1[24] = { 0x76, 0x16, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
        sendData(data1, 24);
        flashLength = 100;
        Serial.println("  Starting Heater");
      }

      if (setTemperature <= (currentTemperature - 3) && (heaterStateNum == 5 || heaterStateNum == 2)) {
        uint8_t data1[24] = { 0x76, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
        sendData(data1, 24);
        cshut = 1;
        flashLength = 3000;
        Serial.println("  Stopping Heater");
      }
    }
   
    if (temperatureChangeByWeb && targetSetTemperature != setTemperature) {
      adjustTemperatureToTarget();
    } else if (temperatureChangeByWeb && targetSetTemperature == setTemperature) {
      // Reset the flag only when the adjustment is complete
      temperatureChangeByWeb = false;
    }
    // LED behavior outside of frame processing
    unsigned long flasherTimeNow = millis();
    unsigned long flasherDiff = flasherTimeNow - flasherLastTime;

    if (flasherDiff > flashLength) {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    if (flasherDiff > flashLength * 2) {
      digitalWrite(LED_BUILTIN, LOW);
      flasherLastTime = flasherTimeNow;
    }
  }
 
  // Read DS18B20 temperature
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead >= 2000) { // Read every 2 seconds
    sensors.requestTemperatures();
    walltemp = sensors.getTempCByIndex(0); // Get temperature in Celsius
    if (walltemp == DEVICE_DISCONNECTED_C) {
      Serial.println("Error: DS18B20 sensor disconnected");
    } else {
      Serial.printf("  Wall Temp: %d°F\n", walltemp);
    }

    // Control for duct fan
    if ((fanSpeed >= 2500 && heaterinternalTemp > 66 && voltagegood) || 
        (fanSpeed == 0 && heaterinternalTemp >= 45)) {
      digitalWrite(DUCT_FAN_RELAY_PIN, HIGH);
      ductfan = 1;
      ductfandelay = 0; // Reset delay if conditions are met for turning on
    } else {
      if (ductfandelay == 0) {
        ductfandelay = millis() + 30000; // Set delay for 30secs
      } else if (millis() >= ductfandelay) {
        digitalWrite(DUCT_FAN_RELAY_PIN, LOW);
        ductfan = 0;
      }
    }

    // Control for wall fan
    float gapC;
    if (walltemptrigger >= -1 && walltemptrigger <= 10) {
      gapC = walltemptrigger; // Use the set trigger if it's within the valid range
    } else {
      gapC = 3; // Default to 3 degrees Celsius if the trigger is out of range
    }

    if (walltemp > -55 && walltemp < 125) { // Valid range for DS18B20
      if (currentTemperature != -200.0f) {
        if (walltemp >= currentTemperature + gapC && voltagegood) { 
          digitalWrite(WALL_FAN_RELAY_PIN, HIGH);
          wallfan = 1;
          wallfandelay = 0; // Reset delay if conditions are met for turning on
        } else {
          if (wallfandelay == 0) {
            wallfandelay = millis() + 600000; // Set delay for 10 minutes (600000 milliseconds)
          } else if (millis() >= wallfandelay) {
            digitalWrite(WALL_FAN_RELAY_PIN, LOW);
            wallfan = 0;
          }
        }
      }  
    }

    lastSensorRead = millis();
  }
  
  // && millis() > wallfandelay
  esp_task_wdt_reset();
  
  // Send heater status update
  static unsigned long lastEvent = 0;
  if (millis() - lastEvent >= 2000) {
    lastEvent = millis();

    // Update runtime if heater is in one of the running states
    if (heaterStateNum >= 2 && heaterStateNum <= 5) {
      heaterRunTime += 2;  // Add 2 seconds to runtime
    }
    // Update runtime and calculate average if pump is running
    // Calculate fuel consumption for this 2-second cycle
    float pumpsPerCycle = pumpHz * 2; // Pumps in 2 seconds
    float cycleFuel = (pumpsPerCycle / 1000.0) * PUMP_FLOW_PER_1000_PUMPS; // Convert Hz to ml per cycle
    // Update fuel consumption and runtime only if pump is running
    if (pumpHz > 0) {
      fuelConsumption += cycleFuel;
      tankConsumption += cycleFuel; // Add to tank consumption

      // Update runtime since pump is running
      tankRuntime += 2;  // Add 2 seconds to tank runtime
      
      // Calculate instantaneous GPH over 2 seconds
      currentGPH = (cycleFuel * ML_TO_GALLON) * 3600.0 / 2.0; // Adjusted for 2-second cycle
    } else {
      currentGPH = 0;
    }
    
    if (glowPlugCurrent_Amps > 0.5) {
        glowPlugHours += 2.0 / 3600.0;                // Increment by 2 second in hours
    }
    
    // Update rolling average using EMA
    rollingAvgGPH = (alpha * currentGPH) + ((1 - alpha) * rollingAvgGPH);
    
    // Calculate average GPH using totalTankTime rather than tankRuntime
    if (totalTankTime > 0) {
        float gallonsUsed = tankConsumption * ML_TO_GALLON;
        avgGallonPerHour = (gallonsUsed * 3600) / totalTankTime;
    } else {
        avgGallonPerHour = 0.0;
    }
    // Calculate remaining fuel in gallons
    float remainingFuelGallons = tankSizeGallons - (tankConsumption * ML_TO_GALLON);

    // Estimate remaining runtime in hours
    float remainingRuntimeHours = (remainingFuelGallons / avgGallonPerHour);
    float rollingRuntimeHours = (remainingFuelGallons / rollingAvgGPH);

    // Check for low fuel condition
    float fuelUsedPercentage = (tankConsumption * ML_TO_GALLON) / tankSizeGallons;
    if (fuelUsedPercentage >= 0.98) { // 98% of tank size
      Serial.println("Fuel level critically low. Shutting down heater.");
      
      // Turn off the heater
      controlEnable = 0;
      uint8_t data1[24] = { 0x76, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
      sendData(data1, 24);
      cshut = 1;
    }
    
    // Check if supply voltage is below safe threshold
    String voltageWarning = "";
    if ((heaterStateNum >= 0 && heaterStateNum <= 4) && supplyVoltage <= 11.0) {
      voltageWarning = "Start volt sag";
    } else if (supplyVoltage <= 10.2 && controlEnable == 1 && heaterStateNum == 5) {
      // Turn off the heater if it's currently enabled
      uint8_t data1[24] = { 0x76, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
      sendData(data1, 24);
      cshut = 1;
      controlEnable = 0;  // Disable control
      voltageWarning = "Low voltage!";
      Serial.println("Heater turned off due to low voltage: " + String(supplyVoltage, 1) + "V");
      message = "Heater turned off due to low votage.";
      // Set LED to constant ON to indicate low voltage shutdown
      flashLength = 1000000;  // A very long time to make it effectively constant ON
      digitalWrite(LED_BUILTIN, HIGH);
    }

    Serial.println("\nFuel Report:");
    Serial.printf("  Fuel Consumed Lifetime: %.2f gallons\n", fuelConsumption * ML_TO_GALLON);
    Serial.printf("  Fuel Consumed Tank: %.2f gallons\n", tankConsumption * ML_TO_GALLON);
    Serial.printf("  Current GPH: %.2f\n", (pumpHz / 1000.0) * PUMP_FLOW_PER_1000_PUMPS * 3600 * ML_TO_GALLON);
    Serial.printf("  Average GPH: %.2f\n", avgGallonPerHour);
    Serial.printf("  Tank Size: %.2f gallons\n", tankSizeGallons);
    Serial.printf("  Tank Runtime: %.2f Hrs\n", tankRuntime / 3600.0);

    // Update event data for web interface
    DynamicJsonDocument jsonDoc(1024);
    jsonDoc["currentTemp"] = celsiusToFahrenheit(currentTemperature);
    jsonDoc["setTemp"] = celsiusToFahrenheit(setTemperature);
    jsonDoc["state"] = heaterState[heaterStateNum];
    jsonDoc["error"] = heaterError[heaterErrorNum];
    jsonDoc["errornum"] = heaterErrorNum;
    jsonDoc["heaterHourMeter"] = heaterRunTime / 3600.0;
    jsonDoc["uptime"] = uptime / 1000;
    jsonDoc["time"] = timeClient.getFormattedTime();
    jsonDoc["date"] = getFormattedDate();
    jsonDoc["fuelConsumedLifetime"] = fuelConsumption * ML_TO_GALLON;
    jsonDoc["fuelConsumedTank"] = tankConsumption * ML_TO_GALLON;
    jsonDoc["currentUsage"] = currentGPH;
    jsonDoc["averageGPH"] = avgGallonPerHour;
    jsonDoc["fanSpeed"] = fanSpeed;
    jsonDoc["supplyVoltage"] = supplyVoltage;
    jsonDoc["voltageWarning"] = voltageWarning;
    jsonDoc["glowPlugHours"] = glowPlugHours;
    jsonDoc["frostMode"] = frostModeEnabled;
    jsonDoc["tankSizeGallons"] = tankSizeGallons;
    jsonDoc["tankRuntime"] = tankRuntime / 3600;
    jsonDoc["remainingRuntimeHours"] = remainingRuntimeHours;
    jsonDoc["rollingAvgGPH"] = rollingAvgGPH;
    const float epsilon = 0.0001f;
    jsonDoc["rollingRuntimeHours"] = (rollingRuntimeHours > 2190.0 || isnan(rollingRuntimeHours) || abs(rollingRuntimeHours) < epsilon) ? JsonVariant() : rollingRuntimeHours;
    jsonDoc["heaterinternalTemp"] = heaterinternalTemp;
    jsonDoc["glowPlugCurrent_Amps"] = glowPlugCurrent_Amps;
    jsonDoc["pumpHz"] = pumpHz;
    jsonDoc["controlEnable"] = controlEnable;
    jsonDoc["walltemp"] = celsiusToFahrenheit(walltemp);
    jsonDoc["walltemptrigger"] = (walltemptrigger * 9.0 / 5.0);
    jsonDoc["ductfan"] = ductfan;
    jsonDoc["wallfan"] = wallfan;
    jsonDoc["voltagegood"] = voltagegood;
    jsonDoc["ductfandelay"] = max(0UL, (ductfandelay - millis()) / 1000UL); // Convert to seconds, ensuring no negative values
    jsonDoc["wallfandelay"] = max(0UL, (wallfandelay - millis()) / 1000UL); // Convert to seconds, ensuring no negative values
    jsonDoc["tempHistory"] = serializeTempHistory();
    jsonDoc["voltageHistory"] = serializeVoltageHistory();
    jsonDoc["message"] = message;

    String jsonString;
    serializeJson(jsonDoc, jsonString);
    //Serial.printf("JSON String length: %d\n", jsonString.length());

    String escapedJsonString = "";
    for (int i = 0; i < jsonString.length(); i++) {
      if (jsonString[i] == '\n') {
        escapedJsonString += "\\n";
      } else if (jsonString[i] == '\r') {
        escapedJsonString += "\\r";
      } else {
        escapedJsonString += jsonString[i];
      }
    }

    // Correct format for Server-Sent Events
    if (eventen) {
      String eventString = "event: heater_update\ndata: " + escapedJsonString + "\n\n";
      events.send(eventString.c_str());
      // Serial.println("  Sending:\n" + jsonString);  
    }

    // Periodically save persistent data
    static unsigned long lastSave = 0;
    if (millis() - lastSave >= 30000) {  // Save every 30sec
      preferences.putFloat("fuelConsumption", fuelConsumption);
      preferences.putFloat("tankRuntime", tankRuntime);
      preferences.putFloat("tankConsumption", tankConsumption);
      preferences.putFloat("avgGallonPerHour", avgGallonPerHour);
      preferences.putULong("totalRuntime", totalRuntime);
      preferences.putFloat("glowPlugHours", glowPlugHours);
      preferences.putBool("frostMode", frostModeEnabled);
      preferences.putULong("runtime", heaterRunTime);
      preferences.putFloat("walltemptrigger", walltemptrigger);
      preferences.putFloat("totalTankTime", totalTankTime);
      preferences.putFloat("rollingAvgGPH", rollingAvgGPH);
      lastSave = millis();
    }
  }
  esp_task_wdt_reset();
  if (currentMillis - lastMemoryCheckTime >= 60000) {  // Check memory every minute
    lastMemoryCheckTime = currentMillis;
    printMemoryStats();
  }

  static unsigned long lastTempUpdate = 0;
  if (millis() - lastTempUpdate >= 300000) {
    lastTempUpdate = millis();
    tempHistory[tempIndex] = currentTemperature;
    tempTimestamps[tempIndex] = millis();
    tempIndex = (tempIndex + 1) % TEMP_HISTORY_SIZE; // Circular buffer index
    // Add voltage history update
    voltageHistory[voltageIndex] = supplyVoltage;
    voltageTimestamps[voltageIndex] = millis();
    voltageIndex = (voltageIndex + 1) % VOLTAGE_HISTORY_SIZE;
  }
 
  // unsigned long rollavgcurrentTime = millis();
  // if (rollavgcurrentTime - lastSampleTime >= 60000) { // Every minute
  //   lastSampleTime = rollavgcurrentTime;
    
  //   // Store the current GPH value
  //   gphSamples[sampleIndex] = currentGPH;
  //   sampleTimestamps[sampleIndex] = rollavgcurrentTime;
    
  //   // Move to the next sample position, wrap around if necessary
  //   sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;
  // }

  // // Calculate rolling average GPH
  // float sum = 0;
  // int rollavgcount = 0;
  // for (int i = 0; i < SAMPLE_SIZE; i++) {
  //   if (rollavgcurrentTime - sampleTimestamps[i] <= 28800000) { // 8 hours in milliseconds
  //     sum += gphSamples[i];
  //     rollavgcount++;
  //   }
  // }

  // if (rollavgcount > 0) {
  //   rollingAvgGPH = sum / rollavgcount;
  // } else {
  //   rollingAvgGPH = 0; // If no samples within the last 8 hours, set to 0 or another default value
  // }
  // static unsigned long lastTempHistorySave = 0;
  // if (millis() - lastTempHistorySave >= INTERVAL_BETWEEN_SAVES) {
  //   saveTempHistory();
  //   lastTempHistorySave = millis();
  // }
  esp_task_wdt_reset();
}

bool validateCRC(uint8_t* frame) {
  uint16_t receivedCRC = (frame[FRAME_SIZE - 2] << 8) | frame[FRAME_SIZE - 1];
  uint16_t calculatedCRC = calculateCRC16(frame, FRAME_SIZE - 2);
  return (receivedCRC == calculatedCRC);
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

void processFrame(uint8_t* frame) {
  if (validateCRC(frame)) {
    if (frame[18] == 0xEB) {  // Check if this frame is a command frame
     // Serial.println("Processing Command Frame");
      memcpy(combinedFrame, frame, FRAME_SIZE);
      commandFrameReceived = true;   // We've received a command frame
    } else if (frame[18] == 0x00) {  // This frame is a status frame
    //  Serial.println("Processing Status Frame");

      if (commandFrameReceived) {
        // If we've received a command frame previously, combine with this status frame
        memcpy(combinedFrame + FRAME_SIZE, frame, FRAME_SIZE);
        commandFrameReceived = false;  // Reset for next cycle
        // Now combinedFrame has both command and status frames
      } else {
        // We've received a status frame without a prior command frame
        // Serial.println("Status frame received without prior command frame, handling independently.");
        // Optionally, handle this status frame alone:
        // handleStatusFrame(frame);
      }
    } else {
      // If we receive a frame with neither 0xEB nor 0x00 at index 18
     // Serial.println("Received Unknown Frame Type");
    }
  } else {
     //Serial.println("CRC Check Failed");
  }
  esp_task_wdt_reset();
}

String frameToHex(uint8_t* frame) {
  String hexString = "";
  for (int i = 0; i < FRAME_SIZE; i++) {
    if (frame[i] < 0x10) hexString += "0";
    hexString += String(frame[i], HEX);
  }
  return hexString;
}

void adjustTemperatureToTarget() {
  int degreesToAdjust = round(targetSetTemperature - setTemperature);

  if (degreesToAdjust == 0) {
    Serial.println("Temperature already at target.");
    return;
  }

  Serial.print("Need to adjust by ");
  Serial.print(abs(degreesToAdjust));
  Serial.println(" degrees.");

  if (degreesToAdjust > 0) {
    simulateButtonPress(increaseTempPin);
    Serial.println("Increased temperature by 1 degree.");
    //  targetSetTemperature -= 1;  // Reduce the target by 1 degree after adjustment
  } else {
    simulateButtonPress(decreaseTempPin);
    Serial.println("Decreased temperature by 1 degree.");
    //  targetSetTemperature += 1;  // Increase the target by 1 degree after adjustment
  }

  // Note: Here we're not waiting for the heater to respond;
  // we're relying on the loop to handle that.
}

void simulateButtonPress(int pin) {
  digitalWrite(pin, LOW);
  delay(buttonPressDuration);
  digitalWrite(pin, HIGH);
}

void printMemoryStats() {
  // Get the total free heap size
  uint32_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  
  // Get the minimum heap size that was ever free
  uint32_t minFreeHeap = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
  
  // Get the largest block of memory that's currently free
  uint32_t largestFreeBlock = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);

  // Print memory stats
  Serial.println("\nMemory Status:");
  Serial.printf("  Free Heap: %d bytes\n", freeHeap);
  Serial.printf("  Minimum Free Heap: %d bytes\n", minFreeHeap);
  Serial.printf("  Largest Free Block: %d bytes\n", largestFreeBlock);
    // Check for low memory conditions
  if (freeHeap < 50000) {  // Example threshold, adjust as needed
    Serial.println("Warning: Low memory detected!");
  }

  // Check for potential fragmentation
  if (largestFreeBlock < (freeHeap / 2)) {  // If less than half of free heap is largest block
    Serial.println("Warning: Possible memory fragmentation detected!");
  }
}

String serializeTempHistory() {
  DynamicJsonDocument tempJsonDoc(8192); // Adjust size as necessary
  JsonArray tempArray = tempJsonDoc.createNestedArray("tempHistory");
  JsonArray timeArray = tempJsonDoc.createNestedArray("timestamps");

  unsigned long currentTime = millis();
  for (int i = 0; i < TEMP_HISTORY_SIZE; i++) {
    int realIndex = (tempIndex + i) % TEMP_HISTORY_SIZE; // Get the correct circular index
    if (tempHistory[realIndex] > -100 && currentTime - tempTimestamps[realIndex] <= 43200000) {  // 12 hours in milliseconds
      tempArray.add(celsiusToFahrenheit(tempHistory[realIndex]));
      timeArray.add((currentTime - tempTimestamps[realIndex]) / 1000);
    }
  }
  String output;
  serializeJson(tempJsonDoc, output);
  return output;
}

String serializeVoltageHistory() {
  DynamicJsonDocument voltageJsonDoc(8192);  // Adjust size as necessary
  JsonArray voltageArray = voltageJsonDoc.createNestedArray("voltageHistory");
  JsonArray timeArray = voltageJsonDoc.createNestedArray("timestamps");

  unsigned long currentTime = millis();
  for (int i = 0; i < VOLTAGE_HISTORY_SIZE; i++) {
    int realIndex = (voltageIndex + i) % VOLTAGE_HISTORY_SIZE;
    if (voltageHistory[realIndex] >= 0 && currentTime - voltageTimestamps[realIndex] <= 43200000) {  // 12 hours
      voltageArray.add(voltageHistory[realIndex]);
      timeArray.add((currentTime - voltageTimestamps[realIndex]) / 1000);
    }
  }
  String output;
  serializeJson(voltageJsonDoc, output);
  return output;
}

void loadTempHistory() {
  // Load temperature data
  for (int i = 0; i < TEMP_HISTORY_SIZE; i++) {
    char keyTemp[20];  // Ensure this is large enough for your key
    char keyTime[20];  // Same here
    snprintf(keyTemp, sizeof(keyTemp), "temp_%d", i);
    snprintf(keyTime, sizeof(keyTime), "time_%d", i);
    
    tempHistory[i] = preferences.getFloat(keyTemp, 0.0);
    tempTimestamps[i] = preferences.getULong(keyTime, 0);
    
    if (tempHistory[i] != 0.0 || tempTimestamps[i] != 0) {
      tempIndex = (i + 1) % TEMP_HISTORY_SIZE;  // Update index to the last written position
    }
  }
}

void saveTempHistory() {
  for (int i = 0; i < TEMP_HISTORY_SIZE; i++) {
    int realIndex = (tempIndex + i) % TEMP_HISTORY_SIZE;
    char keyTemp[20];
    char keyTime[20];
    snprintf(keyTemp, sizeof(keyTemp), "temp_%d", i);
    snprintf(keyTime, sizeof(keyTime), "time_%d", i);
    
    preferences.putFloat(keyTemp, tempHistory[realIndex]);
    preferences.putULong(keyTime, tempTimestamps[realIndex]);
  }
  Serial.println("Temp history saved");
}

void end() {
  preferences.end(); // Close preferences at the end of the program if applicable
}
