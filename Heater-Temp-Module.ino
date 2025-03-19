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
#include <math.h>  // For NAN
#include <map> // Added for deduplication
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <index_html.h>
#include "ESP32SoftwareSerial.h"
#include <OneWire.h>           // Add OneWire library for DS18B20
#include <DallasTemperature.h> // Add DallasTemperature library for DS18B20

#define HEATER_PIN 4 //Need to change to 12 eventually. Possible issues with BLE at 32.
#define LED_BUILTIN 2
// Define the pins for increase and decrease temperature actions
const int increaseTempPin = 25;
const int decreaseTempPin = 26;
const unsigned long buttonPressDuration = 125;  // milliseconds for button press
unsigned long lastAdjustmentTime = 0;
const unsigned long adjustmentInterval = 200;  // interval between adjustments

// New pin definitions for MOSFETS and DY-HFT sensor
#define DUCT_FAN_PWM_PIN 27    // GPIO for duct fan PWM (replacing relay)
#define WALL_FAN_PWM_PIN 33    // GPIO for wall fan PWM (replacing relay)
#define DUCT_FAN_PWM_CHANNEL 0 // PWM channel for duct fan
#define WALL_FAN_PWM_CHANNEL 1 // PWM channel for wall fan
#define PWM_FREQ 1000          // PWM frequency (1kHz)
#define PWM_RESOLUTION 10      // 10-bit resolution (0-1023)
#define DS18B20_SENSOR_PIN 14   // Pin for DY-HFT sensor

// Voltage thresholds and targets
const float NOMINAL_VOLTAGE = 13.5; // Fully charged battery baseline
const float MIN_SUPPLY_VOLTAGE = 10.5; // Minimum supply voltage before fans shut off
const float REC_SUPPLY_VOLTAGE = 11.0; // Recovery supply voltage once the battery charged to 11.5v
const float MAX_SUPPLY_VOLTAGE = 14.3; // Maximum supply voltage while charging
//const float MOSFET_DROP = 0.3;      // IRL540N voltage drop at ~3A load
const float MIN_FAN_VOLTAGE = 10.5;  // Minimum fan voltage for "Low"
const float MED_FAN_VOLTAGE = 11.5;  // Medium fan voltage
const float HIGH_FAN_VOLTAGE = 12.5;// High fan voltage (updated to 12V as requested)
// Fan speed levels (nominal voltages without MOSFET drop)
const int FAN_OFF = 0;           // 0% duty cycle (0V)
const int PWM_MAX = 1023;        // Max PWM value (10-bit resolution)
const float PWM_PER_VOLT = PWM_MAX / NOMINAL_VOLTAGE; // ~75.777 at 13.5V

// Fan states
float manualDuctFanVoltage = 0.0;  // Target voltage for duct fan in manual mode
float manualWallFanVoltage = 0.0;  // Target voltage for wall fan in manual mode
int manualDuctFanSpeed = 0;        // Current PWM for duct fan (updated dynamically)
int manualWallFanSpeed = 0;        // Current PWM for wall fan (updated dynamically)
int ductfan = 0; // 0-Off, 1-Low, 2-Med, 3-High
int wallfan = 0; // 0-Off, 1-Low, 2-Med, 3-High
bool ductFanManualControl = false; // False = automatic, True = manual
bool wallFanManualControl = false; // False = automatic, True = manual
unsigned long ductfandelay = 0;
unsigned long wallfandelay = 0;
float ductFanVoltage = 0.0; // Calculated from PWM
float wallFanVoltage = 0.0; // Calculated from PWM

// Cache for efficiency
int cachedFanLow = 0, cachedFanMed = 0, cachedFanHigh = 0;

// Debug flag (set to 0 in production)
const bool DEBUG = 0;

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
unsigned long lastMillis = 0;
unsigned long overflowCount = 0;
unsigned long long uptime = 0; // Changed to unsigned long long
unsigned long lastMemoryCheckTime = 0;
unsigned int serialinterruptcount = 0;

//default name
String currentBLEName = "HEATER-THERM";

// NTP Client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 10800000);

// Weather API settings
const char* WEATHER_API_KEY = "aeb9ccaba969c927fc2b8ce501da53a8"; // Replace with your API key
String ZIP_CODE = "64856"; // Default value, loaded from Preferences
const char*COUNTRY_CODE = "us"; // Adjust if not in the U.S.
const char* WEATHER_API_HOST = "api.openweathermap.org";
//const String WEATHER_API_ENDPOINT = "/data/2.5/weather?zip=" + ZIP_CODE + "," + String(COUNTRY_CODE) + "&appid=" + String(WEATHER_API_KEY) + "&units=imperial"; // Fahrenheit
float outsideTempF = NAN; // Variable to store outside temperature in Fahrenheit
float outsideHumidity = NAN; // Variable to store outside humidity percentage
unsigned long lastWeatherUpdate = 0;
const unsigned long WEATHER_UPDATE_INTERVAL = 890000;
#define OUTSIDE_TEMP_HISTORY_SIZE 720 // 12 hours * (60 minutes / 5 minutes per update) = 720 entries
float outsideTempHistory[OUTSIDE_TEMP_HISTORY_SIZE];
unsigned long outsideTempTimestamps[OUTSIDE_TEMP_HISTORY_SIZE];
int outsideTempIndex = 0;

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
float avgGallonPerHour = 0.0;    // Average gallons per hour
float currentGPH = 0.0;
float rollingAvgGPH = 0.0; // New variable for rolling average GPH
const float alpha = 0.1;   // Smoothing factor for EMA, you can adjust this
unsigned long totalRuntime = 0;  // Total runtime in seconds to calculate average
int fanSpeed = 0;                // Fan speed in RPM
float supplyVoltage = 0.0;       // Supply voltage in volts
bool frostModeEnabled = false;
float glowPlugHours = 0.0;  // Hours of glow plug operation
int glowPlugCurrent = 0;
float glowPlugCurrent_Amps = 0.0;
bool voltagegood = true;
bool cshut = 0;
String message = "";
String saveError = ""; // Global variable to store errors, add this outside any function
unsigned long lastSerialUpdate = 0;

#define TEMP_HISTORY_SIZE 720 // 12 hours * (60 minutes / 5 minutes per update) = 720 entries
#define INTERVAL_BETWEEN_SAVES 300000
float tempHistory[TEMP_HISTORY_SIZE];
unsigned long tempTimestamps[TEMP_HISTORY_SIZE];
int tempIndex = 0;
#define VOLTAGE_HISTORY_SIZE 720  // Same as temperature: 12 hours of data
float voltageHistory[VOLTAGE_HISTORY_SIZE];
unsigned long voltageTimestamps[VOLTAGE_HISTORY_SIZE];
int voltageIndex = 0;
#define PUMP_HZ_HISTORY_SIZE 720 // 12 hours * (60 minutes / 5 minutes per update) = 720 entries
float pumpHzHistory[PUMP_HZ_HISTORY_SIZE];
unsigned long pumpHzTimestamps[PUMP_HZ_HISTORY_SIZE];
int pumpHzIndex = 0;
float pumpHzAccumulator = 0.0; // Accumulates pump Hz readings
int pumpHzSampleCount = 0;     // Counts number of readings in the interval
// Watt-hour history (24 hours, one entry per hour)
#define WATT_HOUR_HISTORY_SIZE 24
float wattHourHistory[WATT_HOUR_HISTORY_SIZE];
unsigned long wattHourTimestamps[WATT_HOUR_HISTORY_SIZE];
int wattHourIndex = 0;
float wattHourAccumulator = 0.0;
unsigned long wattHourAccumulatorTime = 0;  // Accumulates watt-hours for the current hour
float avgWattHours24h = 0.0; // 24-hour average watt-hours
#define AMPS_HISTORY_SIZE 720 // Same as voltage: 12 hours of data at 5-minute intervals
float ampsHistory[AMPS_HISTORY_SIZE];
unsigned long ampsTimestamps[AMPS_HISTORY_SIZE];
int ampsIndex = 0;

int currentFileIndex = 0;
#define HSAVE_INTERVAL 900000     // 15 minutes in ms
#define HISTORY_FILES 8          // Rotate between 8 files
#define BLOCK_DURATION (6 * 3600)  // 6 hours in seconds
#define HOURLY_FUEL_HISTORY_SIZE 24
float hourlyFuelHistory[HOURLY_FUEL_HISTORY_SIZE];
unsigned long hourlyFuelTimestamps[HOURLY_FUEL_HISTORY_SIZE];
int hourlyFuelIndex = 0;
float hourlyFuelAccumulator = 0.0;
unsigned long hourlyFuelAccumulatorTime = 0; // New global variable
unsigned long lastHourlyUpdate = 0;
unsigned long lastHourlyFuelUpdate = 0;
const unsigned long HOUR_IN_MS = 3600000UL;

unsigned long flasherLastTime;
unsigned long rxLastTime;
unsigned long writerLastTime;
String heaterState[] = { "Off/Stand-by", "Starting", "Pre-Heat", "Retry Start", "Ramping Up", "Heating", "Stop Received", "Shutting Down", "Cooldown"};
int heaterStateNum = -1;
const char* heaterError[] = {
  "No Error",                     // 0 - No Error (0 - 1 = -1, but we treat 0 as no error)
  "Running w/o Error",        // 1 - No Error, but started (1 - 1 = 0)
  "Voltage too high",              // 2 - Voltage too low (2 - 1 = 1)
  "Voltage too low",             // 3 - Voltage too high (3 - 1 = 2)
  "Ignition plug failure",        // 4 - Ignition plug failure (4 - 1 = 3)
  "Pump Failure over current",  // 5 - Pump Failure (5 - 1 = 4)
  "Overheating",                  // 6 - Too hot (6 - 1 = 5)
  "Motor Failure",                // 7 - Motor Failure (7 - 1 = 6)
  "Comms lost",       // 8 - Serial connection lost (8 - 1 = 7)
  "Fire Out",         // 9 - Fire is extinguished (9 - 1 = 8)
  "Temp sensor failed",    // 10 - Temperature sensor failure (10 - 1 = 9)
  "Multiple Starts Failed. Check Fuel." // Fuel failure. Empty or restricted. (11 - 1 = 10)
};
int heaterErrorNum = -1;
int heaterinternalTemp = -200;
unsigned long lastSendTime = 0;
int tempwarn = 0; // 0 = no warning, 1 = warning (>110°F), 2 = critical (>120°F)

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
int controlEnable = 0;
int controlState = -1;

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

// Function to calculate heater fan amps based on RPM
float calculateHeaterFanAmps(int rpm) {
  if (rpm <= 0) return 0.0;
  if (rpm <= 1400) {
    return (rpm / 1400.0) * 3.47;
  } else if (rpm <= 2500) {
    float slope = (6.25 - 3.47) / (2500.0 - 1400.0); // 0.002527 A/RPM
    return 3.47 + slope * (rpm - 1400);
  } else if (rpm <= 4900) {
    float slope = (8.4 - 6.25) / (4900.0 - 2500.0); // 0.0008958 A/RPM
    return 6.25 + slope * (rpm - 2500);
  } else {
    return 8.4; // Cap at 8.4A
  }
}

// Function to calculate duct fan amps based on voltage
float calculateDuctFanAmps(float voltage) {
  if (voltage <= 0) return 0.0;
  if (voltage <= 6.0) {
    return (voltage / 6.0) * 0.08;
  } else if (voltage <= 9.0) {
    float slope = (0.27 - 0.08) / (9.0 - 6.0); // 0.06333 A/V
    return 0.08 + slope * (voltage - 6.0);
  } else {
    float slope = (0.43 - 0.27) / (12.0 - 9.0); // 0.05333 A/V
    return 0.27 + slope * (voltage - 9.0);
  }
}

// Function to calculate wall fan amps based on voltage
// This is for 4" 120mm server fan strongest 
float calculateWallFanAmps(float voltage) {
  if (voltage <= 0) return 0.0; // Invalid input
  if (voltage <= 6.0) {
    return (voltage / 6.0) * 0.4; // 0V to 6V: 0A to 0.4A
  } else if (voltage <= 9.0) {
    float slope = (1.0 - 0.4) / (9.0 - 6.0); // 0.2 A/V
    return 0.4 + slope * (voltage - 6.0); // 6V to 9V
  } else {
    float slope = (1.25 - 1.0) / (12.0 - 9.0); // 0.08333 A/V
    return 1.0 + slope * (voltage - 9.0); // 9V and beyond
  }
}
// this is for white 240CFM inline fan
// Function to calculate wall fan amps based on voltage
// float calculateWallFanAmps(float voltage) {
//     if (voltage <= 0) return 0.0; // Invalid input
    
//     if (voltage <= 6.0) {
//         return (voltage / 6.0) * 0.6; // 0V to 6V: 0A to 0.6A
//     } 
//     else if (voltage <= 9.0) {
//         float slope = (1.2 - 0.6) / (9.0 - 6.0); // 0.2 A/V
//         return 0.6 + slope * (voltage - 6.0); // 6V to 9V
//     } 
//     else {
//         float slope = (3.5 - 1.2) / (12.0 - 9.0); // 0.76667 A/V
//         return 1.2 + slope * (voltage - 9.0); // 9V and beyond
//     }
// }

// Function to calculate fuel pump amps based on Hz
float calculateFuelPumpAmps(float hz) {
  if (hz <= 0) return 0.0;
  float amps = hz * 0.1; // 0.1A per Hz (estimated)
  return min(amps, 1.0f); // Cap at 1A
}

// Function to calculate total power (W)
float calculateTotalPower(float validatedSupply) {
  // Heater fan
  float heaterFanAmps = calculateHeaterFanAmps(fanSpeed);
  float heaterFanPower = heaterFanAmps * validatedSupply;

  // Glow plug
  float glowPlugPower = glowPlugCurrent_Amps * validatedSupply;

  // Duct fan
  float ductFanAmps = calculateDuctFanAmps(ductFanVoltage);
  float ductFanPower = ductFanAmps * ductFanVoltage;

  // Wall fan
  float wallFanAmps = calculateWallFanAmps(wallFanVoltage);
  float wallFanPower = wallFanAmps * wallFanVoltage;

  // Control circuits (.5A when powered on)
  float controlPower = .18 * validatedSupply;

  // Fuel pump
  float fuelPumpAmps = calculateFuelPumpAmps(pumpHz);
  float fuelPumpPower = fuelPumpAmps * validatedSupply;

  // Total power
  return heaterFanPower + glowPlugPower + ductFanPower + wallFanPower + controlPower + fuelPumpPower;
}

// Function to update watt-hour history
void updateWattHourHistory(float wattHours, unsigned long epochTime) {
  static unsigned long lastHourlyUpdate = 0;
  static bool initialized = false;

  // On first call after boot, align lastHourlyUpdate with loaded data
  if (!initialized) {
    if (wattHourAccumulator > 0 && wattHourAccumulatorTime > 0) {
      // Use the timestamp of the loaded accumulator
      lastHourlyUpdate = wattHourAccumulatorTime - (wattHourAccumulatorTime % 3600);
      Serial.println("Initialized lastHourlyUpdate from accumulator to " + String(lastHourlyUpdate));
    } else {
      // Default to current hour start if no accumulator data
      lastHourlyUpdate = epochTime - (epochTime % 3600);
      Serial.println("Initialized lastHourlyUpdate to current hour " + String(lastHourlyUpdate));
    }
    initialized = true;
  }

  wattHourAccumulator += wattHours;

  unsigned long currentHourStart = epochTime - (epochTime % 3600);
  unsigned long lastHourStart = lastHourlyUpdate - (lastHourlyUpdate % 3600);
  if (currentHourStart > lastHourStart && initialized) {
    wattHourHistory[wattHourIndex] = wattHourAccumulator;
    wattHourTimestamps[wattHourIndex] = currentHourStart - 3600;
    wattHourIndex = (wattHourIndex + 1) % WATT_HOUR_HISTORY_SIZE;
    wattHourAccumulator = 0.0;
    lastHourlyUpdate = currentHourStart; // Update to the new hour
    Serial.println("Watt-hour rolled over, reset accumulator at " + String(currentHourStart));
  }
}

// Function to update hourly fuel history
void updateHourlyFuelHistory(float fuelGallons, unsigned long epochTime) {
  static unsigned long lastHourlyFuelUpdate = 0;
  static bool fuelInitialized = false;

  // Initialize lastHourlyFuelUpdate from saved accumulator time or current hour on first run
  if (!fuelInitialized) {
    if (hourlyFuelAccumulator > 0 && hourlyFuelAccumulatorTime > 0) {
      lastHourlyFuelUpdate = hourlyFuelAccumulatorTime - (hourlyFuelAccumulatorTime % 3600);
      if (DEBUG) Serial.println("Initialized lastHourlyFuelUpdate from accumulator to " + String(lastHourlyFuelUpdate));
    } else {
      lastHourlyFuelUpdate = epochTime - (epochTime % 3600);
      if (DEBUG) Serial.println("Initialized lastHourlyFuelUpdate to current hour " + String(lastHourlyFuelUpdate));
    }
    fuelInitialized = true;
  }

  // Add incoming fuel to accumulator
  hourlyFuelAccumulator += fuelGallons;

  // Determine current and last hour start times
  unsigned long currentHourStart = epochTime - (epochTime % 3600);
  unsigned long lastHourStart = lastHourlyFuelUpdate - (lastHourlyFuelUpdate % 3600);

  // Check for hourly rollover or first update after initialization
  if (currentHourStart > lastHourStart || (fuelInitialized && lastHourlyFuelUpdate == 0)) {
    // Store the accumulator value (could be 0 if no fuel used) for the previous hour
    hourlyFuelHistory[hourlyFuelIndex] = hourlyFuelAccumulator;
    hourlyFuelTimestamps[hourlyFuelIndex] = lastHourStart; // Use last hour’s start time
    hourlyFuelIndex = (hourlyFuelIndex + 1) % HOURLY_FUEL_HISTORY_SIZE;

    if (DEBUG) {
      int lastIndex = (hourlyFuelIndex - 1 + HOURLY_FUEL_HISTORY_SIZE) % HOURLY_FUEL_HISTORY_SIZE;
      Serial.println("Hourly fuel updated: " + 
                     String(hourlyFuelHistory[lastIndex], 6) + 
                     " gal at " + String(hourlyFuelTimestamps[lastIndex]));
    }

    // Reset accumulator for the new hour
    hourlyFuelAccumulator = 0.0;
    lastHourlyFuelUpdate = currentHourStart; // Move to current hour
  }
}

// Function to calculate rolling 24-hour average watt-hours per hour
float calculateRolling24HourAverageWattHours() {
  unsigned long currentTime = timeClient.getEpochTime();
  unsigned long oneDayAgo = currentTime - 86400; // 24 hours in seconds
  float totalWattHours = 0.0;
  float hoursCovered = 0.0;

  // Sum completed hourly bins within the last 24 hours
  for (int i = 0; i < WATT_HOUR_HISTORY_SIZE; i++) {
    int realIndex = (wattHourIndex - WATT_HOUR_HISTORY_SIZE + i + WATT_HOUR_HISTORY_SIZE) % WATT_HOUR_HISTORY_SIZE;
    if (wattHourTimestamps[realIndex] > 0 && wattHourTimestamps[realIndex] >= oneDayAgo) {
      totalWattHours += wattHourHistory[realIndex];
      hoursCovered += 1.0; // Each bin represents 1 hour
    }
  }

  // Add the current hour's contribution (wattHourAccumulator), weighted by time elapsed
  unsigned long currentHourStart = currentTime - (currentTime % 3600);
  if (wattHourAccumulator > 0) {
    float elapsedSecondsInHour = currentTime - currentHourStart;
    float fractionOfHour = elapsedSecondsInHour / 3600.0; // 0 to 1
    totalWattHours += wattHourAccumulator;
    hoursCovered += fractionOfHour;
  }

  // Calculate rolling average: total watt-hours divided by 24 hours
  if (hoursCovered > 0) {
    return totalWattHours / 24.0; // Average per hour over 24-hour window
  }
  return 0.0; // Default if no data
}

// Dynamically adjust PWM as voltage sags with battery drain, ensuring minimum 10V output
inline int calculateAdjustedPWM(float targetVoltage, float supplyVoltage) {
  const float REF_VOLTAGE = 13.1;    // Reference supply voltage
  const float MIN_VOLTAGE = 10.0;    // Minimum operational voltage
  const float REF_10V = 10.0;
  const float REF_12V = 12.0;
  const float DEFAULT_SUPPLY = 12.0; // Default if supplyVoltage invalid
  const float REF_PWM_10V = 792.0;   // PWM for 10V at 13.1V (10/13.1 * 1023)
  const float REF_PWM_12V = 952.0;   // PWM for 12V at 13.1V

  // Validate supply voltage
  float validatedSupply = supplyVoltage;
  if (supplyVoltage <= 9.0 || supplyVoltage > 15.0 || supplyVoltage != supplyVoltage) {
    validatedSupply = DEFAULT_SUPPLY;
  }

  // Handle 0V explicitly as off
  if (targetVoltage == 0.0) {
    return 0; // Return 0 PWM for off state
  }

  // Enforce minimum operational voltage of 10V for non-zero targets
  float adjustedTarget = max(targetVoltage, MIN_VOLTAGE);

  float basePWM;
  if (adjustedTarget <= REF_10V) {
    basePWM = REF_PWM_10V;
  } else if (adjustedTarget <= REF_12V) {
    float ratio = (adjustedTarget - REF_10V) / (REF_12V - REF_10V);
    basePWM = REF_PWM_10V + ratio * (REF_PWM_12V - REF_PWM_10V);
  } else {
    basePWM = (adjustedTarget / REF_VOLTAGE) * PWM_MAX;
  }

  float scaleFactor = validatedSupply / REF_VOLTAGE;
  int adjustedPWM = (int)(basePWM / scaleFactor);
  
  return constrain(adjustedPWM, 0, PWM_MAX);
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
  while (WiFi.status() != WL_CONNECTED && (unsigned long)(millis() - wifiConnectStartMillis) < PRIMARY_CONNECT_TIME) {
    delay(1000);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nPrimary WiFi connection failed, trying fallback...");
    WiFi.begin(fallbackSSID, fallbackPassword);
    wifiConnectStartMillis = millis();
    while (WiFi.status() != WL_CONNECTED && (unsigned long)(millis() - wifiConnectStartMillis) < FALLBACK_CONNECT_TIME) {
      delay(1000);
      Serial.print(".");
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    timeClient.begin();
    unsigned long epochTime = timeClient.getEpochTime();
    connectedToAnyNetwork = true;
    if (!MDNS.begin(currentBLEName.c_str())) {
      Serial.println("Error setting up MDNS responder!");
    } else {
      Serial.println("mDNS responder started");
      MDNS.addService("http", "tcp", 80);
      Serial.println("mDNS added HTTP");
    }
  } else {
    Serial.println("Failed to connect to any WiFi network.");
    connectedToAnyNetwork = false;
  }
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting reconnection...");
    server.end();
    MDNS.end();
    connectToWiFi();
    server.begin();
  }
}

String getWeatherApiEndpoint() {
  return "/data/2.5/weather?zip=" + ZIP_CODE + "," + String(COUNTRY_CODE) + "&appid=" + String(WEATHER_API_KEY) + "&units=imperial";
}

void updateWeatherData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping weather update");
    return;
  }

  HTTPClient http;
  String url = "http://" + String(WEATHER_API_HOST) + getWeatherApiEndpoint();
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      outsideTempF = doc["main"]["temp"].as<float>();
      outsideHumidity = doc["main"]["humidity"].as<float>();
      Serial.printf("Weather updated for ZIP %s: Temp=%.1f°F, Humidity=%.1f%%\n", ZIP_CODE.c_str(), outsideTempF, outsideHumidity);
    } else {
      Serial.println("Failed to parse weather JSON: " + String(error.c_str()));
      outsideTempF = NAN;
      outsideHumidity = NAN;
    }
  } else {
    Serial.println("Weather API request failed, HTTP code: " + String(httpCode));
    outsideTempF = NAN;
    outsideHumidity = NAN;
  }

  http.end();
  lastWeatherUpdate = millis();
}

void sendData(const uint8_t* data, size_t length) {
  uint8_t dataToSend[length];
  memcpy(dataToSend, data, length - 2);
  uint16_t crc = calculateCRC16(dataToSend, length - 2);
  dataToSend[length - 2] = (crc >> 8) & 0xFF;
  dataToSend[length - 1] = crc & 0xFF;

  for (int i = 0; i < 2; ++i) {
    for (size_t i = 0; i < length; i++) {
      sOne.write(dataToSend[i]);
    }
   if (DEBUG) Serial.printf("  Sent %d bytes to heater (Attempt %d)\n", length, i + 1);
    delay(20);
  }
}

// File upload handler
void handleUpload(AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
  String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
  Serial.println(logmessage);

  if (!index) {
    logmessage = "Upload Start: " + String(filename);
    Serial.println(logmessage);
    Serial.println("SPIFFS Free: " + String(SPIFFS.totalBytes() - SPIFFS.usedBytes()));
    request->_tempFile = SPIFFS.open("/" + filename, "w");
    if (!request->_tempFile) {
      Serial.println("Failed to open file: " + filename);
      request->send(500, "text/plain", "Failed to open file");
      return;
    }
  }

  if (len) {
    size_t written = request->_tempFile.write(data, len);
    if (written != len) {
      Serial.println("Write failed: wrote " + String(written) + " of " + String(len) + " bytes");
      request->_tempFile.close();
      request->send(500, "text/plain", "Failed to write file");
      return;
    }
    request->_tempFile.flush(); // Ensure data is written
    logmessage = "Writing file: " + String(filename) + " index=" + String(index) + " len=" + String(len) + " written=" + String(written);
    Serial.println(logmessage);
  }

  if (final) {
    logmessage = "Upload Complete: " + String(filename) + ", total size: " + String(index + len);
    request->_tempFile.close();
    File file = SPIFFS.open("/" + filename, "r");
    if (file) {
      Serial.println("Final file size on disk: " + String(file.size()));
      file.close();
    }
    Serial.println(logmessage);
    request->send(200, "text/plain", "Upload successful");
    request->redirect("/");
  }
}

void cleanCorruptedHistoryFiles() {
  Serial.println("Checking for corrupted history files in SPIFFS");
  const unsigned long MIN_VALID_EPOCH = 1710000000UL;
  const size_t MAX_JSON_SIZE = 24576;

  for (int i = 0; i < HISTORY_FILES; i++) {
    String filename = "/history_" + String(i) + ".json";
    if (!SPIFFS.exists(filename)) {
      if (DEBUG) Serial.println("File " + filename + " does not exist, skipping");
      continue;
    }

    File file = SPIFFS.open(filename, FILE_READ);
    if (!file) {
      Serial.println("Failed to open " + filename + ", removing potentially corrupted file");
      SPIFFS.remove(filename);
      continue;
    }

    size_t fileSize = file.size();
    if (fileSize == 0) {
      Serial.println("File " + filename + " is empty, removing");
      file.close();
      SPIFFS.remove(filename);
      continue;
    }
    if (fileSize > MAX_JSON_SIZE) {
      Serial.println("File " + filename + " too large (" + String(fileSize) + " bytes), removing");
      file.close();
      SPIFFS.remove(filename);
      continue;
    }

    DynamicJsonDocument doc(MAX_JSON_SIZE);
    Serial.println("Deserializing " + filename + ", size: " + String(fileSize) + " bytes");
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    yield();

    if (error) {
      Serial.println("Removing corrupted file: " + filename + " - " + error.c_str());
      SPIFFS.remove(filename);
      continue;
    }

    unsigned long startTime = doc["startTime"] | 0;
    if (startTime < MIN_VALID_EPOCH) {
      Serial.println("Removing file with invalid startTime: " + filename + " (startTime: " + String(startTime) + ")");
      SPIFFS.remove(filename);
      continue;
    }

    // Check timestamps for validity
    const char* arraysToCheck[] = {
      "tempTimestamps", "voltageTimestamps", "ampsTimestamps", "pumpHzTimestamps",
      "outsideTempTimestamps", "hourlyFuelTimestamps", "wattHourTimestamps"
    };
    bool hasInvalidTimestamps = false;

    for (const char* key : arraysToCheck) {
      JsonArray timeArray = doc[key];
      if (!timeArray.isNull()) {
        for (size_t j = 0; j < timeArray.size(); j++) {
          unsigned long ts = timeArray[j].as<unsigned long>();
          if (ts < MIN_VALID_EPOCH) {
            hasInvalidTimestamps = true;
            break;
          }
        }
      } else if (DEBUG) {
        Serial.println("Warning: Missing or null array '" + String(key) + "' in " + filename);
      }
      if (hasInvalidTimestamps) break;
    }

    if (hasInvalidTimestamps) {
      Serial.println("Removing file with invalid timestamps: " + filename);
      SPIFFS.remove(filename);
      continue;
    }

    // Check critical arrays for consistency (only watt and fuel matter for removal)
    const char* criticalTimeKeys[] = {"hourlyFuelTimestamps", "wattHourTimestamps"};
    const char* criticalValueKeys[] = {"hourlyFuelHistory", "wattHours"};
    bool hasCriticalMismatch = false;

    for (int k = 0; k < 2; k++) {
      JsonArray timeArray = doc[criticalTimeKeys[k]];
      JsonArray valueArray = doc[criticalValueKeys[k]];
      if (!timeArray.isNull() && !valueArray.isNull() && timeArray.size() != valueArray.size()) {
        hasCriticalMismatch = true;
        Serial.println("Critical array size mismatch in " + filename + ": " + String(criticalTimeKeys[k]) + 
                       " (" + String(timeArray.size()) + ") vs " + String(criticalValueKeys[k]) + 
                       " (" + String(valueArray.size()) + ")");
        break;
      }
    }

    // Log non-critical mismatches but don’t remove
    const char* nonCriticalTimeKeys[] = {
      "tempTimestamps", "voltageTimestamps", "ampsTimestamps", "pumpHzTimestamps", "outsideTempTimestamps"
    };
    const char* nonCriticalValueKeys[] = {
      "tempHistory", "voltageHistory", "ampsHistory", "pumpHzHistory", "outsideTempHistory"
    };
    for (int k = 0; k < 5; k++) {
      JsonArray timeArray = doc[nonCriticalTimeKeys[k]];
      JsonArray valueArray = doc[nonCriticalValueKeys[k]];
      if (!timeArray.isNull() && !valueArray.isNull() && timeArray.size() != valueArray.size()) {
        Serial.println("Non-critical array size mismatch in " + filename + ": " + String(nonCriticalTimeKeys[k]) + 
                       " (" + String(timeArray.size()) + ") vs " + String(nonCriticalValueKeys[k]) + 
                       " (" + String(valueArray.size()) + ")");
      }
    }

    if (hasCriticalMismatch) {
      Serial.println("Removing file with critical data mismatch: " + filename);
      SPIFFS.remove(filename);
    } else {
      if (DEBUG) Serial.println("File " + filename + " is valid for critical data");
    }
    yield();
  }
  Serial.println("Finished checking history files");
}

void setup() {
  esp_task_wdt_deinit();  // wdt is initialized by default. disable and reconfig
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 30000,                             // 30s timeout
    .idle_core_mask = (0),  // All cores
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

  for (int i = 0; i < PUMP_HZ_HISTORY_SIZE; i++) {
    pumpHzHistory[i] = -1.0; // Initialize with an invalid value (e.g., -1 Hz)
    pumpHzTimestamps[i] = 0;
  }
  pumpHzAccumulator = 0.0;
  pumpHzSampleCount = 0;

  for (int i = 0; i < OUTSIDE_TEMP_HISTORY_SIZE; i++) {
    outsideTempHistory[i] = NAN; // Initialize with invalid value
    outsideTempTimestamps[i] = 0;
  }

  // Initialize hourly fuel data
  for (int i = 0; i < HOURLY_FUEL_HISTORY_SIZE; i++) {
    hourlyFuelHistory[i] = 0.0;
    hourlyFuelTimestamps[i] = 0;
  }
  hourlyFuelAccumulator = 0.0;
  lastHourlyUpdate = millis();

  for (int i = 0; i < AMPS_HISTORY_SIZE; i++) {
    ampsHistory[i] = NAN; // Initialize with invalid value
    ampsTimestamps[i] = 0;
  }
  ampsIndex = 0;

  pinMode(increaseTempPin, OUTPUT);
  pinMode(decreaseTempPin, OUTPUT);
  // Ensure pins are not connected to ground at start
  digitalWrite(increaseTempPin, HIGH);
  digitalWrite(decreaseTempPin, HIGH);
  // Send button command to "wake-up" controller
  simulateButtonPress(increaseTempPin);
  simulateButtonPress(decreaseTempPin);
  // Ensure fan pins are off at start. ie MOSFET off
  //digitalWrite(DUCT_FAN_PWM_PIN, LOW);
  digitalWrite(WALL_FAN_PWM_PIN, LOW);

  // Configure PWM for duct fan (Version 3.x API)
  ledcAttachChannel(DUCT_FAN_PWM_PIN, PWM_FREQ, PWM_RESOLUTION, DUCT_FAN_PWM_CHANNEL);
  ledcWrite(DUCT_FAN_PWM_PIN, FAN_OFF); // Start with fan off

  // Configure PWM for wall fan (Version 3.x API)
  ledcAttachChannel(WALL_FAN_PWM_PIN, PWM_FREQ, PWM_RESOLUTION, WALL_FAN_PWM_CHANNEL);
  ledcWrite(WALL_FAN_PWM_PIN, FAN_OFF); // Start with fan off

  sensors.begin();
  pinMode(HEATER_PIN, INPUT);
  sOne.begin(25000);

  Serial.begin(115200);
  Serial.println("Heater Controller Starting...");
  
  bootTime = millis();
  lastMillis = bootTime;
  flasherLastTime = millis(); // No change needed (initialization)
  rxLastTime = millis();     // No change needed (initialization)
  writerLastTime = millis(); // No change needed (initialization)
  pinMode(LED_BUILTIN, OUTPUT);

  preferences.begin("heater", false);
  currentBLEName = preferences.getString("bleName", "HEATER-THERM");  // Load with default
  ZIP_CODE = preferences.getString("zipcode", "64856");
  currentFileIndex = preferences.getInt("currentFileIndex", 0);
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
  ductFanManualControl = preferences.getBool("ductFanManual", false);
  wallFanManualControl = preferences.getBool("wallFanManual", false);
  if (ductFanManualControl) {
    manualDuctFanSpeed = preferences.getInt("manualDuctSpeed", 0);
    manualDuctFanVoltage = preferences.getFloat("manualDuctVoltage", 0.0); // Load target voltage
    if (manualDuctFanVoltage == 0 && manualDuctFanSpeed > 0) {
      manualDuctFanVoltage = (manualDuctFanSpeed * NOMINAL_VOLTAGE) / PWM_MAX; // Backward compatibility
    }
  }
  if (wallFanManualControl) {
    manualWallFanSpeed = preferences.getInt("manualWallSpeed", 0);
    manualWallFanVoltage = preferences.getFloat("manualWallVoltage", 0.0); // Load target voltage
    if (manualWallFanVoltage == 0 && manualWallFanSpeed > 0) {
      manualWallFanVoltage = (manualWallFanSpeed * NOMINAL_VOLTAGE) / PWM_MAX; // Backward compatibility
    }
  }

  connectToWiFi();
  delay(1000); // Delay for wifi stabilization
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  Serial.print("Manage at http://" + currentBLEName + ".local or http://");
  Serial.println(WiFi.localIP());
  delay(1000); // Delay for NTP sync
  updateWeatherData();
  delay(1000); // Delay for weathersync
  esp_task_wdt_reset();

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
    return;
  }
  Serial.println("SPIFFS mounted");

  // Clean corrupted or invalid history files by calling the function
  cleanCorruptedHistoryFiles();

  Serial.println("Loaded currentFileIndex: " + String(currentFileIndex));
  printMemoryStats();
  if (!loadHistoryFromSPIFFS()) {
    Serial.println("No history files or load failed, initializing defaults");
    for (int i = 0; i < TEMP_HISTORY_SIZE; i++) {
      tempHistory[i] = -200.0;
      tempTimestamps[i] = 0;
      voltageHistory[i] = -1.0;
      voltageTimestamps[i] = 0;
    }
  } else {
    Serial.println("History loaded successfully");
  }

  esp_task_wdt_reset();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index_html", "text/html");
  });

  // Handler for JavaScript files
  server.on("^.+\.js$", HTTP_GET, [](AsyncWebServerRequest *request) {
    String path = request->url();
    if (SPIFFS.exists(path)) {
      // Create a response with the file and content type
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, path, "application/javascript");
      
      // Add Cache-Control header for caching
      response->addHeader("Cache-Control", "public, max-age=2592000");

      // Send the response
      request->send(response);
    } else {
      request->send(404, "text/plain", "File not found");
    }
  });

  server.on("/fallback", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/setName", HTTP_POST, [](AsyncWebServerRequest* request) {
      if (request->hasArg("name")) {
          String newName = request->arg("name");
          // Validate the new name
          if (newName.length() > 0 && newName.length() <= 32) {  // Reasonable length limit
              // Save to preferences
              preferences.putString("bleName", newName);
              
              // Update current name
              currentBLEName = newName;
              
              // Restart mDNS with new name
              MDNS.end();
              if (MDNS.begin(currentBLEName.c_str())) {
                  MDNS.addService("http", "tcp", 80);
                  Serial.println("mDNS restarted with new name: " + currentBLEName);
                  request->send(200, "text/plain", "BLE name updated to: " + currentBLEName);
              } else {
                  Serial.println("Failed to restart mDNS with new name: " + currentBLEName);
                  request->send(500, "text/plain", "Failed to restart mDNS");
              }
          } else {
              request->send(400, "text/plain", "Invalid name length (1-32 characters)");
          }
      } else {
          request->send(400, "text/plain", "No name provided");
      }
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
    // controlEnable = 1;
    request->send(200, "text/plain", "New target temperature set.");
  });

  // Updated server-side handler
  server.on("/setFanSpeed", HTTP_POST, [](AsyncWebServerRequest* request) {
    auto setFanSpeed = [&](const char* arg, bool manualControl, float& targetVoltage, int& fanSpeed, int pin) {
      if (request->hasArg(arg) && manualControl) {
        float percent = request->arg(arg).toFloat();
        if (percent < 0 || percent > 100) return;

        float validatedSupply = (supplyVoltage <= 5.0 || supplyVoltage > 15.0 || isnan(supplyVoltage)) ? 12.0 : supplyVoltage;
        if (strcmp(arg, "ductSpeed") == 0) {
          float voltageRange = validatedSupply - 10.0;
          targetVoltage = (percent == 0) ? 0.0 : 10.0 + (percent / 100.0) * voltageRange;
          fanSpeed = (targetVoltage < 10.0) ? 0 : calculateAdjustedPWM(targetVoltage, validatedSupply);
        } else if (strcmp(arg, "wallSpeed") == 0) {
          float voltageRange = validatedSupply - 6.7; // 12.5 - 7 = 5.5V
          targetVoltage = (percent == 0) ? 0.0 : 6.7 + (percent / 100.0) * voltageRange;
          fanSpeed = calculateAdjustedPWM(targetVoltage, validatedSupply);
        }
        ledcWrite(pin, fanSpeed);
        if (strcmp(arg, "ductSpeed") == 0) {
          preferences.putFloat("manualDuctVoltage", targetVoltage);
          preferences.putInt("manualDuctSpeed", fanSpeed);
        } else {
          preferences.putFloat("manualWallVoltage", targetVoltage);
          preferences.putInt("manualWallSpeed", fanSpeed);
        }
      }
    };

    setFanSpeed("ductSpeed", ductFanManualControl, manualDuctFanVoltage, manualDuctFanSpeed, DUCT_FAN_PWM_PIN);
    setFanSpeed("wallSpeed", wallFanManualControl, manualWallFanVoltage, manualWallFanSpeed, WALL_FAN_PWM_PIN);
    request->send(200, "text/plain", "Fan speed updated");
  });

  server.on("/setFanControlMode", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasArg("fan") && request->hasArg("mode")) {
      String fan = request->arg("fan");
      String mode = request->arg("mode");
      if (fan == "duct") {
        ductFanManualControl = (mode == "manual");
        preferences.putBool("ductFanManual", ductFanManualControl);
        if (!ductFanManualControl) {
          manualDuctFanSpeed = 0; // Clear manual speed, let auto mode take over
          manualDuctFanVoltage = 0.0; // Clear target voltage as well
          ductfan = 0; // Reset auto state to Off
          preferences.putInt("manualDuctSpeed", 0);
          preferences.putFloat("manualDuctVoltage", 0.0);
        }
      } else if (fan == "wall") {
        wallFanManualControl = (mode == "manual");
        preferences.putBool("wallFanManual", wallFanManualControl);
        if (!wallFanManualControl) {
          manualWallFanSpeed = 0; // Clear manual speed, let auto mode take over
          manualWallFanVoltage = 0.0; // Clear target voltage as well
          wallfan = 0; // Reset auto state to Off
          preferences.putInt("manualWallSpeed", 0);
          preferences.putFloat("manualWallVoltage", 0.0);
        }
      }
      request->send(200, "text/plain", "Fan control mode set to " + mode + " for " + fan);
    } else {
      request->send(400, "text/plain", "Fan or mode not specified");
    }
  });

  server.on(
    "/upload",
    HTTP_POST,
    [](AsyncWebServerRequest* request) {}, // Empty initial handler
    handleUpload
  );

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
      if (frostModeEnabled) {
          controlEnable = false;  // Only set controlEnable to false when frostModeEnabled is true
      }
      // If frostModeEnabled is false, controlEnable remains unchanged
      preferences.putBool("frostMode", frostModeEnabled);  // Save frostModeEnabled to preferences
      preferences.putBool("controlEnable", controlEnable); // Save controlEnable to preferences (corrected to putBool)
      request->send(200, "text/plain", "Frost Mode updated");
  });

  server.on("/listfiles", HTTP_GET, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(1024);
    JsonArray files = doc.createNestedArray("files");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
      JsonObject fileObj = files.createNestedObject();
      fileObj["name"] = String(file.name());
      fileObj["size"] = file.size();
      file = root.openNextFile();
    }
    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });

  server.on("/deleteFile", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("filename", true)) {
      // Use const pointer since we only read the value
      const AsyncWebParameter* p = request->getParam("filename", true);
      String filename = p->value();
      if (filename.startsWith("/")) filename = filename.substring(1);
      String fullPath = "/" + filename;
      if (SPIFFS.exists(fullPath)) {
        if (SPIFFS.remove(fullPath)) {
          request->send(200, "text/plain", "File deleted: " + filename);
          Serial.println("Deleted file: " + filename);
        } else {
          request->send(500, "text/plain", "Failed to delete file: " + filename);
          Serial.println("Failed to delete: " + filename);
        }
      } else {
        request->send(404, "text/plain", "File not found: " + filename);
        Serial.println("File not found: " + filename);
      }
    } else {
      request->send(400, "text/plain", "No filename provided");
      Serial.println("No filename in delete request");
    }
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
    // controlEnable = 1;
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

  server.on("/turnHeaterOn", HTTP_POST, [](AsyncWebServerRequest* request) {
    // Send heater on command
    uint8_t data1[24] = { 0x76, 0x16, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
    sendData(data1, 24);
    //controlEnable = 1; // Enable thermostat control
    cshut = 0; // Reset shutdown flag
    if (DEBUG) Serial.println("Heater on command sent");
    request->send(200, "text/plain", "Heater on command sent");
  });

  server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest* request) {
  if (request->hasParam("confirm") && request->getParam("confirm")->value() == "DOITS" && eventen) {
    if (DEBUG) Serial.println("Reboot w/ save request received with confirmation");
    request->send(200, "text/plain", "Rebooting w/save ESP32...");
    request->onDisconnect([]() {
      saveHistoryToSPIFFS();
      end();
      ESP.restart();
    });
  } else if (request->hasParam("confirm") && request->getParam("confirm")->value() == "DOIT" && eventen) {
      if (DEBUG) Serial.println("Reboot request received with confirmation");
      request->send(200, "text/plain", "Rebooting ESP32...");
      request->onDisconnect([]() {
        end();
        ESP.restart();
      });
  } else if (request->hasParam("confirm") && request->getParam("confirm")->value() == "DOITANYWAY") {
      if (DEBUG) Serial.println("Reboot request received with confirmation");
      request->send(200, "text/plain", "Forced Rebooting ESP32...");
      request->onDisconnect([]() {
        end();
        ESP.restart();
      });
  } else {
    if (DEBUG) Serial.println("Reboot request denied: missing or incorrect confirmation");
    request->send(400, "text/plain", "Reboot requires confirm=DOIT parameter");
  }
  });

  server.on("/setZipCode", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasArg("zipcode")) {
      String newZip = request->arg("zipcode");
      if (newZip.length() == 5 && newZip.toInt() > 0) { // Basic validation: 5 digits
        if (newZip != ZIP_CODE) { // Only update if changed
          ZIP_CODE = newZip;
          preferences.putString("zipcode", ZIP_CODE);
          Serial.println("Zip code updated to: " + ZIP_CODE);
          updateWeatherData(); // Fetch weather immediately on change
        }
        request->send(200, "text/plain", "Zip code set to " + ZIP_CODE);
      } else {
        request->send(400, "text/plain", "Invalid zip code. Must be a 5-digit number.");
      }
    } else {
      request->send(400, "text/plain", "No zip code provided");
    }
  });

  // Handler for all other files (excluding .js)
  server.onNotFound([](AsyncWebServerRequest *request) {
    if (request->method() == HTTP_GET) {
      String path = request->url();

      if (SPIFFS.exists(path)) {
        // Determine content type based on file extension
        String contentType = "text/plain"; // Default
        if (path.endsWith(".html")) contentType = "text/html";
        else if (path.endsWith(".css")) contentType = "text/css";
        else if (path.endsWith(".png")) contentType = "image/png";
        else if (path.endsWith(".jpg") || path.endsWith(".jpeg")) contentType = "image/jpeg";
        else if (path.endsWith(".gif")) contentType = "image/gif";
        else if (path.endsWith(".json")) contentType = "application/json";

        // Serve the file with caching headers
        AsyncWebServerResponse *response = request->beginResponse(SPIFFS, path, contentType);
        response->addHeader("Cache-Control", "public, max-age=2592000"); // Cache for 1 year

        request->send(response);
      } else {
        request->send(404, "text/plain", "File not found");
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
    saveHistoryToSPIFFS();
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
}

void loop() {
  // Prioritize serial reads
  static byte Data[48];  // For compatibility with existing code
  static int count = 0;  // Tracks combined frame size
  esp_task_wdt_reset();
  if (DEBUG) Serial.println("Sent wdt rest");

  if (sOne.available() && eventen) {
    int inByte = sOne.read();
    if (inByte == 0x76) {
      RxActive = true;
      frameIndex = 0;  // Reset frame index
    }
    if (RxActive) {
      frameBuffer[frameIndex++] = inByte;
      if (frameIndex == FRAME_SIZE) {
        processFrame(frameBuffer);  // Process the frame
        RxActive = false;
        frameIndex = 0;  // Reset for next frame
      }
    }
  }

  // Watchdog check and reset
  if (esp_task_wdt_status(NULL) == ESP_ERR_TIMEOUT) {
    Serial.println("Warning: Task Watchdog Timer timeout detected!");
    esp_task_wdt_reset();
  } else {
    esp_task_wdt_reset();
  }

  // OTA and WiFi maintenance
  ElegantOTA.loop();
  checkWiFiConnection();
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();

  // Static variables and initial setup
  static bool firstRun = true;
  static bool serialEstablished = false;
  unsigned long currentMillis = millis();
  static bool serialActive = false;


    // Detect overflow
  if (currentMillis < lastMillis) {
    overflowCount++; // Increment overflow counter on wraparound
  }
  lastMillis = currentMillis;

  // Calculate uptime with overflow adjustment using unsigned long long
  uptime = (unsigned long long)overflowCount * 4294967295UL + (unsigned long)(currentMillis - bootTime);


  // LED Heartbeat
  static unsigned long flasherLastTime = 0; // Adjusted initialization
  unsigned long flasherTimeNow = millis();
  unsigned long flasherDiff = (unsigned long)(flasherTimeNow - flasherLastTime);
  static long flashLength = 800;
  if (controlEnable == 0) flashLength = 800;
  if (flasherDiff > flashLength) digitalWrite(LED_BUILTIN, HIGH);
  if (flasherDiff > (unsigned long)(flashLength * 2)) { // Overflow-safe
    digitalWrite(LED_BUILTIN, LOW);
    flasherLastTime = flasherTimeNow;
  }

  // Tank runtime calculation
  static unsigned long prevMillis = 0;
  unsigned long elapsedMillis = (unsigned long)(currentMillis - prevMillis);
  float elapsedSeconds = elapsedMillis / 1000.0;
  prevMillis = currentMillis;
  totalTankTime += elapsedSeconds;

  // Process combined frame data (after serial reads)
  if (combinedFrame[0] == 0x76 && combinedFrame[FRAME_SIZE] == 0x76) {
    memcpy(Data, combinedFrame, COMBINED_FRAME_SIZE);
    count = COMBINED_FRAME_SIZE;
    unsigned long writerTimeNow = millis();
    unsigned long writerDiff = (unsigned long)(writerTimeNow - writerLastTime);
    heaterCommand = int(Data[2]);
    currentTemperature = int(Data[3]);
    setTemperature = int(Data[4]);
    heaterStateNum = int(Data[26]);
    fanSpeed = (int(Data[30]) << 8) | int(Data[31]);
    supplyVoltage = ((int(Data[28]) << 8) | int(Data[29])) * 0.1;
    heaterinternalTemp = (int(Data[34]) << 8) | int(Data[35]);
    glowPlugCurrent = (int(Data[38]) << 8) | int(Data[39]);
    glowPlugCurrent_Amps = glowPlugCurrent / 100.0;
    pumpHz = int(Data[40] * 0.1);
    heaterErrorNum = int(Data[27]);
    memset(combinedFrame, 0, COMBINED_FRAME_SIZE);

    lastSerialUpdate = millis();
    serialActive = true;
    if (controlState >= 0) {
      controlEnable = controlState; //restore previous state
    }
    if (!serialEstablished) {
      controlEnable = 1;
      Serial.println("Serial communications established - Control Enabled");
      serialEstablished = true;
    }

    if (heaterErrorNum > 1 && controlEnable) {
      controlEnable = 0;
      Serial.println("Heater control turned off due to error.");
      message = "Heater control turned off due to error.";
      flashLength = 1000000;
      digitalWrite(LED_BUILTIN, HIGH);
    }

    if (heaterStateNum >= 6 && !cshut && controlEnable) {
      controlEnable = 0;
      Serial.println("Heater shutdown unexpectedly. Disabling Thermostat.");
      message = "Heater shutdown unexpectedly. Disabling Thermostat.";
      flashLength = 1000000;
      digitalWrite(LED_BUILTIN, HIGH);
    }

    if (supplyVoltage >= REC_SUPPLY_VOLTAGE) voltagegood = true;
    else if (supplyVoltage < MIN_SUPPLY_VOLTAGE) voltagegood = false;

    if (currentTemperature > 155.0f && currentTemperature <= 255.0f) {
      currentTemperature -= 256.0f;
    }
    if (firstRun) {
      targetSetTemperature = setTemperature;
      firstRun = false;
    }

    static float lastSetTemperature = setTemperature;
    if (setTemperature != lastSetTemperature) {
      lastSetTemperature = setTemperature;
      if (!temperatureChangeByWeb) targetSetTemperature = (int)setTemperature;
    }

    if ((unsigned long)(currentMillis - lastSendTime) >= 30000) { // Overflow-safe
      lastSendTime = currentMillis;
      Serial.println("Heater Status:");
      Serial.printf("  Current Time: %s\n", timeClient.getFormattedTime());
      Serial.printf("  Date: %s\n", getFormattedDate());
      Serial.printf("  Command: %d\n", heaterCommand);
      Serial.printf("  Current Temp: %.2f°C\n", currentTemperature);
      Serial.printf("  Set Temp: %.2f°C\n", setTemperature);
      Serial.printf("  Heater Internal Temp: %d°C\n", heaterinternalTemp);
      Serial.printf("  Glow Plug Current: %.2f A\n", glowPlugCurrent_Amps);
      Serial.printf("  Heater Hour Meter: %.2f Hrs\n", heaterRunTime / 3600.0);
      Serial.printf("  Pump Hz: %.2f Hz\n", pumpHz);
      Serial.printf("  Fan Speed: %d RPM\n", fanSpeed);
      Serial.printf("  Supply Voltage: %.1f V\n", supplyVoltage);
      Serial.printf("  State: %s\n", heaterState[heaterStateNum]);
      Serial.printf("  Error: %s\n", heaterError[heaterErrorNum]);
    }

    if (int(heaterCommand) == 160) {
      controlEnable = 1;
      Serial.println("  Heater ON command detected");
    }
    if (int(heaterCommand) == 5) {
      controlEnable = 0;
      Serial.println("  Heater OFF command detected");
    }

    if (controlEnable == 1 && currentTemperature > -100 && setTemperature > 0 && (unsigned long)(writerDiff) > 30000) { // Overflow-safe
      writerLastTime = writerTimeNow;
      if (setTemperature >= (currentTemperature + 1) && heaterErrorNum <= 1 && heaterStateNum == 0) {
        uint8_t data1[24] = { 0x76, 0x16, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
        sendData(data1, 24);
        flashLength = 100;
        Serial.println("  Starting Heater");
      }
      if (setTemperature <= (currentTemperature - 2) && (heaterStateNum >= 1 && heaterStateNum <= 5)) {
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
      temperatureChangeByWeb = false;
    }
  }

  // Check serial timeout (20s without frames)
  if (serialActive && (millis() - lastSerialUpdate > 20000)) {
    controlState = controlEnable;
    serialActive = false;
    controlEnable = 0;
    heaterCommand = 0;
    currentTemperature = 0;
    setTemperature = 0;
    heaterStateNum = -1;
    fanSpeed = 0;
    supplyVoltage = 0;
    heaterinternalTemp = -200;
    glowPlugCurrent = 0;
    glowPlugCurrent_Amps = 0;
    pumpHz = 0;
    heaterErrorNum = -1;
    
    if (eventen) {
      DynamicJsonDocument jsonDoc(1024);
      jsonDoc["controlEnable"] = controlEnable;    
      jsonDoc["serialActive"] = serialActive;
      jsonDoc["serialinterruptcount"] = serialinterruptcount;
      String jsonString;
      serializeJson(jsonDoc, jsonString);
      String eventString = "event: heater_update\ndata: " + jsonString + "\n\n";
      events.send(eventString.c_str());
    }
    serialinterruptcount++;
    Serial.println("Serial communication stopped " + String(serialinterruptcount) + "times.");
    //jiggle controller
    // Send button command to "wake-up" controller
    //pinMode(HEATER_PIN, INPUT_PULLDOWN);  //Pull comms down
    simulateButtonPress(increaseTempPin);
    simulateButtonPress(decreaseTempPin);
    //pinMode(HEATER_PIN, INPUT);  // Set back to input
  }

  if (temperatureChangeByWeb && !serialActive) {
        temperatureChangeByWeb = false;
        message = "Temp change ignored. Serial Comms down.";
  }

  // Fan control and sensor reading
  static unsigned long lastSensorRead = 0;
  const unsigned long READ_INTERVAL = 2000;
  const float VOLTAGE_CHANGE_THRESHOLD = 0.1;
  const float PWM_VOLTAGE_THRESHOLD = 0.2; // New threshold for PWM update (0.2V)
  if ((unsigned long)(millis() - lastSensorRead) >= READ_INTERVAL) { // Overflow-safe
      sensors.requestTemperatures();
      walltemp = sensors.getTempCByIndex(0);
      if (walltemp == DEVICE_DISCONNECTED_C) {
          if (DEBUG) Serial.println("Error: DS18B20 sensor disconnected");
      } else if (DEBUG) {
          Serial.printf("Wall Temp: %d°F\n", (int)(walltemp * 1.8 + 32));
      }

      float wallTempF = celsiusToFahrenheit(walltemp);
      if (walltemp == DEVICE_DISCONNECTED_C) {
          tempwarn = -1;
      } else {
          if (wallTempF > 120.0) {
              uint8_t data1[24] = { 0x76, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
              sendData(data1, 24);
              cshut = 1;
              controlEnable = 0;
              tempwarn = 2;
              if (DEBUG) Serial.printf("Wall temp %.1f°F > 120°F, shutting down heater\n", wallTempF);
          } else if (wallTempF > 110.0) {
              tempwarn = 1;
              if (DEBUG) Serial.printf("Wall temp %.1f°F > 110°F\n", wallTempF);
          } else {
              tempwarn = 0;
          }
      }

      if (frostModeEnabled) {
        controlEnable = 0;
        if (wallTempF < 40.0 && heaterStateNum == 0) {
          uint8_t data1[24] = { 0x76, 0x16, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
          sendData(data1, 24);
          Serial.println("Frost Mode: Starting Heater");
          message = "Frost Mode Start";
        } else if (wallTempF >= 46.0 && heaterStateNum > 0) {
          uint8_t data1[24] = { 0x76, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
          sendData(data1, 24);
          Serial.println("Frost Mode: Shutting Down Heater");
          message = "Frost Mode Shutdown";
        }
      }

      // Update cached PWM values only if supply voltage changed significantly
      static float lastSupplyVoltage = -1.0; 
      if (abs(supplyVoltage - lastSupplyVoltage) > VOLTAGE_CHANGE_THRESHOLD || lastSupplyVoltage < 0) {
          if (!voltagegood) {
              cachedFanLow = cachedFanMed = cachedFanHigh = 0;
          } else {
              cachedFanLow = calculateAdjustedPWM(MIN_FAN_VOLTAGE, supplyVoltage);  // 10.5V
              cachedFanMed = calculateAdjustedPWM(MED_FAN_VOLTAGE, supplyVoltage);  // 11.5V
              cachedFanHigh = calculateAdjustedPWM(supplyVoltage, supplyVoltage);   // supplyVoltage (e.g., 12V+)
          }
          lastSupplyVoltage = supplyVoltage;
      }

      float validatedSupply = (supplyVoltage <= 9.0 || supplyVoltage > 15.0 || isnan(supplyVoltage)) ? 12.0 : supplyVoltage;
      float gapC = constrain(walltemptrigger, -1, 10);
      static unsigned long speedChangeDelay = 0;
      const unsigned long DEBOUNCE_PERIOD = 30000;

      // Duct fan control
      static int lastDuctFanPWM = -1;
      static float lastDuctVoltage = -1.0; // Track last applied voltage
      if (ductFanManualControl) {
          if (manualDuctFanVoltage > 0) {
              float newVoltage = manualDuctFanVoltage;
              if (abs(newVoltage - lastDuctVoltage) >= PWM_VOLTAGE_THRESHOLD || lastDuctVoltage < 0) {
                  manualDuctFanSpeed = calculateAdjustedPWM(newVoltage, supplyVoltage);
                  ductfan = manualDuctFanSpeed; // Sync ductfan with manual PWM
                  lastDuctVoltage = newVoltage;
              }
          } else {
              manualDuctFanSpeed = 0;
              ductfan = 0;
              lastDuctVoltage = 0.0;
          }
          ledcWrite(DUCT_FAN_PWM_PIN, manualDuctFanSpeed);
          lastDuctFanPWM = manualDuctFanSpeed;
          if (DEBUG) Serial.printf("Duct fan manual: PWM=%d, Voltage=%.1fV\n", manualDuctFanSpeed, manualDuctFanVoltage);
      } else {
          int newDuctFanPWM = 0;
          static unsigned long ductfandelay = 0;
          float newVoltage = 0.0;

          if (walltemp > -55 && walltemp < 125 && currentTemperature != -200.0f) {
              float tempGap = walltemp - currentTemperature;

              if (tempGap < gapC) {
                  if (ductfandelay == 0) {
                      ductfandelay = millis() + 60000;
                      if (DEBUG) Serial.printf("Duct fan preparing to turn off, delay until %lu\n", ductfandelay);
                  }
                  if (millis() >= ductfandelay) {
                      newDuctFanPWM = 0;
                      newVoltage = 0.0;
                  } else {
                      newDuctFanPWM = ductfan;  // Hold current PWM during delay
                      newVoltage = lastDuctVoltage; // Maintain last voltage during delay
                  }
              } else if (voltagegood) {
                  if (tempGap < gapC + 1) {
                      newDuctFanPWM = cachedFanLow;
                      newVoltage = MIN_FAN_VOLTAGE; // 10.5V
                  } else if (tempGap < gapC + 2) {
                      newDuctFanPWM = cachedFanMed;
                      newVoltage = MED_FAN_VOLTAGE; // 11.5V
                  } else {
                      newDuctFanPWM = cachedFanHigh;
                      newVoltage = validatedSupply; // supplyVoltage
                  }
                  ductfandelay = 0;
                  if (DEBUG) Serial.printf("Duct fan auto (tempGap mode): tempGap=%.1f, PWM=%d, Voltage=%.1fV\n",
                      tempGap, newDuctFanPWM, newVoltage);
              } else {
                  newDuctFanPWM = 0;
                  newVoltage = 0.0;
                  if (DEBUG) Serial.printf("Duct fan off (voltage not good)\n");
              }

              // Mode 1: Fan speed-based control (if tempGap isn’t driving)
              if (newDuctFanPWM == 0 && fanSpeed > 2000 && heaterStateNum > 3) {
                  newVoltage = map(fanSpeed, 2000, 4900, 10.0, validatedSupply);
                  newVoltage = constrain(newVoltage, 10.0, validatedSupply);
                  newDuctFanPWM = calculateAdjustedPWM(newVoltage, validatedSupply);
                  ductfandelay = 0;
                  if (DEBUG) Serial.printf("Duct fan auto (fanSpeed mode): fanSpeed=%d, heaterState=%d, targetV=%.1f, PWM=%d\n",
                      fanSpeed, heaterStateNum, newVoltage, newDuctFanPWM);
              }
          } else {
              if (fanSpeed > 2000 && heaterStateNum > 3) {
                  newVoltage = map(fanSpeed, 2000, 4900, 10.0, validatedSupply);
                  newVoltage = constrain(newVoltage, 10.0, validatedSupply);
                  newDuctFanPWM = calculateAdjustedPWM(newVoltage, validatedSupply);
                  ductfandelay = 0;
                  if (DEBUG) Serial.printf("Duct fan auto (fanSpeed mode, temp invalid): fanSpeed=%d, heaterState=%d, targetV=%.1f, PWM=%d\n",
                      fanSpeed, heaterStateNum, newVoltage, newDuctFanPWM);
              } else {
                  newDuctFanPWM = 0;
                  newVoltage = 0.0;
                  ductfandelay = 0;
                  if (DEBUG && lastDuctFanPWM != 0) Serial.printf("Duct fan auto: Neither mode applies, PWM=0\n");
              }
          }

          // Update ductfan only if voltage changes by 0.2V or more
          if (abs(newVoltage - lastDuctVoltage) >= PWM_VOLTAGE_THRESHOLD || lastDuctVoltage < 0) {
              ductfan = newDuctFanPWM;
              lastDuctVoltage = newVoltage;
          } else {
              newDuctFanPWM = ductfan; // Retain last PWM if voltage change is small
          }

          ledcWrite(DUCT_FAN_PWM_PIN, newDuctFanPWM);
          if (newDuctFanPWM != lastDuctFanPWM) {
              if (DEBUG) Serial.printf("Duct fan PWM updated: %d -> %d, Voltage=%.1fV\n", lastDuctFanPWM, newDuctFanPWM, lastDuctVoltage);
              lastDuctFanPWM = newDuctFanPWM;
          }
      }
      ductFanVoltage = lastDuctVoltage; // Update global for power calculation

      // Wall fan control
      static int lastWallFanPWM = -1;
      static float lastWallVoltage = -1.0; // Track last applied voltage
      if (wallFanManualControl) {
          float newVoltage = manualWallFanVoltage;
          if (newVoltage >= 0) {
              float percent = (newVoltage - 6.7) / (HIGH_FAN_VOLTAGE - 6.7) * 100.0;
              if (percent <= 0 || newVoltage == 0) {
                  manualWallFanSpeed = 0;
                  newVoltage = 0.0;
              } else if (percent <= 50.0) {
                  float voltageRange = 9.5 - 6.7;
                  newVoltage = 6.7 + (percent / 50.0) * voltageRange;
                  manualWallFanSpeed = (int)(cachedFanLow * (6.7 / 10.5) + (cachedFanMed * (9.5 / 11.5) - cachedFanLow * (6.7 / 10.5)) * (percent / 50.0));
              } else {
                  float voltageRange = validatedSupply - 9.5;
                  newVoltage = 9.5 + ((percent - 50.0) / 50.0) * voltageRange;
                  manualWallFanSpeed = (int)(cachedFanMed * (9.5 / 11.5) + (cachedFanHigh - cachedFanMed * (9.5 / 11.5)) * ((percent - 50.0) / 50.0));
              }
              manualWallFanSpeed = constrain(manualWallFanSpeed, 0, PWM_MAX);
              if (abs(newVoltage - lastWallVoltage) >= PWM_VOLTAGE_THRESHOLD || lastWallVoltage < 0) {
                  wallfan = manualWallFanSpeed;
                  lastWallVoltage = newVoltage;
              }
              if (DEBUG) Serial.printf("Wall fan manual: Percent=%.1f%%, Voltage=%.1fV, PWM=%d\n", percent, newVoltage, manualWallFanSpeed);
          }
          ledcWrite(WALL_FAN_PWM_PIN, manualWallFanSpeed);
          lastWallFanPWM = manualWallFanSpeed;
      } else {
          int newWallFanPWM = 0;
          float newVoltage = 0.0;

          if (voltagegood && heaterStateNum > 3 && heaterinternalTemp >= 38 && pumpHz != 1) {
              if (fanSpeed <= 2000) {
                  if (wallfandelay == 0) {
                      wallfandelay = millis() + 30000; // 30-second delay to turn off
                      if (DEBUG) Serial.printf("Wall fan preparing to turn off (low speed), delay until %lu\n", wallfandelay);
                  }
                  if (millis() >= wallfandelay) {
                      newWallFanPWM = 0;
                      newVoltage = 0.0;
                  } else {
                      newWallFanPWM = wallfan; // Hold last PWM during delay
                      newVoltage = lastWallVoltage; // Maintain last voltage during delay
                  }
              } else {
                  // Linear interpolation: 2000 RPM (6.7V) to 4900 RPM (10V)
                  // Capping at 10v in auto mode to limit noise
                  float minPWM = cachedFanLow * (6.7 / 10.5); // PWM at 6.7V
                  float maxPWM = cachedFanHigh; // PWM at validatedSupply
                  float rpmRange = 4900.0 - 2000.0; // 2900 RPM span
                  float voltageRange = 10.0 - 6.7; // 3.3V span
                  float pwmRange = maxPWM - minPWM;
                  float rpmFraction = (fanSpeed - 2000.0) / rpmRange; // 0.0 at 2000 RPM, 1.0 at 4900 RPM
                  if (fanSpeed >= 4900) rpmFraction = 1.0; // Cap at 4900 RPM
                  newWallFanPWM = (int)(minPWM + pwmRange * rpmFraction);
                  newVoltage = 6.7 + (rpmFraction * voltageRange); // Voltage scales from 6.7V to 10V
                  newWallFanPWM = constrain(newWallFanPWM, (int)minPWM, (int)maxPWM);
                  wallfandelay = 0; // No delay on speed changes
              }
          } else if (wallfan > 0) { // Condition false but fan was on
              if (wallfandelay == 0) {
                  wallfandelay = millis() + 30000; // 30-second delay to turn off
                  if (DEBUG) Serial.printf("Wall fan preparing to turn off (state/voltage), delay until %lu\n", wallfandelay);
              }
              if (millis() >= wallfandelay) {
                  newWallFanPWM = 0;
                  newVoltage = 0.0;
              } else {
                  newWallFanPWM = wallfan; // Hold last PWM during delay
                  newVoltage = lastWallVoltage; // Maintain last voltage during delay
              }
          } else {
              newWallFanPWM = 0;
              newVoltage = 0.0;
          }

          // Update wallfan only if voltage changes by 0.2V or more
          if (abs(newVoltage - lastWallVoltage) >= PWM_VOLTAGE_THRESHOLD || lastWallVoltage < 0) {
              wallfan = newWallFanPWM;
              lastWallVoltage = newVoltage;
          } else {
              newWallFanPWM = wallfan; // Retain last PWM if voltage change is small
          }

          ledcWrite(WALL_FAN_PWM_PIN, newWallFanPWM);
          if (newWallFanPWM != lastWallFanPWM) {
              if (DEBUG) Serial.printf("Wall fan PWM updated: %d -> %d, Voltage=%.1fV\n", lastWallFanPWM, newWallFanPWM, lastWallVoltage);
              lastWallFanPWM = newWallFanPWM;
          }
      }
      wallFanVoltage = lastWallVoltage; // Update global for power calculation

      lastSensorRead = millis();
  }
  
  float remainingFuelGallons = tankSizeGallons - (tankConsumption * ML_TO_GALLON);
  float rollingRuntimeHours = (remainingFuelGallons / rollingAvgGPH);
  float remainingRuntimeHours = NAN; // Default to NAN
  // updates calcs and accumulators if serial is active every 2s
  static unsigned long lastEvent = 0;
  if ((unsigned long)(millis() - lastEvent) >= 2000 && serialActive) { // Overflow-safe
    lastEvent = millis();
    if (heaterStateNum >= 2 && heaterStateNum <= 5) heaterRunTime += 2;
        float cycleFuelGallons = 0.0; // Default to zero when pump is off
    if (pumpHz > 0) {
      float pumpsPerCycle = pumpHz * 2;
      float cycleFuel = (pumpsPerCycle / 1000.0) * PUMP_FLOW_PER_1000_PUMPS;
      cycleFuelGallons = cycleFuel * ML_TO_GALLON;
      fuelConsumption += cycleFuel;
      tankConsumption += cycleFuel;
      tankRuntime += 2;
      currentGPH = cycleFuelGallons * 3600.0 / 2.0;
      // Accumulate pump Hz for averaging
      pumpHzAccumulator += pumpHz;
      pumpHzSampleCount++;
    } else {
      currentGPH = 0;
    }

    // Update hourly fuel accumulator
    updateHourlyFuelHistory(cycleFuelGallons, timeClient.getEpochTime());

    // Calculate power consumption and watt-hours
    float validatedSupply = (supplyVoltage <= 9.0 || supplyVoltage > 15.0 || isnan(supplyVoltage)) ? 12.0 : supplyVoltage;
    float totalPower = calculateTotalPower(validatedSupply);
    float wattHours = totalPower * (2.0 / 3600.0); // 2-second contribution to watt-hours
    updateWattHourHistory(wattHours, timeClient.getEpochTime());
    avgWattHours24h = calculateRolling24HourAverageWattHours(); // Update rolling average


    if (glowPlugCurrent_Amps > 0.5) glowPlugHours += 2.0 / 3600.0;
    rollingAvgGPH = (alpha * currentGPH) + ((1 - alpha) * rollingAvgGPH);
    if (totalTankTime > 0) {
      float gallonsUsed = tankConsumption * ML_TO_GALLON;
      avgGallonPerHour = (gallonsUsed * 3600) / totalTankTime;
    } else {
      avgGallonPerHour = 0.0;
    }
    // float remainingRuntimeHours = (remainingFuelGallons / avgGallonPerHour);

    // Calculate remainingRuntimeHours using recent hourly fuel usage
    float recentAvgGPH = 0.0;
    int validHours = 0;
    const int HOURS_TO_AVERAGE = 12; // Use last 6 hours (adjustable)
    unsigned long currentTime = timeClient.getEpochTime();

    // Calculate average GPH from the last HOURS_TO_AVERAGE completed hours
    for (int i = 0; i < HOURLY_FUEL_HISTORY_SIZE && validHours < HOURS_TO_AVERAGE; i++) {
      int idx = (hourlyFuelIndex - 1 - i + HOURLY_FUEL_HISTORY_SIZE) % HOURLY_FUEL_HISTORY_SIZE; // Most recent first
      if (hourlyFuelTimestamps[idx] > 0 && (currentTime - hourlyFuelTimestamps[idx]) <= (HOURS_TO_AVERAGE * 3600)) {
        recentAvgGPH += hourlyFuelHistory[idx]; // Each entry is gallons per hour
        validHours++;
      }
    }

    if (validHours > 0) {
      recentAvgGPH /= validHours; // Average GPH over valid hours
      if (tankSizeGallons > 0 && recentAvgGPH > 0) {
        remainingRuntimeHours = remainingFuelGallons / recentAvgGPH;
      }
    } else if (currentGPH > 0 && tankSizeGallons > 0) {
      // Fallback to currentGPH if no hourly data
      remainingRuntimeHours = remainingFuelGallons / currentGPH;
    }
  }
  
  // JSON event updates 2s
  static unsigned long lastJEvent = 0;
  if ((unsigned long)(millis() - lastJEvent) >= 2000) { // Overflow-safe
    lastJEvent = millis();

        // History update every 5min
    static unsigned long lastHistUpdate = 0;
    static bool firstHistUpdate = true;  // Flag for first run
    if ((firstHistUpdate || (unsigned long)(millis() - lastHistUpdate) >= 300000) && eventen && serialActive) { // Overflow-safe
      lastHistUpdate = millis();
      if (epochTime < 1710000000) {
        Serial.println("NTP not synced, epoch: " + String(epochTime));
      }
      tempHistory[tempIndex] = (currentTemperature == -200.0f) ? 0 : currentTemperature;
      tempTimestamps[tempIndex] = epochTime;
      tempIndex = (tempIndex + 1) % TEMP_HISTORY_SIZE;
      outsideTempHistory[outsideTempIndex] = round(outsideTempF); // Store in Fahrenheit
      outsideTempTimestamps[outsideTempIndex] = epochTime;
      outsideTempIndex = (outsideTempIndex + 1) % OUTSIDE_TEMP_HISTORY_SIZE;
      voltageHistory[voltageIndex] = supplyVoltage;
      voltageTimestamps[voltageIndex] = epochTime;
      voltageIndex = (voltageIndex + 1) % VOLTAGE_HISTORY_SIZE;
      // Calculate average pump Hz over the 5-minute period
      pumpHzHistory[pumpHzIndex] = (pumpHzSampleCount > 0) ? (pumpHzAccumulator / pumpHzSampleCount) : pumpHz;
      pumpHzTimestamps[pumpHzIndex] = epochTime;
      pumpHzIndex = (pumpHzIndex + 1) % PUMP_HZ_HISTORY_SIZE;
      // Reset accumulator and count for the next period
      pumpHzAccumulator = 0.0;
      pumpHzSampleCount = 0;
      float validatedSupply = (supplyVoltage <= 9.0 || supplyVoltage > 15.0 || isnan(supplyVoltage)) ? 12.0 : supplyVoltage;
      float totalPower = calculateTotalPower(validatedSupply);
      float totalAmps = (validatedSupply > 0) ? totalPower / validatedSupply : 0.0;
      ampsHistory[ampsIndex] = totalAmps;
      ampsTimestamps[ampsIndex] = epochTime;
      ampsIndex = (ampsIndex + 1) % AMPS_HISTORY_SIZE;
      saveHistoryToSPIFFS();
      firstHistUpdate = false;  // Disable first-run trigger after this
    }

    String voltageWarning = "";
    if ((heaterStateNum > 0 && heaterStateNum <= 4) && supplyVoltage <= 11.0) {
      voltageWarning = "Start volt sag";
    } else if (supplyVoltage <= 10.2 && controlEnable == 1 && heaterStateNum == 5) {
      uint8_t data1[24] = { 0x76, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0x13, 0x88, 0x00, 0x00, 0x32, 0x00, 0x00, 0x05, 0x00, 0xEB, 0x02, 0x00, 0xC8, 0x00, 0x00 };
      sendData(data1, 24);
      cshut = 1;
      controlEnable = 0;
      voltageWarning = "Low voltage!";
      Serial.println("Heater turned off due to low voltage: " + String(supplyVoltage, 1) + "V");
      message = "Heater turned off due to low voltage.";
      flashLength = 1000000;
      digitalWrite(LED_BUILTIN, HIGH);
    }

    static unsigned long lastSave = 0;
    if ((unsigned long)(millis() - lastSave) >= 30000) { // Overflow-safe
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
      if (ductFanManualControl) {
        preferences.putInt("manualDuctSpeed", manualDuctFanSpeed);
        preferences.putFloat("manualDuctVoltage", manualDuctFanVoltage);
      }
      if (wallFanManualControl) {
        preferences.putInt("manualWallSpeed", manualWallFanSpeed);
        preferences.putFloat("manualWallVoltage", manualWallFanVoltage);
      }
      lastSave = millis();
    }
    
    DynamicJsonDocument jsonDoc(24576);
    jsonDoc["bleName"] = currentBLEName;
    jsonDoc["currentTemp"] = (currentTemperature == -200.0f) ? 0 : round(celsiusToFahrenheit(currentTemperature));
    jsonDoc["setTemp"] = round(celsiusToFahrenheit(setTemperature));
    jsonDoc["targettemp"] = round(celsiusToFahrenheit(targetSetTemperature));
    jsonDoc["tempadjusting"] = temperatureChangeByWeb;
    jsonDoc["state"] = (heaterStateNum < 0) ? "Est Coms" : heaterState[heaterStateNum];
    jsonDoc["error"] = (heaterErrorNum < 0) ? "Est Coms" : heaterError[heaterErrorNum];
    jsonDoc["statenum"] = heaterStateNum;
    jsonDoc["errornum"] = heaterErrorNum;
    jsonDoc["heaterHourMeter"] = heaterRunTime / 3600.0;
    jsonDoc["uptime"] = uptime / 1000;
//    jsonDoc["time"] = timeClient.getFormattedTime();
    jsonDoc["epochTime"] = epochTime;
//    jsonDoc["date"] = getFormattedDate();
    jsonDoc["fuelConsumedLifetime"] = fuelConsumption * ML_TO_GALLON;
    jsonDoc["fuelConsumedTank"] = tankConsumption * ML_TO_GALLON;
    jsonDoc["fuelUsedPercentage"] = (tankConsumption * ML_TO_GALLON) / tankSizeGallons;
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
    jsonDoc["rollingRuntimeHours"] = (rollingRuntimeHours > 2190.0 || isnan(rollingRuntimeHours) || abs(rollingRuntimeHours) < 0.0001f) ? JsonVariant() : rollingRuntimeHours;
    jsonDoc["heaterinternalTemp"] = round(celsiusToFahrenheit(heaterinternalTemp));
    jsonDoc["glowPlugCurrent_Amps"] = glowPlugCurrent_Amps;
    jsonDoc["pumpHz"] = pumpHz;
    jsonDoc["controlEnable"] = controlEnable;
    jsonDoc["walltemp"] = (walltemp < -100.0f) ? 0 : round(celsiusToFahrenheit(walltemp));
    jsonDoc["walltemptrigger"] = (walltemptrigger * 9.0 / 5.0);
    jsonDoc["tempwarn"] = tempwarn;
    jsonDoc["ductfan"] = ductfan;
    jsonDoc["wallfan"] = wallfan;
    jsonDoc["ductFanManualControl"] = ductFanManualControl;
    jsonDoc["wallFanManualControl"] = wallFanManualControl;
    jsonDoc["manualDuctFanSpeed"] = manualDuctFanSpeed;
    jsonDoc["manualWallFanSpeed"] = manualWallFanSpeed;
    jsonDoc["manualDuctFanVoltage"] = manualDuctFanVoltage;
    jsonDoc["manualWallFanVoltage"] = manualWallFanVoltage;
    jsonDoc["avgWattHours24h"] = avgWattHours24h;
    jsonDoc["voltagegood"] = voltagegood;
    jsonDoc["ductfandelay"] = max(0UL, (unsigned long)(ductfandelay - millis()) / 1000UL); // Overflow-safe
    jsonDoc["wallfandelay"] = max(0UL, (unsigned long)(wallfandelay - millis()) / 1000UL); // Overflow-safe
    jsonDoc["tempHistory"] = serializeTempHistory();
    jsonDoc["outsideTempHistory"] = serializeOutsideTempHistory(); // Add new history
    jsonDoc["voltageHistory"] = serializeVoltageHistory();
    jsonDoc["pumpHzHistory"] = serializePumpHzHistory();
    jsonDoc["hourlyFuelHistory"] = serializeHourlyFuelHistory(); // Simplified inclusion
    jsonDoc["wattHourHistory"] = serializeWattHourHistory();
    jsonDoc["ampsHistory"] = serializeAmpsHistory(); // Add amps history for plotting
    jsonDoc["message"] = message;
    jsonDoc["serialEstablished"] = serialEstablished;
    jsonDoc["serialActive"] = serialActive;
    jsonDoc["outsideTempF"] = isnan(outsideTempF) ? JsonVariant() : round(outsideTempF); // Add outside temp
    jsonDoc["outsideHumidity"] = isnan(outsideHumidity) ? JsonVariant() : outsideHumidity; // Add humidity
    jsonDoc["zipcode"] = ZIP_CODE;
    jsonDoc["saveError"] = saveError; // Add error message to JSON
    jsonDoc["serialinterruptcount"] = serialinterruptcount;


    String jsonString;
    serializeJson(jsonDoc, jsonString);
    String escapedJsonString = "";
    for (int i = 0; i < jsonString.length(); i++) {
      if (jsonString[i] == '\n') escapedJsonString += "\\n";
      else if (jsonString[i] == '\r') escapedJsonString += "\\r";
      else escapedJsonString += jsonString[i];
    }
    if (eventen) {
      String eventString = "event: heater_update\ndata: " + escapedJsonString + "\n\n";
      events.send(eventString.c_str());
    }
  }

  // Memory stats every 60s
  if ((unsigned long)(currentMillis - lastMemoryCheckTime) >= 60000) { // Overflow-safe
    lastMemoryCheckTime = currentMillis;
    printMemoryStats();
  }

  if ((unsigned long)(millis() - lastWeatherUpdate) >= WEATHER_UPDATE_INTERVAL) {
    updateWeatherData();
  }
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

void adjustTemperatureToTarget() {
  int degreesToAdjust = round(targetSetTemperature - setTemperature);

  if (degreesToAdjust == 0) {
    if (DEBUG) Serial.println("Temperature already at target.");
    return;
  }

  // Only adjust if interval allows
  if ((unsigned long)(millis() - lastAdjustmentTime) >= adjustmentInterval) { // Overflow-safe
    if (DEBUG) {
      Serial.print("Need to adjust by ");
      Serial.print(abs(degreesToAdjust));
      Serial.println(" degrees.");
    }
    if (degreesToAdjust > 0) {
      simulateButtonPress(increaseTempPin);
      if (DEBUG) Serial.println("Increased temperature by 1 degree.");
    } else {
      simulateButtonPress(decreaseTempPin);
      if (DEBUG) Serial.println("Decreased temperature by 1 degree.");
    }
    lastAdjustmentTime = millis();
  }
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

  unsigned long currentTime = timeClient.getEpochTime();
  for (int i = 0; i < TEMP_HISTORY_SIZE; i++) {
    int realIndex = (tempIndex + i) % TEMP_HISTORY_SIZE; // Correct circular index
    if (tempHistory[realIndex] > -100 && (currentTime - tempTimestamps[realIndex]) <= 43200) { // 12 hours in seconds
      tempArray.add(round(celsiusToFahrenheit(tempHistory[realIndex])));
      timeArray.add(tempTimestamps[realIndex]); // Use absolute epoch timestamp
    }
  }
  String output;
  serializeJson(tempJsonDoc, output);
  return output;
}

String serializeVoltageHistory() {
  DynamicJsonDocument voltageJsonDoc(8192); // Adjust size as necessary
  JsonArray voltageArray = voltageJsonDoc.createNestedArray("voltageHistory");
  JsonArray timeArray = voltageJsonDoc.createNestedArray("timestamps");

  unsigned long currentTime = timeClient.getEpochTime();
  for (int i = 0; i < VOLTAGE_HISTORY_SIZE; i++) {
    int realIndex = (voltageIndex + i) % VOLTAGE_HISTORY_SIZE;
    if (voltageHistory[realIndex] >= 0 && (currentTime - voltageTimestamps[realIndex]) <= 43200) { // 12 hours in seconds
      voltageArray.add(voltageHistory[realIndex]);
      timeArray.add(voltageTimestamps[realIndex]); // Use absolute epoch timestamp
    }
  }
  String output;
  serializeJson(voltageJsonDoc, output);
  return output;
}

String serializePumpHzHistory() {
  DynamicJsonDocument pumpHzJsonDoc(8192); // Adjust size as necessary
  JsonArray pumpHzArray = pumpHzJsonDoc.createNestedArray("pumpHzHistory");
  JsonArray timeArray = pumpHzJsonDoc.createNestedArray("timestamps");

  unsigned long currentTime = timeClient.getEpochTime();
  for (int i = 0; i < PUMP_HZ_HISTORY_SIZE; i++) {
    int realIndex = (pumpHzIndex + i) % PUMP_HZ_HISTORY_SIZE;
    if (pumpHzHistory[realIndex] >= 0 && (currentTime - pumpHzTimestamps[realIndex]) <= 43200) { // 12 hours in seconds
      pumpHzArray.add(pumpHzHistory[realIndex]);
      timeArray.add(pumpHzTimestamps[realIndex]); // Use absolute epoch timestamp
    }
  }
  String output;
  serializeJson(pumpHzJsonDoc, output);
  return output;
}

// New serialization function for outdoor temperature
String serializeOutsideTempHistory() {
  DynamicJsonDocument outsideTempJsonDoc(8192); // Adjust size as necessary
  JsonArray outsideTempArray = outsideTempJsonDoc.createNestedArray("outsideTempHistory");
  JsonArray timeArray = outsideTempJsonDoc.createNestedArray("timestamps");

  unsigned long currentTime = timeClient.getEpochTime();
  for (int i = 0; i < OUTSIDE_TEMP_HISTORY_SIZE; i++) {
    int realIndex = (outsideTempIndex + i) % OUTSIDE_TEMP_HISTORY_SIZE;
    if (!isnan(outsideTempHistory[realIndex]) && (currentTime - outsideTempTimestamps[realIndex]) <= 43200) { // 12 hours in seconds
      outsideTempArray.add(outsideTempHistory[realIndex]); // Already in Fahrenheit
      timeArray.add(outsideTempTimestamps[realIndex]); // Use absolute epoch timestamp
    }
  }
  String output;
  serializeJson(outsideTempJsonDoc, output);
  return output;
}

String serializeHourlyFuelHistory() {
  DynamicJsonDocument fuelJsonDoc(8192); // Adjust size as necessary
  JsonArray fuelArray = fuelJsonDoc.createNestedArray("hourlyFuelHistory");
  JsonArray timeArray = fuelJsonDoc.createNestedArray("hourlyFuelTimestamps");

  unsigned long currentTime = timeClient.getEpochTime();
  for (int i = 0; i < HOURLY_FUEL_HISTORY_SIZE; i++) {
    int realIndex = (hourlyFuelIndex - HOURLY_FUEL_HISTORY_SIZE + i + HOURLY_FUEL_HISTORY_SIZE) % HOURLY_FUEL_HISTORY_SIZE;
    if (hourlyFuelTimestamps[realIndex] > 0 && (currentTime - hourlyFuelTimestamps[realIndex]) <= 86400) { // 24 hours
      fuelArray.add(hourlyFuelHistory[realIndex]);
      timeArray.add(hourlyFuelTimestamps[realIndex]); // Use absolute epoch timestamp
    }
  }
  fuelJsonDoc["hourlyFuelAccumulator"] = hourlyFuelAccumulator; // Current hour’s running total

  String output;
  serializeJson(fuelJsonDoc, output);
  return output;
}

String serializeWattHourHistory() {
  DynamicJsonDocument wattHourJsonDoc(8192); // Adjust size as necessary
  JsonArray wattHourArray = wattHourJsonDoc.createNestedArray("wattHours");
  JsonArray timeArray = wattHourJsonDoc.createNestedArray("wattHourTimestamps");

  unsigned long currentTime = timeClient.getEpochTime();
  for (int i = 0; i < WATT_HOUR_HISTORY_SIZE; i++) {
    int realIndex = (wattHourIndex - WATT_HOUR_HISTORY_SIZE + i + WATT_HOUR_HISTORY_SIZE) % WATT_HOUR_HISTORY_SIZE;
    if (wattHourTimestamps[realIndex] > 0 && (currentTime - wattHourTimestamps[realIndex]) <= 86400) { // 24 hours
      wattHourArray.add(wattHourHistory[realIndex]);
      timeArray.add(wattHourTimestamps[realIndex]); // Use absolute epoch timestamp
    }
  }
  wattHourJsonDoc["wattHourAccumulator"] = wattHourAccumulator; // Current hour’s running total

  String output;
  serializeJson(wattHourJsonDoc, output);
  return output;
}

String serializeAmpsHistory() {
  DynamicJsonDocument ampsJsonDoc(8192); // Adjust size as necessary
  JsonArray ampsArray = ampsJsonDoc.createNestedArray("ampsHistory");
  JsonArray timeArray = ampsJsonDoc.createNestedArray("timestamps");

  unsigned long currentTime = timeClient.getEpochTime();
  for (int i = 0; i < AMPS_HISTORY_SIZE; i++) {
    int realIndex = (ampsIndex + i) % AMPS_HISTORY_SIZE;
    if (!isnan(ampsHistory[realIndex]) && (currentTime - ampsTimestamps[realIndex]) <= 43200) { // 12 hours
      ampsArray.add(ampsHistory[realIndex]);
      timeArray.add(ampsTimestamps[realIndex]); // Use absolute epoch timestamp
    }
  }
  String output;
  serializeJson(ampsJsonDoc, output);
  return output;
}

// Update saveHistoryToSPIFFS to include outdoor temperature
void saveHistoryToSPIFFS() {
  saveError = "";
  unsigned long currentTime = timeClient.getEpochTime();
  if (currentTime < 1710000000) {
    saveError = "NTP not synced, skipping save";
    Serial.println(saveError);
    return;
  }

  unsigned long secondsSinceEpochStart = currentTime % (HISTORY_FILES * BLOCK_DURATION);
  int blockIndex = secondsSinceEpochStart / BLOCK_DURATION;
  String filename = "/history_" + String(blockIndex) + ".json";

  unsigned long blockStart = (currentTime / BLOCK_DURATION) * BLOCK_DURATION;
  unsigned long blockEnd = blockStart + BLOCK_DURATION;

  DynamicJsonDocument* doc = new DynamicJsonDocument(24576);
  if (!doc) {
    saveError = "Failed to allocate JSON document";
    Serial.println(saveError);
    return;
  }

  bool appendMode = false;
  if (SPIFFS.exists(filename)) {
    File file = SPIFFS.open(filename, FILE_READ);
    if (file) {
      DeserializationError error = deserializeJson(*doc, file);
      file.close();
      if (!error && (*doc).containsKey("startTime")) {
        unsigned long fileStartTime = (*doc)["startTime"].as<unsigned long>();
        if (fileStartTime == blockStart) {
          appendMode = true;
        }
      }
    }
  }

  if (!appendMode) {
    (*doc)["startTime"] = blockStart;
    (*doc).createNestedArray("tempHistory");
    (*doc).createNestedArray("tempTimestamps");
    (*doc).createNestedArray("voltageHistory");
    (*doc).createNestedArray("voltageTimestamps");
    (*doc).createNestedArray("ampsHistory");
    (*doc).createNestedArray("ampsTimestamps");
    (*doc).createNestedArray("pumpHzHistory");
    (*doc).createNestedArray("pumpHzTimestamps");
    (*doc).createNestedArray("outsideTempHistory");
    (*doc).createNestedArray("outsideTempTimestamps");
    (*doc).createNestedArray("hourlyFuelHistory");
    (*doc).createNestedArray("hourlyFuelTimestamps");
    (*doc).createNestedArray("wattHours");
    (*doc).createNestedArray("wattHourTimestamps");
  }

  JsonArray tempArray = (*doc)["tempHistory"];
  JsonArray tempTimeArray = (*doc)["tempTimestamps"];
  JsonArray voltArray = (*doc)["voltageHistory"];
  JsonArray voltTimeArray = (*doc)["voltageTimestamps"];
  JsonArray ampsArray = (*doc)["ampsHistory"];
  JsonArray ampsTimeArray = (*doc)["ampsTimestamps"];
  JsonArray pumpHzArray = (*doc)["pumpHzHistory"];
  JsonArray pumpHzTimeArray = (*doc)["pumpHzTimestamps"];
  JsonArray outsideTempArray = (*doc)["outsideTempHistory"];
  JsonArray outsideTempTimeArray = (*doc)["outsideTempTimestamps"];
  JsonArray hourlyFuelArray = (*doc)["hourlyFuelHistory"];
  JsonArray hourlyTimeArray = (*doc)["hourlyFuelTimestamps"];
  JsonArray wattHourArray = (*doc)["wattHours"];
  JsonArray wattTimeArray = (*doc)["wattHourTimestamps"];

  // Filter and add temperature data
  int validTempEntries = 0;
  for (int i = 0; i < TEMP_HISTORY_SIZE; i++) {
    int idx = (tempIndex - TEMP_HISTORY_SIZE + i + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
    if (tempHistory[idx] > -100 && tempTimestamps[idx] > 0 &&
        tempTimestamps[idx] >= blockStart && tempTimestamps[idx] < blockEnd) {
      bool exists = false;
      for (size_t j = 0; j < tempTimeArray.size(); j++) {
        if (tempTimeArray[j].as<unsigned long>() == tempTimestamps[idx]) {
          exists = true;
          break;
        }
      }
      if (!exists) {
        tempArray.add(tempHistory[idx]);
        tempTimeArray.add(tempTimestamps[idx]);
        validTempEntries++;
      }
    }
    if (i % 100 == 0) yield();  // Yield every 100 iterations
  }

  // Filter and add voltage data
  int validVoltEntries = 0;
  for (int i = 0; i < VOLTAGE_HISTORY_SIZE; i++) {
    int idx = (voltageIndex - VOLTAGE_HISTORY_SIZE + i + VOLTAGE_HISTORY_SIZE) % VOLTAGE_HISTORY_SIZE;
    if (voltageHistory[idx] >= 0 && voltageTimestamps[idx] > 0 &&
        voltageTimestamps[idx] >= blockStart && voltageTimestamps[idx] < blockEnd) {
      bool exists = false;
      for (size_t j = 0; j < voltTimeArray.size(); j++) {
        if (voltTimeArray[j].as<unsigned long>() == voltageTimestamps[idx]) {
          exists = true;
          break;
        }
      }
      if (!exists) {
        voltArray.add(voltageHistory[idx]);
        voltTimeArray.add(voltageTimestamps[idx]);
        validVoltEntries++;
      }
    }
    if (i % 100 == 0) yield();
  }

  // Filter and add amps data (12-hour window)
  int validAmpsEntries = 0;
  unsigned long twelveHoursAgo = currentTime - 43200;
  for (int i = 0; i < AMPS_HISTORY_SIZE; i++) {
    int idx = (ampsIndex - AMPS_HISTORY_SIZE + i + AMPS_HISTORY_SIZE) % AMPS_HISTORY_SIZE;
    if (!isnan(ampsHistory[idx]) && ampsTimestamps[idx] > 0 &&
        ampsTimestamps[idx] >= twelveHoursAgo) {
      bool exists = false;
      for (size_t j = 0; j < ampsTimeArray.size(); j++) {
        if (ampsTimeArray[j].as<unsigned long>() == ampsTimestamps[idx]) {
          exists = true;
          break;
        }
      }
      if (!exists) {
        ampsArray.add(ampsHistory[idx]);
        ampsTimeArray.add(ampsTimestamps[idx]);
        validAmpsEntries++;
      }
    }
    if (i % 100 == 0) yield();
  }

  // Filter and add pump Hz data
  int validPumpHzEntries = 0;
  for (int i = 0; i < PUMP_HZ_HISTORY_SIZE; i++) {
    int idx = (pumpHzIndex - PUMP_HZ_HISTORY_SIZE + i + PUMP_HZ_HISTORY_SIZE) % PUMP_HZ_HISTORY_SIZE;
    if (pumpHzHistory[idx] >= 0 && pumpHzTimestamps[idx] > 0 &&
        pumpHzTimestamps[idx] >= blockStart && pumpHzTimestamps[idx] < blockEnd) {
      bool exists = false;
      for (size_t j = 0; j < pumpHzTimeArray.size(); j++) {
        if (pumpHzTimeArray[j].as<unsigned long>() == pumpHzTimestamps[idx]) {
          exists = true;
          break;
        }
      }
      if (!exists) {
        pumpHzArray.add(pumpHzHistory[idx]);
        pumpHzTimeArray.add(pumpHzTimestamps[idx]);
        validPumpHzEntries++;
      }
    }
    if (i % 100 == 0) yield();
  }

  // Filter and add outside temperature data
  int validOutsideTempEntries = 0;
  for (int i = 0; i < OUTSIDE_TEMP_HISTORY_SIZE; i++) {
    int idx = (outsideTempIndex - OUTSIDE_TEMP_HISTORY_SIZE + i + OUTSIDE_TEMP_HISTORY_SIZE) % OUTSIDE_TEMP_HISTORY_SIZE;
    if (!isnan(outsideTempHistory[idx]) && outsideTempTimestamps[idx] > 0 &&
        outsideTempTimestamps[idx] >= blockStart && outsideTempTimestamps[idx] < blockEnd) {
      bool exists = false;
      for (size_t j = 0; j < outsideTempTimeArray.size(); j++) {
        if (outsideTempTimeArray[j].as<unsigned long>() == outsideTempTimestamps[idx]) {
          exists = true;
          break;
        }
      }
      if (!exists) {
        outsideTempArray.add(outsideTempHistory[idx]);
        outsideTempTimeArray.add(outsideTempTimestamps[idx]);
        validOutsideTempEntries++;
      }
    }
    if (i % 100 == 0) yield();
  }

  // Filter and add hourly fuel data
  int validHourlyFuelEntries = 0;
  for (int i = 0; i < HOURLY_FUEL_HISTORY_SIZE; i++) {
    int realIndex = (hourlyFuelIndex - HOURLY_FUEL_HISTORY_SIZE + i + HOURLY_FUEL_HISTORY_SIZE) % HOURLY_FUEL_HISTORY_SIZE;
    if (hourlyFuelTimestamps[realIndex] > 0 &&
        hourlyFuelTimestamps[realIndex] >= blockStart && hourlyFuelTimestamps[realIndex] < blockEnd) {
      bool exists = false;
      for (size_t j = 0; j < hourlyTimeArray.size(); j++) {
        if (hourlyTimeArray[j].as<unsigned long>() == hourlyFuelTimestamps[realIndex]) {
          exists = true;
          if (hourlyFuelArray[j].as<float>() != hourlyFuelHistory[realIndex]) {
            hourlyFuelArray[j] = hourlyFuelHistory[realIndex];
          }
          break;
        }
      }
      if (!exists) {
        hourlyFuelArray.add(hourlyFuelHistory[realIndex]);
        hourlyTimeArray.add(hourlyFuelTimestamps[realIndex]);
        validHourlyFuelEntries++;
      }
    }
    if (i % 100 == 0) yield();
  }
  (*doc)["hourlyFuelAccumulator"] = hourlyFuelAccumulator;
  (*doc)["hourlyFuelAccumulatorTime"] = currentTime;
  (*doc)["lastHourlyFuelUpdate"] = lastHourlyFuelUpdate; // Add this line

  // Filter and add watt-hour data (24-hour window)
  int validWattHourEntries = 0;
  unsigned long oneDayAgo = currentTime - 86400;
  for (int i = 0; i < WATT_HOUR_HISTORY_SIZE; i++) {
    int realIndex = (wattHourIndex - WATT_HOUR_HISTORY_SIZE + i + WATT_HOUR_HISTORY_SIZE) % WATT_HOUR_HISTORY_SIZE;
    if (wattHourTimestamps[realIndex] > 0 &&
        wattHourTimestamps[realIndex] >= oneDayAgo &&
        wattHourHistory[realIndex] > 0) {
      bool exists = false;
      for (size_t j = 0; j < wattTimeArray.size(); j++) {
        if (wattTimeArray[j].as<unsigned long>() == wattHourTimestamps[realIndex]) {
          exists = true;
          if (wattHourArray[j].as<float>() != wattHourHistory[realIndex]) {
            wattHourArray[j] = wattHourHistory[realIndex];
          }
          break;
        }
      }
      if (!exists) {
        wattHourArray.add(wattHourHistory[realIndex]);
        wattTimeArray.add(wattHourTimestamps[realIndex]);
        validWattHourEntries++;
      }
    }
    if (i % 100 == 0) yield();
  }
  (*doc)["wattHourAccumulator"] = wattHourAccumulator;
  (*doc)["wattHourAccumulatorTime"] = currentTime;
  (*doc)["lastHourlyUpdate"] = lastHourlyUpdate;

  // Write file if there’s data to save
  if (validTempEntries > 0 || validVoltEntries > 0 || validAmpsEntries > 0 || validPumpHzEntries > 0 ||
      validOutsideTempEntries > 0 || validHourlyFuelEntries > 0 || validWattHourEntries > 0) {
    size_t jsonSize = measureJson(*doc) + 1;
    if (SPIFFS.totalBytes() - SPIFFS.usedBytes() < jsonSize) {
      saveError = "Insufficient SPIFFS space for " + filename + " (" + String(jsonSize) + " bytes needed)";
      Serial.println(saveError);
      delete doc;
      return;
    }
    File file = SPIFFS.open(filename, FILE_WRITE);
    if (!file) {
      saveError = "Failed to open " + filename + " for writing";
      Serial.println(saveError);
      delete doc;
      return;
    }
    size_t bytesWritten = serializeJson(*doc, file);
    file.close();
    if (bytesWritten == 0 || bytesWritten < jsonSize / 2) {
      saveError = "Incomplete write to " + filename + " (" + String(bytesWritten) + " bytes)";
      Serial.println(saveError);
      SPIFFS.remove(filename);
    } else {
      Serial.println("Wrote " + String(bytesWritten) + " bytes to " + filename + " with " +
                     String(tempArray.size()) + " temp entries, " +
                     String(voltArray.size()) + " voltage entries, " +
                     String(ampsArray.size()) + " amps entries, " +
                     String(pumpHzArray.size()) + " pumpHz entries, " +
                     String(outsideTempArray.size()) + " outdoor temp entries, " +
                     String(hourlyFuelArray.size()) + " hourly fuel entries, " +
                     String(wattHourArray.size()) + " watt-hour entries");
      saveError = "Saved " + String(bytesWritten/1024) + "Kb to " + filename + " successfully";
      currentFileIndex = blockIndex;
      preferences.putInt("currentFileIndex", currentFileIndex);
    }
  } else {
    Serial.println("No new data to save within block " + String(blockStart) + " to " + String(blockEnd) + ", skipping write to " + filename);
    saveError = "No changes to save";
  }

  delete doc;  // Free heap memory
}

// Update loadHistoryFromSPIFFS to include outdoor temperature
bool loadHistoryFromSPIFFS() {
  Serial.println("Loading history from SPIFFS");
  unsigned long currentTime = timeClient.getEpochTime();
  Serial.println("Current epoch time: " + String(currentTime));

  // Reset all history arrays
  for (int i = 0; i < TEMP_HISTORY_SIZE; i++) {
    tempHistory[i] = -200.0;
    tempTimestamps[i] = 0;
    voltageHistory[i] = -1.0;
    voltageTimestamps[i] = 0;
    pumpHzHistory[i] = -1.0;
    pumpHzTimestamps[i] = 0;
    outsideTempHistory[i] = NAN;
    outsideTempTimestamps[i] = 0;
  }
  tempIndex = voltageIndex = pumpHzIndex = outsideTempIndex = 0;

  for (int i = 0; i < HOURLY_FUEL_HISTORY_SIZE; i++) {
    hourlyFuelHistory[i] = 0.0;
    hourlyFuelTimestamps[i] = 0;
  }
  hourlyFuelIndex = 0;

  for (int i = 0; i < WATT_HOUR_HISTORY_SIZE; i++) {
    wattHourHistory[i] = 0.0;
    wattHourTimestamps[i] = 0;
  }
  wattHourIndex = 0;

  // Use std::map for deduplication of all datasets
  std::map<unsigned long, float> tempMap;         // 12h
  std::map<unsigned long, float> voltMap;         // 12h
  std::map<unsigned long, float> ampsMap;         // 12h
  std::map<unsigned long, float> pumpHzMap;       // 12h
  std::map<unsigned long, float> outsideTempMap;  // 12h
  std::map<unsigned long, float> hourlyFuelMap;   // 24h
  std::map<unsigned long, float> wattHourMap;     // 24h

  int tempCount = 0, voltCount = 0, ampsCount = 0, pumpHzCount = 0, outsideTempCount = 0;
  int hourlyFuelCount = 0, wattHourCount = 0;

  const unsigned long LAST_12_HOURS = 43200UL;
  const unsigned long LAST_24_HOURS = 86400UL;
  const unsigned long MIN_VALID_EPOCH = 1710000000UL;

  int latestFileIndex = -1;
  unsigned long latestWattTime = 0;
  float loadedWattHourAccumulator = 0.0;
  float loadedFuelAccumulator = 0.0;
  unsigned long loadedWattHourAccumulatorTime = 0;
  unsigned long loadedFuelAccumulatorTime = 0;
  unsigned long loadedLastHourlyUpdate = 0;
  unsigned long loadedLastHourlyFuelUpdate = 0;

  DynamicJsonDocument* doc = new DynamicJsonDocument(24576);
  if (!doc) {
    Serial.println("Failed to allocate JSON document for accumulator load");
    return false;
  }

  // First pass: Find latest accumulator file
  for (int i = 0; i < HISTORY_FILES; i++) {
    String filename = "/history_" + String(i) + ".json";
    if (!SPIFFS.exists(filename)) continue;

    File file = SPIFFS.open(filename, FILE_READ);
    if (!file) continue;

    DeserializationError error = deserializeJson(*doc, file);
    file.close();
    if (error) continue;

    unsigned long wattTime = (*doc)["wattHourAccumulatorTime"].as<unsigned long>();
    if (wattTime > latestWattTime && wattTime >= MIN_VALID_EPOCH && 
        (currentTime - wattTime) <= LAST_24_HOURS) {
      latestWattTime = wattTime;
      latestFileIndex = i;
      loadedWattHourAccumulator = (*doc)["wattHourAccumulator"].as<float>();
      loadedFuelAccumulator = (*doc)["hourlyFuelAccumulator"].as<float>();
      loadedWattHourAccumulatorTime = wattTime;
      loadedFuelAccumulatorTime = (*doc)["hourlyFuelAccumulatorTime"].as<unsigned long>();
      loadedLastHourlyUpdate = (*doc)["lastHourlyUpdate"].as<unsigned long>();
      loadedLastHourlyFuelUpdate = (*doc)["lastHourlyFuelUpdate"].as<unsigned long>();
    }
  }

  if (latestFileIndex >= 0) {
    String filename = "/history_" + String(latestFileIndex) + ".json";
    Serial.println("Loading accumulators from most recent file: " + filename);

    unsigned long currentHourStart = currentTime - (currentTime % 3600);
    unsigned long prevHourStart = currentHourStart - 3600;

    if (loadedWattHourAccumulator > 0 && loadedWattHourAccumulatorTime >= prevHourStart && 
        loadedWattHourAccumulatorTime < currentHourStart + 3600) {
      wattHourAccumulator = loadedWattHourAccumulator;
      wattHourAccumulatorTime = loadedWattHourAccumulatorTime;
      if (DEBUG) Serial.printf("Restored wattHourAccumulator: %.2f Wh from %s (saved at %lu)\n", 
                    wattHourAccumulator, filename.c_str(), wattHourAccumulatorTime);
    } else {
      wattHourAccumulator = 0.0;
      wattHourAccumulatorTime = currentHourStart;
    }

    if (loadedFuelAccumulator > 0 && loadedFuelAccumulatorTime >= prevHourStart && 
        loadedFuelAccumulatorTime < currentHourStart + 3600) {
      hourlyFuelAccumulator = loadedFuelAccumulator;
      hourlyFuelAccumulatorTime = loadedFuelAccumulatorTime;
      if (DEBUG) Serial.printf("Restored hourlyFuelAccumulator: %.6f gal from %s (saved at %lu)\n", 
                    hourlyFuelAccumulator, filename.c_str(), hourlyFuelAccumulatorTime);
    } else {
      hourlyFuelAccumulator = 0.0;
      hourlyFuelAccumulatorTime = currentHourStart;
    }

    lastHourlyUpdate = (loadedLastHourlyUpdate > 0 && loadedLastHourlyUpdate >= MIN_VALID_EPOCH && 
                       (currentTime - loadedLastHourlyUpdate) <= LAST_24_HOURS) 
                       ? loadedLastHourlyUpdate : currentHourStart;
    lastHourlyFuelUpdate = (loadedLastHourlyFuelUpdate > 0 && loadedLastHourlyFuelUpdate >= MIN_VALID_EPOCH && 
                           (currentTime - loadedLastHourlyFuelUpdate) <= LAST_24_HOURS) 
                           ? loadedLastHourlyFuelUpdate : currentHourStart;
  } else {
    Serial.println("No recent history files found for accumulators within 24 hours");
    wattHourAccumulatorTime = currentTime - (currentTime % 3600);
    hourlyFuelAccumulatorTime = currentTime - (currentTime % 3600);
    lastHourlyUpdate = wattHourAccumulatorTime;
    lastHourlyFuelUpdate = hourlyFuelAccumulatorTime;
  }

  // Second pass: Load all history data into maps
  for (int i = 0; i < HISTORY_FILES; i++) {
    String filename = "/history_" + String(i) + ".json";
    if (!SPIFFS.exists(filename)) continue;

    File file = SPIFFS.open(filename, FILE_READ);
    if (!file) continue;

    doc->clear();
    DeserializationError error = deserializeJson(*doc, file);
    file.close();
    if (error) continue;

    unsigned long blockStart = (*doc)["startTime"].as<unsigned long>();
    if (blockStart < MIN_VALID_EPOCH) continue;

    JsonArray tempArray = (*doc)["tempHistory"];
    JsonArray tempTimeArray = (*doc)["tempTimestamps"];
    JsonArray voltArray = (*doc)["voltageHistory"];
    JsonArray voltTimeArray = (*doc)["voltageTimestamps"];
    JsonArray ampsArray = (*doc)["ampsHistory"];
    JsonArray ampsTimeArray = (*doc)["ampsTimestamps"];
    JsonArray pumpHzArray = (*doc)["pumpHzHistory"];
    JsonArray pumpHzTimeArray = (*doc)["pumpHzTimestamps"];
    JsonArray outsideTempArray = (*doc)["outsideTempHistory"];
    JsonArray outsideTempTimeArray = (*doc)["outsideTempTimestamps"];
    JsonArray hourlyFuelArray = (*doc)["hourlyFuelHistory"];
    JsonArray hourlyTimeArray = (*doc)["hourlyFuelTimestamps"];
    JsonArray wattHourArray = (*doc)["wattHours"];
    JsonArray wattTimeArray = (*doc)["wattHourTimestamps"];

    for (size_t j = 0; j < tempArray.size(); j++) {
      if (tempArray[j].is<float>() && tempTimeArray[j].is<unsigned long>()) {
        unsigned long ts = tempTimeArray[j].as<unsigned long>();
        float value = tempArray[j].as<float>();
        if (ts >= MIN_VALID_EPOCH && (currentTime - ts) <= LAST_12_HOURS && value > -100) {
          tempMap[ts] = value;
          tempCount = tempMap.size();
        }
      }
      if (j % 100 == 0) yield();
    }

    for (size_t j = 0; j < voltArray.size(); j++) {
      if (voltArray[j].is<float>() && voltTimeArray[j].is<unsigned long>()) {
        unsigned long ts = voltTimeArray[j].as<unsigned long>();
        float value = voltArray[j].as<float>();
        if (ts >= MIN_VALID_EPOCH && (currentTime - ts) <= LAST_12_HOURS && value >= 0) {
          voltMap[ts] = value;
          voltCount = voltMap.size();
        }
      }
      if (j % 100 == 0) yield();
    }

    for (size_t j = 0; j < ampsArray.size(); j++) {
      if (ampsArray[j].is<float>() && ampsTimeArray[j].is<unsigned long>()) {
        unsigned long ts = ampsTimeArray[j].as<unsigned long>();
        float value = ampsArray[j].as<float>();
        if (ts >= MIN_VALID_EPOCH && (currentTime - ts) <= LAST_12_HOURS && value >= 0) {
          ampsMap[ts] = value;
          ampsCount = ampsMap.size();
        }
      }
      if (j % 100 == 0) yield();
    }

    for (size_t j = 0; j < pumpHzArray.size(); j++) {
      if (pumpHzArray[j].is<float>() && pumpHzTimeArray[j].is<unsigned long>()) {
        unsigned long ts = pumpHzTimeArray[j].as<unsigned long>();
        float value = pumpHzArray[j].as<float>();
        if (ts >= MIN_VALID_EPOCH && (currentTime - ts) <= LAST_12_HOURS && value >= 0) {
          pumpHzMap[ts] = value;
          pumpHzCount = pumpHzMap.size();
        }
      }
      if (j % 100 == 0) yield();
    }

    for (size_t j = 0; j < outsideTempArray.size(); j++) {
      if (outsideTempArray[j].is<float>() && outsideTempTimeArray[j].is<unsigned long>()) {
        unsigned long ts = outsideTempTimeArray[j].as<unsigned long>();
        float value = outsideTempArray[j].as<float>();
        if (ts >= MIN_VALID_EPOCH && (currentTime - ts) <= LAST_12_HOURS && value > -100) {
          outsideTempMap[ts] = value;
          outsideTempCount = outsideTempMap.size();
        }
      }
      if (j % 100 == 0) yield();
    }

    for (size_t j = 0; j < hourlyFuelArray.size(); j++) {
      if (hourlyFuelArray[j].is<float>() && hourlyTimeArray[j].is<unsigned long>()) {
        unsigned long ts = hourlyTimeArray[j].as<unsigned long>();
        float value = hourlyFuelArray[j].as<float>();
        if (ts >= MIN_VALID_EPOCH && (currentTime - ts) <= LAST_24_HOURS && value >= 0) {
          hourlyFuelMap[ts] = value;
          hourlyFuelCount = hourlyFuelMap.size();
        }
      }
      if (j % 100 == 0) yield();
    }

    for (size_t j = 0; j < wattHourArray.size(); j++) {
      if (wattHourArray[j].is<float>() && wattTimeArray[j].is<unsigned long>()) {
        unsigned long ts = wattTimeArray[j].as<unsigned long>();
        float value = wattHourArray[j].as<float>();
        if (ts >= MIN_VALID_EPOCH && (currentTime - ts) <= LAST_24_HOURS && value >= 0) {
          wattHourMap[ts] = value;
          wattHourCount = wattHourMap.size();
        }
      }
      if (j % 100 == 0) yield();
    }
  }

  struct Entry {
    float value;
    unsigned long timestamp;
  };

  // Helper function to load map into array
  auto loadMapToArray = [](std::map<unsigned long, float>& map, float* values, unsigned long* timestamps, 
                           int& index, int maxSize, bool newestFirst) {
    int count = map.size();
    Entry* entries = (Entry*)malloc(count * sizeof(Entry));
    if (!entries) {
      Serial.println("Failed to allocate entries for loading");
      return;
    }
    int idx = 0;
    for (const auto& entry : map) {
      entries[idx++] = {entry.second, entry.first};
    }
    qsort(entries, count, sizeof(Entry),
          [](const void* a, const void* b) -> int {
            const Entry* ea = (const Entry*)a;
            const Entry* eb = (const Entry*)b;
            return (ea->timestamp > eb->timestamp) - (ea->timestamp < eb->timestamp);
          });
    int loadCount = count < maxSize ? count : maxSize;
    for (int i = 0; i < loadCount; i++) {
      int srcIdx = newestFirst ? (count - loadCount + i) : i;
      values[i] = entries[srcIdx].value;
      timestamps[i] = entries[srcIdx].timestamp;
    }
    index = loadCount % maxSize;
    if (DEBUG && loadCount > 0) {
      Serial.printf("Loaded %d entries, last: %.2f at %lu\n", loadCount, values[loadCount-1], timestamps[loadCount-1]);
    }
    free(entries);
  };

  // Load 12-hour datasets (newest first)
  if (tempCount > 0) {
    loadMapToArray(tempMap, tempHistory, tempTimestamps, tempIndex, TEMP_HISTORY_SIZE, true);
  }
  if (voltCount > 0) {
    loadMapToArray(voltMap, voltageHistory, voltageTimestamps, voltageIndex, VOLTAGE_HISTORY_SIZE, true);
  }
  if (ampsCount > 0) {
    loadMapToArray(ampsMap, ampsHistory, ampsTimestamps, ampsIndex, AMPS_HISTORY_SIZE, true);
  }
  if (pumpHzCount > 0) {
    loadMapToArray(pumpHzMap, pumpHzHistory, pumpHzTimestamps, pumpHzIndex, PUMP_HZ_HISTORY_SIZE, true);
  }
  if (outsideTempCount > 0) {
    loadMapToArray(outsideTempMap, outsideTempHistory, outsideTempTimestamps, outsideTempIndex, OUTSIDE_TEMP_HISTORY_SIZE, true);
  }

  // Load 24-hour datasets (newest first, consistent with current behavior)
  if (hourlyFuelCount > 0) {
    loadMapToArray(hourlyFuelMap, hourlyFuelHistory, hourlyFuelTimestamps, hourlyFuelIndex, HOURLY_FUEL_HISTORY_SIZE, true);
  }
  if (wattHourCount > 0) {
    loadMapToArray(wattHourMap, wattHourHistory, wattHourTimestamps, wattHourIndex, WATT_HOUR_HISTORY_SIZE, true);
  }

  delete doc;

  bool success = tempCount > 0 || voltCount > 0 || ampsCount > 0 || pumpHzCount > 0 || 
                 outsideTempCount > 0 || hourlyFuelCount > 0 || wattHourCount > 0 ||
                 wattHourAccumulator > 0 || hourlyFuelAccumulator > 0;

  if (success) {
    Serial.println("Loaded history: " + String(tempCount) + " temp, " + String(voltCount) + " volt, " +
                   String(ampsCount) + " amps, " + String(pumpHzCount) + " pumpHz, " +
                   String(outsideTempCount) + " outdoor, " + String(hourlyFuelCount) + " fuel, " +
                   String(wattHourCount) + " Watt-hour, Fuel Acc: " + String(hourlyFuelAccumulator, 6) + 
                   ", Watt Acc: " + String(wattHourAccumulator, 2));
  } else {
    Serial.println("No valid history data loaded");
  }

  avgWattHours24h = calculateRolling24HourAverageWattHours();
  return success;
}

void end() {
  preferences.end(); // Close preferences at the end of the program if applicable
}