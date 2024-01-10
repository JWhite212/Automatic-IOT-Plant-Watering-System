// ==== DEFINES ===================================================================================

// ==== Debug and Test options ==================
#define _DEBUG_
//#define _MAIN_

//===== Debugging macros ========================
#ifdef _DEBUG_
#define SerialD Serial
#define _PM(a)                                                                                                                                                 \
  SerialD.print(millis());                                                                                                                                     \
  SerialD.print(": ");                                                                                                                                         \
  SerialD.println(a)
#define _PP(a) SerialD.print(a)
#define _PL(a) SerialD.println(a)
#define _PX(a) SerialD.println(a, HEX)
#else
#define _PM(a)
#define _PP(a)
#define _PL(a)
#define _PX(a)
#endif

// ==== INCLUDES ==================================================================================
#include <BH1750.h>

#include <MovingAveragePlus.h>

#include <RTClib.h>

#include <Wire.h>

#include <WiFi.h>

#include <LiquidCrystal_I2C.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <SPI.h>

#include <Preferences.h>

#include <ThingSpeak.h>
#include <Arduino.h>
// ==== Uncomment desired compile options =================================
// ----------------------------------------
// The following "defines" control library functionality at compile time,
// and should be used in the main sketch depending on the functionality required
// Should be defined BEFORE #include <TaskScheduler.h>  !!!
//
// #define _TASK_TIMECRITICAL       // Enable monitoring scheduling overruns
// #define _TASK_SLEEP_ON_IDLE_RUN  // Enable 1 ms SLEEP_IDLE powerdowns between runs if no callback methods were invoked during the pass
#define _TASK_STATUS_REQUEST // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS            // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER        // Compile with support for local task storage pointer
// #define _TASK_PRIORITY           // Support for layered scheduling priority
// #define _TASK_MICRO_RES          // Support for microsecond resolution
// #define _TASK_STD_FUNCTION       // Support for std::function (ESP8266 ONLY)
// #define _TASK_DEBUG              // Make all methods and variables public for debug purposes
// #define _TASK_INLINE             // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
// #define _TASK_TIMEOUT            // Support for overall task timeout
// #define _TASK_OO_CALLBACKS       // Support for callbacks via inheritance
// #define _TASK_EXPOSE_CHAIN       // Methods to access tasks in the task chain
// #define _TASK_SCHEDULING_OPTIONS // Support for multiple scheduling options
// #define _TASK_DEFINE_MILLIS      // Force forward declaration of millis() and micros() "C" style
// #define _TASK_EXTERNAL_TIME      // Custom millis() and micros() methods
// #define _TASK_THREAD_SAFE        // Enable additional checking for thread safety
// #define _TASK_SELF_DESTRUCT      // Enable tasks to "self-destruct" after disable

#include <TaskScheduler.h>

// ==== GLOBALS ===================================================================================
String ssid;     // Set your own SSID
String password; // Set your own WiFi password
// ThingSpeak information
char thingSpeakAddress[] = "api.thingspeak.com";
unsigned long channelID = 2073592;
const char *readAPIKey = "MM2WIMUEIAVZ3E0U";
const char *writeAPIKey = "TJ1SO2U076DFZUXS";
const unsigned long postingInterval = TASK_SECOND * 30;
unsigned int tempFieldNo = 1;                // Field to write temperature (*c) data
unsigned int humidityFieldNo = 2;            // Field to write humidity (%) data
unsigned int soilMoistureFieldNo = 3;        // Field to write Soil Moisture (%) data
unsigned int pumpActivationTimeFieldNo = 4;  // Field to write pump activation time data
unsigned int LightActivationTimeFieldNo = 5; // Field to write Light Activation time data
unsigned int luxLevelsFieldNo = 6;           // Field to write Light levels (Lux) data
unsigned int waterTankLevelsFieldNo = 7;     // Field to write Water tank levels (%) data

unsigned long lastConnectionTime = 0;
long lastUpdateTime = 0;

WiFiClient client;

Preferences preferences; // Object class for storing on Flash for the ESP

#define TRIG_PIN 17 // ESP32 pin GIOP17 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 16 // ESP32 pin GIOP16 connected to Ultrasonic Sensor's ECHO pin

float duration_us, distance_cm; // Variable stores for Ultrasonic sensor

int waterTankLevel;

#define DHT_SENSOR1_PIN 25 // Digital pin connected to the DHT sensor1
#define DHT_SENSOR2_PIN 26 // Digital pin connected to the DHT sensor2
#define DHTTYPE DHT11      // DHT 11

DHT dhtSensor1(DHT_SENSOR1_PIN, DHTTYPE);
DHT dhtSensor2(DHT_SENSOR2_PIN, DHTTYPE);

RTC_DS3231 rtc;

BH1750 lightMeter;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);

int moisture_sensor_pin1 = 36; // signal from the capacitive soil moisture
int moisture_sensor_pin2 = 39; // signal from the capacitive soil moisture
int moisture_sensor_pin3 = 34; // signal from the capacitive soil moisture

int moisture_value1; // value of soil moisture1
int moisture_value2; // value of soil moisture2
int moisture_value3; // value of soil moisture3

int moistureCombinedAvg; // Combined moving average of soil moisture sensor

int moistureCombinedAvgAtPump; // Combined moving average of soil moisture sensor when pump is activated

int pumpPin = 14;           // digital pin where the pump's relay is plugged in
int lightPin = 13;          // digital pin where the light's relay is plugged in
int moistureThreshold = 35; // threshold value to trigger pump %

int LastPumpMonth;
int LastPumpDay;
int LastPumpHours;
int LastPumpMinutes;
int LastPumpUnixSeconds;
int currentUnixSeconds;
// Variables for the timing of the wifi connection, to break out of the loop
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long timeoutPeriod = 100000; // the value is a number of milliseconds

int MTregVal = 0;
bool connected = false;
bool rtcCheck = false;

// Read sensor data from both sensors
float humidity1;
float temperature1;
float humidity2;
float temperature2;

float avg_humidity;
float avg_temperature;

float TEMP_UPPER_THRESHOLD = 25;
float HUM_UPPER_THRESHOLD = 25;
float TEMP_LOWER_THRESHOLD = 12;
float HUM_LOWER_THRESHOLD = 12;

// ==== Status Requests ========================
StatusRequest rtcEnabled;
StatusRequest wifiEnabled;
StatusRequest measure;
StatusRequest screenValues;

// ==== Scheduler ==============================
Scheduler ts;

void tickerCallback();
void tickerCallback();

bool tickerEnable();
bool tickerDebugEnable();

void setClock();
void readDHTCallback();
bool setupDHTReadings();
void checkBH1750Callback();
bool testWiFiConnection();
void setupDeviceForWiFi();
void setRTCWithNTC();
void printTime();
void printRTCTime();
void cleanDHTReadings();
bool S1Enable();
bool S2Enable();
void S1Callback();
void S2Callback();
void CalcCallback();
void ScreenUpdateCallback();
void checkSoilMoistThreshold();
bool setupMoistureReadings();
bool checkTimeThreshold();
void checkUltrasonic();
void ultrasonicOnDisable();
void WriteDataToThingSpeak();
bool pumpCallbackEnableOn();
void pumpCallback();
void pumpCallbackEnableOff();

// ==== Scheduling defines (cheat sheet) =====================
/*
  TASK_MILLISECOND  - one millisecond in millisecond/microseconds
  TASK_SECOND       - one second in millisecond/microseconds
  TASK_MINUTE       - one minute in millisecond/microseconds
  TASK_HOUR         - one hour in millisecond/microseconds
  TASK_IMMEDIATE    - schedule task to run as soon as possible
  TASK_FOREVER      - run task indefinitely
  TASK_ONCE         - run task once
  TASK_NOTIMEOUT    - set timeout interval to No Timeout

  TASK_SCHEDULE     - schedule is a priority, with "catch up" (default)
  TASK_SCHEDULE_NC  - schedule is a priority, without "catch up"
  TASK_INTERVAL     - interval is a priority, without "catch up"

  TASK_SR_OK        - status request triggered with an OK code (all good)
  TASK_SR_ERROR     - status request triggered with an ERROR code
  TASK_SR_CANCEL    - status request was cancelled
  TASK_SR_ABORT     - status request was aborted
  TASK_SR_TIMEOUT   - status request timed out
*/

/* ==== Task definitions ========================
Task tTask(TIME, AMOUNT, &CallBackMethod, &ts, ENABLED, &onEnable, &OnDisable);
*/
#ifdef _DEBUG_
Task tTicker(TASK_SECOND * 10, TASK_FOREVER, &tickerCallback, &ts, false, &tickerDebugEnable);
#else
Task tTicker(TASK_SECOND * 10, TASK_FOREVER, &tickerCallback, &ts, false, &tickerEnable);
#endif

Task tWifiCheck(TASK_SECOND, TASK_ONCE, &WriteDataToThingSpeak, &ts, false, &testWiFiConnection);

Task tPrintTimes(TASK_SECOND * 10, TASK_ONCE, NULL, &ts, false, &testWiFiConnection);

Task tReadDHT(TASK_SECOND * 2, TASK_ONCE, &readDHTCallback, &ts, false, &setupDHTReadings, &cleanDHTReadings);
Task tCalculate(&CalcCallback, &ts);
Task tSensor1(TASK_IMMEDIATE, TASK_ONCE, &S1Callback, &ts, false, &S1Enable);
Task tSensor2(TASK_IMMEDIATE, TASK_ONCE, &S2Callback, &ts, false, &S2Enable);

Task tScreen(&ScreenUpdateCallback, &ts);

Task tReadBH1750(TASK_SECOND / 2, TASK_FOREVER, &checkBH1750Callback, &ts, false);

Task tReadMoisture(TASK_SECOND * 3, TASK_FOREVER, &checkSoilMoistThreshold, &ts, false, &setupMoistureReadings);

Task tReadUltrasonic(TASK_MILLISECOND * 200, TASK_ONCE * 5, &checkUltrasonic, &ts, false, NULL, &ultrasonicOnDisable);

Task tPump(TASK_SECOND, TASK_ONCE, &pumpCallback, &ts, false, &pumpCallbackEnableOn, &pumpCallbackEnableOff);

// ==== CODE ======================================================================================

/**************************************************************************/
/*!
    @brief    Standard Arduino SETUP method - initialize sketch
    @param    none
    @returns  none
*/
/**************************************************************************/
void setup() {
// put your setup code here, to run once:
#if defined(_DEBUG_)
  Serial.begin(115200);
  delay(2000);
  _PL("DEBUG Mode: setup()");
#endif
  Wire.begin(21, 22); // initialize the I2C bus on GPIO 21 (SDA) and GPIO 22 (SCL)
  rtc.begin();        // DS3231 RTC clock activate

  delay(2000);

  preferences.begin("credentials", false);

  LastPumpUnixSeconds = preferences.getUInt("pumpTime", 0);

  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");

  if (ssid == "" || password == "") {
    Serial.println("No values saved for ssid or password");
  } else {
    // Length (with one extra character for the null terminator)
    int str_len = ssid.length() + 1;
    int pass_len = password.length() + 1;

    // Prepare the character array (the buffer)
    char char_array[str_len];
    char char_array_pass[pass_len];

    // Copy it over
    ssid.toCharArray(char_array, str_len);
    password.toCharArray(char_array_pass, pass_len);

    // Connect to Wi-Fi
    WiFi.begin(char_array, char_array_pass);
  }

  delay(3000);

  startMillis = millis(); // initial start time

  pinMode(pumpPin, OUTPUT);  // setup for the Pump OUTPUT
  pinMode(lightPin, OUTPUT); // setup for the Light OUTPUT

  digitalWrite(pumpPin, HIGH);  // Start Pump off
  digitalWrite(lightPin, HIGH); // Start Light off

  pinMode(TRIG_PIN, OUTPUT); // configure the trigger pin to output mode
  pinMode(ECHO_PIN, INPUT);  // configure the echo pin to input mode

  delay(2500);

  Serial.println("Connecting to Wifi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    currentMillis = millis(); // get the current "time" (actually the number of milliseconds since the program started)
    if ((currentMillis - startMillis) >= timeoutPeriod) {
      startMillis = currentMillis;
      connected = false;
      break; // break out of loop if 100 seconds passed without a connection
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    setupDeviceForWiFi();
  } else if (!connected) {
    // do nothing
  }

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) { // BH1750 Light Sensor Begin
    MTregVal = 69;
    lightMeter.setMTreg((byte)BH1750_DEFAULT_MTREG);
    Serial.println(F("BH1750 Light Sensor Activated"));
    tReadBH1750.enableDelayed(TASK_MINUTE);
  } else {
    Serial.println(F("Error initialising BH1750"));
  }

  pinMode(moisture_sensor_pin1, INPUT); // setup for the soil moisture sensor input pin (pin 36)
  pinMode(moisture_sensor_pin2, INPUT); // setup for the soil moisture sensor input pin (pin 36)
  pinMode(moisture_sensor_pin3, INPUT); // setup for the soil moisture sensor input pin (pin 36)

  lcd.init(); // LCD Begin code
  lcd.backlight();

  dhtSensor1.begin(); // DHT11 Temp and Humidity sensor begin
  dhtSensor2.begin(); // DHT11 Temp and Humidity sensor begin

  delay(2000);

  // initializing the rtc
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!");
    rtcCheck = false;
    Serial.flush();
    while (!rtc.begin()) {
      delay(10000);
      Serial.println("Couldn't find RTC!");
      currentMillis = millis(); // get the current "time" (actually the number of milliseconds since the program started)
      if ((currentMillis - startMillis) >= timeoutPeriod) {
        startMillis = currentMillis;
        Serial.println("ERROR: RTC not Working!");
        rtcCheck = false;
        break; // break out of loop if 100 seconds passed without a connection
      }
    }
  }
  if (rtc.lostPower()) {
    // this will adjust to the date and time at compilation
    Serial.println("Adjusting RTC to ");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  } else if (connected && !rtc.lostPower()) {
    setRTCWithNTC();
    rtcCheck = true;
  } else {
    Serial.println("ERROR: WiFi Connection failed & RTC is non-responsive!");
  }

  // Read sensor data from both sensors
  humidity1 = dhtSensor1.readHumidity();
  temperature1 = dhtSensor1.readTemperature();
  humidity2 = dhtSensor2.readHumidity();
  temperature2 = dhtSensor2.readTemperature();

  bool dhtCheck;

  delay(1000);

  // Check if DHT11 Sensor is reading
  if (isnan(humidity1) || isnan(temperature1)) {
    Serial.println("ERROR: Failed to read from DHT sensor1!");
    bool dhtCheck = false;
  } else {
    dhtCheck = true;
  }

  // Check if DHT11 Sensor is reading
  if (isnan(humidity2) || isnan(temperature2)) {
    Serial.println("ERROR: Failed to read from DHT sensor2!");
    bool dhtCheck = false;
  } else {
    dhtCheck = true;
  }

  delay(1000);

  // Calculate average readings
  avg_humidity = (humidity1 + humidity2) / 2;
  avg_temperature = (temperature1 + temperature2) / 2;

  delay(TASK_SECOND * 2);

  tTicker.enableDelayed(TASK_SECOND * 2);

  ts.startNow();
}

/**************************************************************************/
/*!
    @brief    Standard Arduino LOOP method - using with TaskScheduler there should be nothing here but ts.execute()
    @param    none
    @returns  none
*/
/**************************************************************************/
void loop() { ts.execute(); }

/**************************************************************************/
/*!
*/
/**************************************************************************/
bool Debug = false;

bool tickerEnable() {
  Debug = false;
  return true;
}

bool tickerDebugEnable() {
  Debug = true;
  Serial.println("tickerDebugEnable(): Setting up Debug Config");
  tTicker.setInterval(TASK_SECOND * 10);

  tPrintTimes.setInterval(TASK_SECOND * 5);
  tPrintTimes.enableDelayed(TASK_SECOND * 2);

  tReadMoisture.setInterval(TASK_SECOND * 2);
  tReadMoisture.enableDelayed(TASK_SECOND * 2);

  tReadDHT.restartDelayed();
  return true;
}

void tickerCallback() {
  tReadDHT.restartDelayed();
  if (tTicker.getRunCounter() % 3 == 0) {
    tReadUltrasonic.restartDelayed(TASK_SECOND);
  }

  if (tTicker.getRunCounter() % 4 == 0) {
    tWifiCheck.restartDelayed(TASK_SECOND * 5);
  }
}

bool testWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    connected = false;
    Serial.println("WiFi not connected");
    Serial.println("ERROR: Wifi connection failed!");
    Serial.println("Check your configuration\n");
    tPrintTimes.setCallback(&printRTCTime);
    return connected;
  } else if (WiFi.status() == WL_CONNECTED) {
    connected = true;
    tPrintTimes.setCallback(&printTime);
    return connected;
  } else {
    Serial.println("ERROR: Check WiFi connection!");
    return false;
  }
}

bool setupDHTReadings() {
  // Need to include code for upper and lower threshold status request Waiting methods
  // to alert to thresholds
  Serial.println("setupDHTReadings: Activating DHT sensors");
  measure.setWaiting(2); // Set the StatusRequest to wait for 2 signals.
  tCalculate.waitFor(&measure);

  screenValues.setWaiting(); // Set the StatusRequest to wait for
  tScreen.waitFor(&screenValues);

  tSensor1.restart();
  tSensor2.restart();

  return true;
}
void readDHTCallback() {
  // Serial.println("readDHTCallback: Invoked by calculate task or one second later");

  if (measure.pending()) {
    tCalculate.disable();
    Serial.println("readDHTCallback: Timeout!");
    tReadDHT.restart();

  } else {
    Serial.println("");
  }
}

void cleanDHTReadings() {
  // Serial.println("cleanDHTReadings: Cleaning up");
  tSensor1.disable();
  tSensor2.disable();
}

void CalcCallback() {
  Serial.println("CalcCallback: calculating Averages");
  if (measure.getStatus() >= 0) // only calculate if status request ended successfully
  {
    // Calculate average readings
    avg_humidity = (humidity1 + humidity2) / 2;
    avg_temperature = (temperature1 + temperature2) / 2;
    Serial.print("Average readings: ");
    Serial.print("Humidity = ");
    Serial.print(avg_humidity);
    Serial.print("%, Temperature = ");
    Serial.print(avg_temperature);
    Serial.print("C\n");
    screenValues.signalComplete(1);
  } else {
    Serial.println("CalcCallback: TIMEOUT");
    screenValues.signalComplete(-1);
  }
}

/** Simulation code for sensor 1
 *  ---------------------------
 */
bool S1Enable() {
  Serial.print("S1Enable: Triggering sensor1. \n");
  humidity1 = dhtSensor1.readHumidity();
  temperature1 = dhtSensor1.readTemperature();

  return true;
}

void S1Callback() {
  if (isnan(humidity1) || isnan(temperature1)) {
    Serial.println("ERROR: Failed to read from DHT sensor1!");
    bool dhtCheck = false;
    Serial.println("Restarting Sensor 1!");
    tSensor1.restart();
    return;
  } else {
    measure.signal();
    // Print sensor data and average readings
    Serial.print("Sensor 1: ");
    Serial.print("Humidity = ");
    Serial.print(humidity1);
    Serial.print("%, Temperature = ");
    Serial.print(temperature1);
    Serial.print("C\n");
  }
}

/** Simulation code for sensor 2
 *  ---------------------------
 */
bool S2Enable() {
  Serial.print("S2Enable: Triggering sensor2. \n");
  humidity2 = dhtSensor2.readHumidity();
  temperature2 = dhtSensor2.readTemperature();

  return true;
}

void S2Callback() {
  if (isnan(humidity2) || isnan(temperature2)) {
    Serial.println("ERROR: Failed to read from DHT sensor2!");
    bool dhtCheck = false;
    Serial.println("Restarting Sensor 2!");
    tSensor2.restart();
    return;
  } else {
    measure.signal();
    Serial.print("Sensor 2: ");
    Serial.print("Humidity = ");
    Serial.print(humidity2);
    Serial.print("%, Temperature = ");
    Serial.print(temperature2);
    Serial.print("C\n");
  }
}

void ScreenUpdateCallback() {
  // Display for the LCD
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(avg_temperature);
  lcd.print((char)223); // used to display degree symbol on display
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(avg_humidity);
  lcd.print("%");
  lcd.setCursor(0, 2);
  if (rtcCheck) {
    DateTime now = rtc.now();
    lcd.print("Date: ");
    lcd.print(now.day(), DEC);
    lcd.print('/');
    lcd.print(now.month(), DEC);
    lcd.print(" ");
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    lcd.print(now.minute(), DEC);
  } else {
    lcd.print("ERROR:RTC BROKEN!");
  }
  lcd.setCursor(0, 3);
  lcd.print("DaysSinceWater:");
  lcd.print(":)");

  checkTimeThreshold();
}

void printTime() {
  if (testWiFiConnection()) {
    connected = true;
    time_t nowSecs = time(nullptr);
    Serial.println("---------------------------------------------------------");
    Serial.println();
    struct tm timeinfo;
    localtime_r(&nowSecs, &timeinfo);
    Serial.print(F("Current time (BST): "));
    Serial.println(asctime(&timeinfo));
    printRTCTime();
    Serial.println("---------------------------------------------------------");
  } else {
    connected = false;
    Serial.println("---------------------------------------------------------");
    printRTCTime();
    Serial.println("---------------------------------------------------------");
  }
}

void setClock() {
  configTime(0, 3600, "uk.pool.ntp.org", "time.nist.gov");

  Serial.print(F("Waiting for NTP time sync: "));
  time_t nowSecs = time(nullptr);
  while (nowSecs < 10) {
    delay(500);
    Serial.print(F("."));
    yield();
    nowSecs = time(nullptr);
  }

  int LastPumpUnixSecondsDownload = ThingSpeak.readIntField(channelID, 4, "MM2WIMUEIAVZ3E0U");
  if (LastPumpUnixSecondsDownload == LastPumpUnixSeconds) {
    // do nothing
  } else {
    LastPumpUnixSeconds = LastPumpUnixSecondsDownload;
    Serial.println("------------------------------------------------");
    Serial.println("Reading Last Pump Time from ThingSpeak!");
    preferences.remove("pumpTime");
    size_t size = preferences.putUInt("pumpTime", LastPumpUnixSeconds);
    Serial.println(LastPumpUnixSeconds);
    Serial.println("= Last pump time flashed to ESP32 flash memory!");
    Serial.print("Size = ");
    Serial.println(size);
    Serial.println("------------------------------------------------");
  }

  printTime();
}

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void printRTCTime() {

  DateTime now = rtc.now();

  Serial.print("RTC time: ");
  Serial.print("(");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.day(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.year(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}

// Set the RTC to The NTP time
void setRTCWithNTC() {
  Serial.println("Setting RTC with NTP time");
  Serial.println("");
  time_t nowSecs = time(nullptr);
  struct tm timeinfo;
  localtime_r(&nowSecs, &timeinfo);

  // rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));

  printRTCTime();
}

const int OnHour = 12;
const int OnMin = 00;
const int OffHour = 14;
const int OffMin = 00;

// Function used to check if time is within threshold, to turn the lightbulb on. Returns true if within light on time, false if light should be off
bool checkTimeThreshold() {
  if (testWiFiConnection()) {
    time_t nowSecs = time(nullptr);
    Serial.println();
    struct tm timeinfo;
    localtime_r(&nowSecs, &timeinfo);
    Serial.print(F("Current time (BST): "));
    Serial.print(asctime(&timeinfo));

    printRTCTime();

    // Extract the minutes and hours variables
    int hours = timeinfo.tm_hour;
    int minutes = timeinfo.tm_min;
    Serial.println();

    if (hours >= OnHour && hours <= OffHour) {
      // Light stuff
      Serial.println("Light on");
      digitalWrite(lightPin, LOW); // Light on
      return true;
    } else {
      Serial.println("Light off");
      digitalWrite(lightPin, HIGH); // Light off
      return false;
    }
  } else {
    DateTime now = rtc.now();

    int hours = now.hour();
    int minutes = now.minute();

    Serial.print("Current Hours minutes HH:MM : ");
    Serial.print(hours);
    Serial.print(":");
    Serial.print(minutes);
    Serial.println();

    if (hours >= OnHour && hours <= OffHour) {
      // Light stuff
      Serial.println("Light on");
      digitalWrite(lightPin, LOW); // Light on
      return true;
    } else {
      Serial.println("Light off");
      digitalWrite(lightPin, HIGH); // Light off
      return false;
    }
  }
}

MovingAveragePlus<float> moistureAvg(45);

bool setupMoistureReadings() {
  digitalWrite(pumpPin, HIGH);
  return true;
}

// Function to check Soil Moisture Sensor Readings, performing actions as necassary
void checkSoilMoistThreshold() {
  moisture_value1 = analogRead(moisture_sensor_pin1);         // gets the value from the soil moisture sensor
  moisture_value1 = map(moisture_value1, 2500, 6000, 0, 100); // this sets the percentage value
  moisture_value2 = analogRead(moisture_sensor_pin2);         // gets the value from the soil moisture sensor
  moisture_value2 = map(moisture_value2, 2500, 6000, 0, 100); // this sets the percentage value
  moisture_value3 = analogRead(moisture_sensor_pin3);         // gets the value from the soil moisture sensor
  moisture_value3 = map(moisture_value3, 2500, 6000, 0, 100); // this sets the percentage value

  if (Debug) {
    Serial.print("Soil moisture: ");
    Serial.print(moisture_value1); // print the percent of soil moisture
    Serial.println("%");
    Serial.print("Soil moisture: ");
    Serial.print(moisture_value2); // print the percent of soil moisture
    Serial.println("%");
    Serial.print("Soil moisture: ");
    Serial.print(moisture_value3); // print the percent of soil moisture
    Serial.println("%");
    Serial.print("Run Counter:");
    Serial.println(tReadMoisture.getRunCounter());
  }

  moistureAvg.push(moisture_value1);
  moistureAvg.push(moisture_value2);
  moistureAvg.push(moisture_value3);
  if (tReadMoisture.getRunCounter() % 16 == 0) {
    moistureCombinedAvg = moistureAvg.get();
    if (moistureCombinedAvg < moistureThreshold) // if the soil is dry then pump out water for 10 second
    {
      Serial.print("Avg Soil moisture: ");
      Serial.print(moistureCombinedAvg); // print the percent of soil moisture
      Serial.println("%");
      tPump.enable();
    } else {
      digitalWrite(pumpPin, HIGH);
      Serial.println("Do not activate pump");
    }
  }
}

void checkSoilMoistThresholdWhenWatering() {}

bool pumpCallbackEnableOn() {

  DateTime now = rtc.now();

  Serial.print("TPump Enabled");

  currentUnixSeconds = now.unixtime();

  Serial.print("Hours Since last water: ");
  Serial.print((currentUnixSeconds - LastPumpUnixSeconds) / 3600);
  Serial.println(" Hours!");

  if ((currentUnixSeconds - LastPumpUnixSeconds) / 3600 < 2 && (currentUnixSeconds - LastPumpUnixSeconds) / 3600 > 1) {
    Serial.println("Moisture level before pump:");
    Serial.print(moistureCombinedAvgAtPump);
    Serial.print("%");
    Serial.println("Moisture level currently:");
    Serial.print(moistureCombinedAvg);
    Serial.print("%");
    if (moistureCombinedAvgAtPump > moistureCombinedAvg) {
      Serial.println("Moisture Levels have not increased!!");
      Serial.println("ERROR in PUMP!");
      return true;
    }

    return false;
  }

  if ((currentUnixSeconds - LastPumpUnixSeconds) / 3600 < 12) {
    Serial.print(" Watering triggered too soon after last ");
    Serial.print((currentUnixSeconds - LastPumpUnixSeconds) / 3600);
    Serial.println(" Hours since ");
    return false;
  } else if ((currentUnixSeconds - LastPumpUnixSeconds) / 3600 < 60) {
    Serial.print(" Watering triggered too soon after last ");
    Serial.print((currentUnixSeconds - LastPumpUnixSeconds) / 3600);
    Serial.println(" Hours since ");
    return false;
  } else if ((currentUnixSeconds - LastPumpUnixSeconds) / 3600 < 84) {
    Serial.print(" Watering triggered too soon after last ");
    Serial.print((currentUnixSeconds - LastPumpUnixSeconds) / 3600);
    Serial.println(" Hours since ");
    return false;
  } else if (waterTankLevel < 5) {
    Serial.print("Water tank level too low to water without burning out pump: ");
    Serial.print(waterTankLevel);
    Serial.println("% ");
    return false;
  } else {
    Serial.println("Activating Pump!");
  }

  return true;
}

void pumpCallback() {
  Serial.println("Setting Moisture level at pump on:");
  Serial.print(moistureCombinedAvg);
  moistureCombinedAvgAtPump = moistureCombinedAvg;

  Serial.println("Setting Time for pump");

  DateTime now = rtc.now();

  Serial.print(" Current Epoch Time = ");
  Serial.print(now.unixtime());
  LastPumpUnixSeconds = now.unixtime();
  Serial.print("seconds = ");
  Serial.print(now.unixtime() / 86400L);
  Serial.println("d");

  LastPumpMonth = now.month();
  LastPumpDay = now.day();
  LastPumpHours = now.hour();
  LastPumpMinutes = now.minute();

  Serial.print("Current Day/Month: ");
  Serial.print(LastPumpDay);
  Serial.print('/');
  Serial.print(LastPumpMonth);
  Serial.println(" Current Time HH:MM: ");
  Serial.print(LastPumpHours);
  Serial.print(":");
  Serial.print(LastPumpMinutes);
  Serial.println(" !");

  if (testWiFiConnection()) {
    time_t nowSecs = time(nullptr);
    Serial.println();
    struct tm timeinfo;
    localtime_r(&nowSecs, &timeinfo);
    Serial.print(F("Current time (BST): "));
    Serial.print(asctime(&timeinfo));
    Serial.println();
  }

  digitalWrite(pumpPin, LOW);
  Serial.println("Pump on for 4 second");
  delay(4000); // run pump for 10 second;
  digitalWrite(pumpPin, HIGH);
  Serial.println("Pump off, Waiting 60 minutes to retake Moisture readings");
  //    delay(300000); //wait 5 minutes before checking again
}

void pumpCallbackEnableOff() { digitalWrite(pumpPin, HIGH); }

MovingAveragePlus<float> luxAverage(9);

// Checks Lux Value
void checkBH1750Callback() {
  if (lightMeter.measurementReady(true)) {
    float lux = lightMeter.readLightLevel();
    luxAverage.push(lux);

    if (tReadBH1750.getRunCounter() % 10 == 0) {
      Serial.println(F("--------------------------------------"));
      Serial.print(F("Light: "));
      Serial.print(luxAverage.get());
      Serial.println("lx");
      Serial.println(F("--------------------------------------"));

      if (luxAverage.get() < 0) {
        Serial.println(F("Error condition detected"));
      } else {
        if (luxAverage.get() > 40000.0 && MTregVal != 32) {
          // reduce measurement time - needed in direct sun light
          if (lightMeter.setMTreg(32)) {
            MTregVal = 32;
            Serial.println(F("Setting MTReg to low value for high light environment"));
          } else {
            Serial.println(F("Error setting MTReg to low value for high light environment"));
          }
        } else {
          if (luxAverage.get() > 10.0 && luxAverage.get() < 40000.0 && MTregVal != 69) {
            // typical light environment
            if (lightMeter.setMTreg(69)) {
              MTregVal = 69;
              Serial.println(F("Setting MTReg to default value for normal light environment"));
            } else {
              Serial.println(F("Error setting MTReg to default value for normal light environment"));
            }
          } else {
            if (luxAverage.get() <= 10.0 && MTregVal != 138) {
              // very low light environment
              if (lightMeter.setMTreg(138)) {
                MTregVal = 138;
                Serial.println(F("Setting MTReg to high value for low light environment"));
              } else {
                Serial.println(F("Error setting MTReg to high value for low light environment"));
              }
            }
          }
        }
      }
    }
  }
}

// Establish ThingSpeak connection and set clock from NTP time
void setupDeviceForWiFi() {
  if (testWiFiConnection()) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    ThingSpeak.begin(client);

    wifiEnabled.signal(1);

    setClock();
  }
}

MovingAveragePlus<float> waterLevelAvg(5);

// Function for taking Ultrasonic Sensor Readings
void checkUltrasonic() {
  // Clears the trigPin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  distance_cm = 0.017 * duration_us;

  waterLevelAvg.push(distance_cm);

  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
}

void ultrasonicOnDisable() {
  float distance = waterLevelAvg.get();
  Serial.print("Mapping Average Distance: ");
  Serial.print(distance);
  Serial.println(" cm to");
  Serial.print("Water Tank Percentage:");

  if (distance > 33) {
    Serial.println("Water level is > 33cm which is an invalid value");
    tReadUltrasonic.restart();
  } else if (distance < 0) {
    Serial.println("Water level is < 0cm which is an invalid value");
    tReadUltrasonic.restart();
  }

  waterTankLevel = map(distance, 1, 33, 100, 0); // this sets the percentage value
  Serial.print(waterTankLevel);
  Serial.println("%");
}

// Function for Writing Sensor Data to ThingSpeak Fields
void WriteDataToThingSpeak() {
  ThingSpeak.setField(tempFieldNo, avg_temperature);
  ThingSpeak.setField(humidityFieldNo, avg_humidity);

  ThingSpeak.setField(soilMoistureFieldNo, moistureAvg.get());

  ThingSpeak.setField(luxLevelsFieldNo, luxAverage.get());

  ThingSpeak.setField(waterTankLevelsFieldNo, waterTankLevel);

  ThingSpeak.setField(4, LastPumpUnixSeconds);

  int writeSuccess = ThingSpeak.writeFields(channelID, writeAPIKey);

  Serial.println("------------------------------------------------");
  Serial.println("Writing Sensor Data to ThingSpeak!");
  Serial.println("------------------------------------------------");
}
