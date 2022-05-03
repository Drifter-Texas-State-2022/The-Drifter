/*************************************************** 
 * Drifter 2.5 Engineers: Jacob Smith, Amber Fergusson, Rod Michael, Rodolfo Montero
 * Version 7.3.3
 * ESP32 Firebeetle micro controller
 *
 * Additional libraries used: ESP32, Adafruit EPD, Adafruit LIS3DH,
 * RTC formating options http://www.cplusplus.com/reference/ctime/strftime/
 * Parts of code below were taken from Adafruit's website:
 * https://learn.adafruit.com/adafruit-2-13-eink-display-breakouts-and-featherwings
 * Parts of code were also taken from the ESP32 sample library:
 * File path in Arduino: File/Examples/SD/Datalogger
 * External Temperature Sensor code based on the Dallas Temperature Library example.
 * Internal Temp Code: Adafruit DHT Sensor Library
 * Bluetooth: ESP32 example Code
 ****************************************************/

// Libraries needed to run the code
#include "Adafruit_EPD.h"
#include "math.h"
#include "Adafruit_INA219.h"
#include "RTClib.h"
#include <DHT.h>
#include <SD.h>
#include <SPI.h>
#include <LoRa.h>
#include <ESP32Time.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

RTC_PCF8523 etc;
ESP32Time rtc;                          
#define COLOR1 EPD_BLACK
#define COLOR2 EPD_RED

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Pin assigments for the ESP32 Firebeetle board
#define EPD_CS      27
#define EPD_DC      26
#define SRAM_CS     25
#define cardSelect  15         // SD card
#define EPD_RESET   12         // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY    14         // can set to -1 to not use a pin (will wait a fixed delay)
#define SCL         22         // I2C Clock Pin                                  
#define SDA         21         // I2C Data Pin
#define CLK         18         // Universal Pins
#define MISO        19
#define MOSI        23
#define ET_ONE_WIRE_BUS 4      // External Temp Sensor
#define DHTPIN       0         // Internal Temp Sensor
#define ss          5          // Define the pins used by the transceiver module
#define rst         13
#define dio0        2
const int analogInPin = 35;    // pH sensor analog pin
#define ADC_PIN 39
#define OFFSET 43
#define address 100            //default I2C ID number for EZO EC Conudctivity Sensor Circuit.

Adafruit_LIS3DH lis = Adafruit_LIS3DH();
OneWire oneWire(ET_ONE_WIRE_BUS);    // Setup a oneWire instance to communicate with any OneWire devices
#define DHTTYPE DHT11                // DHT 11 external temp sensor.
DHT dht(DHTPIN, DHTTYPE);
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor
Adafruit_SSD1680 display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
BLEServer *pServer = NULL;         // Bluetooth Setup
BLECharacteristic * pTxCharacteristic;
Adafruit_INA219 ina219_1(0x40); //Solar Panel
Adafruit_INA219 ina219_2(0x41); //Charge Board
Adafruit_INA219 ina219_3(0x44); //Battery

String Hour,                      // Variables to set time and date using the onboard RTC.
       Min,
       Sec,
       AP,
       Date,
       fileName,                  // .CSV file name.
       outgoing,                  // Data or text line sent over to LoRa transmitter.
       inputString,               // Conductivity sensor 'r' character read string.
       condData,
       loraCheck = "";
bool   sentB,                     // Check to see if Drifter has recieved a bluetooth connection from a device.
       bluetooth,                 // Display board check to show if a bluetooth connection has been made.
       deviceConnected = false,   // Bluetooth checks.
       oldDeviceConnected = false;
char   conductivityChArray[20] = {0},
       StrArray[128],             // Array to hold the bluetooth data.
       ec_data[32],               // Make a 32 byte character array to hold incoming data from the EC circuit.
       StrArrayB[128];            // Copy of StrArray to hold the bluetooth data.
char  *ec,                        // Char pointers used in string parsing.
      *tds,                       
      *sal,                       
      *sg;                        
float  iTemp = 0,                 // Sensor data variables.
       iHumidity = 0,
       eTemp = 0,
       pH = 0,
       xAccelerometer = 0.0,
       yAccelerometer = 0.0,
       zAccelerometer = 0.0,
       salinity = 0,
       conductivityC = 0,
       conductivity= 0,
       TDS = 0,
       takeVoltage = 0,           // Test variables for power subsytem readings.
       takeCurrent = 0,
       voltage_1 = 0,
       voltage_2 = 0,
       voltage_3 = 0,
       current_1 = 0,
       current_2 = 0,
       current_3 = 0;
int    buf[10],                   // Buffer array for pH values in PH function.
       temp,                      // Variable to store buffer array value in PH function.
       LED_PIN = 16,              // Sleep cycle status LED.
       hum = 0,                   // Incrementer variable to get humidty value in another getIntTemp() function.             
       scanInt = 5000,            // Scan interval for sending over the data via LoRa. 5 seconds.
       scanIntB = 90000,          // Scan interval for sending over the data via Bluetooth. 90 seconds.
       time_ = 570,               // Used to change the delay needed depending on the command sent to the EZO Class EC Circuit.
       TimeAwake = 0,             // Time it took the Drifter to completer one cycle.
       percentage = 0,            // Percentage level of the battery.
       takeCondData = 0;          // Incrementer for taking conductivity, salinity, and TDS from getConductivity(takeCondData) function
long   SleepDuration = 30,        // Sleep time in minutes
       StartTime = 0,             // Time at the beginning of a cycle.
       lastSendTime = 0,          // Set the starting time for the LoRa transmission process in the void loop.
       lastSendTimeB = 0;         // Set the starting time for the Bluetooth transmission process in the void loop.
unsigned long int avgValue;       // pH sensor calculation variable.
byte localAddress = 0xDD,         // LoRa board address on the Drifter.
     destination  = 0xBB,         // LoRa board address on the Base Station.
     in_char = 0,                 // Used as a 1 byte buffer to store inbound bytes from the EC Circuit.
     icount = 0,                  // Counter used for ec_data array.
     code = 0;                    // Used to hold the I2C response code for conductivity sensor.

//################ Variables Stored int the ESP32 Flash Memory ###########################
// These variables will keep their values upon the Drifter coming back out of sleep mode
RTC_DATA_ATTR int bootCount = 0;        // the boot counter will be reset to 0 only after a hard RESET
RTC_DATA_ATTR int count;                // count variable for count and parse functions
RTC_DATA_ATTR String tCheck;            // The ETC time value
RTC_DATA_ATTR String tCheck2;           // The ETC date value
RTC_DATA_ATTR int TimeAwakeCheck = 0;   // Copy of the TimeAwake variable used to correct the ETC time
RTC_DATA_ATTR int rebootTimeCheck = 0;  // Variable that makes sure the time is set only once at startup

class MyServerCallbacks: public BLEServerCallbacks {  // Class for the bluetooth send and receive functions
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {  // Class for the bluetooth send and receive functions
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        sentB = true;
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};

/*
 * Initializes the serial monitor, LoRa board, external time clock, LED, and sensors.
 * The RTC time and date will be hard coded on start up.
 * An LED is powered on for the entirety of the sampling process until the Drifter enters sleep mode.
 * Each sensor is called to be setup and ready to take data.
 */
void setup()
{ 
  int pass = 0;
  Serial.begin(115200);
  LoRa.setPins(ss, rst, dio0);
  Serial.println("Drifter started with Boot number:" + String(bootCount));
  
  bool statusETC = etc.begin();
  if (! statusETC) {
    Serial.println("Couldn't find ETC");
    Serial.flush();
    while (1) delay(10);
  }

  StartTime = millis();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);                      //LED on when out of sleepmode

  ina219_1.begin(); ina219_1.setCalibration_16V_400mA ();
  ina219_2.begin(); ina219_2.setCalibration_16V_400mA ();
  ina219_3.begin(); ina219_3.setCalibration_16V_400mA ();
  
  BatteryCharge();
  BLESetup();                                     // Sets up the Bluetooth connection
  accelSetup();                                     // Accelerometer setup
  sensors.begin();                                  // External temp sensor startup
  dht.begin();                                      // Internal temp sensor startup
  //Wire.begin();                                   // Conductivity sensor startup                                   
  LoRa.setPins(ss, rst, dio0); 
  
  while (!LoRa.begin(915E6)) {
    Serial.println(".");
    delay(500);
    }
  // Change sync word (0xF3) to match the receiver
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

/*
 * Takes in nothing.
 * The void loop function repeatedly calls the list of functions below it to perform various actions.
 * This is continuously done within a 30 minute sampling window.
 * returns nothing.
 */
void loop()
{
  bool acknowledgeL = false;                            // Reset LoRa data sent ankowledge back to false
    if(bootCount == 0){
      rtc.setTime(00, 30, 12, 21, 04, 2022);            // 1st Feb 2022 10:30:00
      Serial.println(rtc.getTime("%a, %x %H:%M:%S"));   // (String) returns time with specified format 
      etc.adjust(DateTime(F(__DATE__), F(__TIME__)));   // Sets time based off of computer
      EtcLostPower_Calibrate(); 
      formatFile();
    }
    /*if(bootCount == 0 && percentage < 5){               // Check to see if the Drifter has rebooted and the battery level is above 5 percent
      File logfile;
      if (!SD.begin(cardSelect)) {
        Serial.println("Card init. failed!");
      }
      else
      {    
        char filename[15];
        strcpy(filename, "/DATALOG.CSV");
        String fileCopy = "/DATALOG.CSV";
        digitalWrite(33, HIGH);
        logfile = SD.open(filename, FILE_APPEND);
        logfile.print("Total Power Loss Detected...\n");
        digitalWrite(33, LOW);
        outgoing = "Total Power Loss Detected...\n";
        LoRa.setSpreadingFactor(7);
        onSend(outgoing);
        int rebootTimeCheck = 1;
        count = count + 1;
        BeginSleep();
      }
    }*/
    changeTimeDelay(bootCount, TimeAwakeCheck, rebootTimeCheck);
    bootCount += 1;
    Serial.println("Drifter started with Boot number:" + String(bootCount));
    Serial.print("This is the Date: ");
    Serial.println(tCheck2);
    Serial.print("This is the new Time: ");
    Serial.println(tCheck);
    Serial.println("Adafruit EPD test");
    Serial.println("setup done");
    display.begin();
    display.clearBuffer();
    set_date_time(tCheck2, tCheck, COLOR1);  
    percentage = BatteryCharge();
    voltage_1 = CurrentSensor(takeVoltage, takeCurrent);
    takeVoltage = takeVoltage + 1;
    voltage_2 = CurrentSensor(takeVoltage, takeCurrent);
    takeVoltage = takeVoltage + 1;
    voltage_3 = CurrentSensor(takeVoltage, takeCurrent);
    takeVoltage = takeVoltage + 1;
    current_1 = CurrentSensor(takeVoltage, takeCurrent);
    takeCurrent = takeCurrent + 1;
    current_2 = CurrentSensor(takeVoltage, takeCurrent);
    takeCurrent = takeCurrent + 1;
    current_3 = CurrentSensor(takeVoltage, takeCurrent);
    takeCurrent = takeCurrent + 1;                                  // Take this line out in the final code version
    CurrentSensor(takeVoltage, takeCurrent);                        // Take this line out in the final code version
    Serial.println("All voltage and Current Data Taken.");
    xAccelerometer = getAccelX();
    yAccelerometer = getAccelY();
    zAccelerometer = getAccelZ();
    if(zAccelerometer <= 0)                                         // Check to see if the Drifter is upside down 
      count = count + 1;                                            // If true then add 1 to count since an error log has been sent to the SD card within the getAccelZ() function
    eTemp = getExternalTemp();
    iTemp = getIntTemp(hum);
    hum += 1;
    iHumidity = getIntTemp(hum);
    pH = getPH();
    conductivity = getConductivity(takeCondData);
    takeCondData += 1;        
    TDS = getConductivity(takeCondData);
    takeCondData += 1;         
    salinity = getConductivity(takeCondData);
    salinity = salinity * .01;          // move the decimal place over two for correct value  
    delay(50);
    fileName = logFile(tCheck, tCheck2, iTemp, iHumidity, conductivity, eTemp, salinity, pH, xAccelerometer, yAccelerometer, zAccelerometer, TDS, percentage, voltage_1, voltage_2, voltage_3, current_1, current_2, current_3, bootCount);
    count = getCount(fileName, count);
    outgoing = SendLoRa(fileName, count);
    LoRa.setSpreadingFactor(7);
    onSend(outgoing);
    int scan = 0; 
    bool sent = false;
    lastSendTime = millis();
    while((millis() - lastSendTime < scanInt) && sent != true){
      if (scan == 0)
      {
        Serial.println("Scanning....");
        scan = 1;
      }
      loraCheck = onReceive(LoRa.parsePacket());
      if(loraCheck != "")
      {
        Serial.print("This is loraCheck: ");
        Serial.println(loraCheck);
        sent = true;
        acknowledgeL = true;
      }
    }
    delay(2000);
    lastSendTimeB = millis();
    sentB = connectBLE(deviceConnected, oldDeviceConnected);
    while(millis() - lastSendTimeB < scanIntB){
      if(sentB == true)
        bluetooth = sendBLE(fileName, count, StrArray);
        sentB == false;
      break;
      }
    DrawBattery();
    set_sensor_data(iTemp, iHumidity, conductivity, eTemp, salinity, pH, xAccelerometer, yAccelerometer, zAccelerometer, TDS, COLOR1);
    communications(acknowledgeL, bluetooth, COLOR1);
    display.display();
    delay(5000);
    TimeAwakeCheck = (millis() - StartTime);
    BeginSleep();
    Serial.println("Dont read me");
}

/*
 * Takes in bootCount, the number of cycles the Drifter has gone through; TimeAwake,  
 * the total time the Drifter was on last cycle; and rebootTimeCheck, a copy of the 
 * TimeAwake value for calculations purposes.
 * Fixes the external time clock delay due to the Drifter going to sleep.
 * returns nothing
 */
void changeTimeDelay(int bootCount, int TimeAwakeCheck, int rebootTimeCheck)
{
  int  tCheckHours = 0,
       tCheckMinutes = 0,
       tCheckSeconds = 0,
       tCheckTotalSeconds = 0,
       tCheckTotalMinutes = 0,
       tCheckTotalHours = 0;
  char tCheckChar[10];
  
  if(bootCount == 0 && rebootTimeCheck != 1)
    {
      etc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Sets time based off of computer 
    }
    DateTime now = etc.now();
    char buf1[] = "hh:mm:ss";
    char buf2[] = "MM-DD-YYYY";
    tCheck = (now.toString(buf1));
    tCheck2 = (now.toString(buf2));
    Serial.println("setup done");
    Serial.print("This is the ETC time: ");
    Serial.println(tCheck);
    int mTime = 0;
    int minutesChange = 0;
    int minutesOverflow = 0;
    
    if(bootCount >= 1)
    {
      tCheck = (now.toString(buf1));
      Serial.println("This is tCheck: ");
      Serial.println(tCheck);
      tCheck.toCharArray(tCheckChar,10);
      tCheckHours = atoi(&tCheckChar[0]);
      if(tCheckHours >= 12)
        mTime = 12;
      Serial.println(tCheckHours);
      tCheckMinutes = atoi(&tCheckChar[3]);
      Serial.println(tCheckMinutes);
      if(tCheckMinutes == 0)
        tCheckMinutes = 59;
      tCheckSeconds = atoi(&tCheckChar[6]);
      Serial.println(tCheckSeconds);
      if(tCheckHours == 0)
        tCheckHours == 12 * 3600;
      if(tCheckHours < 12)
        tCheckHours = tCheckHours * 3600;
      else
        tCheckHours = (tCheckHours-12) * 3600;
      Serial.println("tCheckHours value: ");
      Serial.println(tCheckHours);
      Serial.println("tCheckMinutes value: ");
      tCheckMinutes = (tCheckMinutes * 60) + (minutesChange * 60);   // multiply to get total minutes to seconds. Add 60 more seconds for rollover if tCheckMinutes above is 0.
      Serial.println(tCheckMinutes);
      int tCheckTotal = tCheckHours + tCheckMinutes + tCheckSeconds;
      int tCheckTotalCpy = tCheckTotal - TimeAwake;
      Serial.print("Total Seconds added: ");
      Serial.println(tCheckTotal);
      tCheckTotalSeconds = tCheckTotalCpy%60;
      Serial.print("After the modulo conversion:");
      Serial.println(tCheckTotalSeconds);
      TimeAwakeCheck = TimeAwakeCheck* .001;     // convert millis to seconds
      Serial.print("This is TimeAwake: ");
      Serial.println(TimeAwakeCheck);
      if(TimeAwakeCheck > tCheckTotalSeconds || tCheckTotalSeconds == 0)
      {
        tCheckTotalSeconds = (tCheckTotalSeconds + 60) - TimeAwakeCheck;
        minutesOverflow = 1;
      }
      else
        tCheckTotalSeconds = tCheckTotalSeconds-TimeAwakeCheck;
      Serial.print("Total Changed Seconds:");
      Serial.println(tCheckTotalSeconds);
      tCheckTotalHours = (tCheckTotalCpy / 3600) + mTime;
      Serial.print("Total Changed Hours: ");
      Serial.println(tCheckTotalHours);
      tCheckTotalMinutes = tCheckMinutes / 60 - minutesOverflow;
      Serial.print("Converted to Minutes: ");
      Serial.println(tCheckTotalMinutes);
      etc.adjust(DateTime(now.year(), now.month(), now.day(), tCheckTotalHours, tCheckTotalMinutes, tCheckTotalSeconds));
      tCheck2 = (now.toString(buf2));
      if(tCheckTotalHours < 10)
      {
        tCheck = String(0);
        tCheck.concat(tCheckTotalHours);
        tCheck.concat(':');
        if(tCheckTotalMinutes < 10)
          tCheck.concat(0);
        tCheck.concat(tCheckTotalMinutes);
        tCheck.concat(':');
        if(tCheckTotalSeconds < 10)
          tCheck.concat(0);
        tCheck.concat(tCheckTotalSeconds);
      }
      else
      {
        tCheck = String(tCheckTotalHours);
        tCheck.concat(':');
        if(tCheckTotalMinutes < 10)
          tCheck.concat(0);
        tCheck.concat(tCheckTotalMinutes);
        tCheck.concat(':');
        if(tCheckTotalSeconds < 10)
          tCheck.concat(0);
        tCheck.concat(tCheckTotalSeconds);
      } 
    }
}

/*
 * Takes in nothing.
 * Resets the time of the external time clock upon losing power from the coin cell battery.
 * Returns nothing.
 */
void EtcLostPower_Calibrate()
{
  bool statusETC = etc.begin();
  if (! statusETC) {
    Serial.println("Couldn't find ETC");
    Serial.flush();
    while (1) delay(10);
  }

  if (! etc.initialized() || etc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    // This line sets the ETC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
     etc.adjust(DateTime(2022, 5, 22, 1, 30, 0));
    //
    // Note: allow 2 seconds after inserting battery or applying external power
    // without battery before calling adjust(). This gives the PCF8523's
    // crystal oscillator time to stabilize. If you call adjust() very quickly
    // after the RTC is powered, lostPower() may still return true.
  }

  // When the ETC was stopped and stays connected to the battery, it has
  // to be restarted by clearing the STOP bit. Let's do this to ensure
  // the RTC is running.
  etc.start();
  //etc.calibrate(PCF8523_TwoHours, OFFSET);
}

/*
 * Doesn't take in any inputs
 * Reads the external temperature value in Celsius 
 * returns the value to the void loop function
 */
float getExternalTemp()
{
  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  float ExtTemp = 0;
  sensors.requestTemperatures(); 
  
  Serial.print("External Temperature: in Celsius: ");
  Serial.print(sensors.getTempCByIndex(0));
  Serial.println("C"); 
  ExtTemp = (sensors.getTempCByIndex(0));

  return (ExtTemp);
}

/*
 * Takes in hum that serves as an incrementer to return various data points for internal temperature and humidity.
 * Reads the internal temperature value in Celsius and humidity value in percentage.
 * returns the temperature and humidity values to the void loop function.
 */
float getIntTemp(int hum)
{
  float IntTemp = 0,
        humidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  IntTemp = t;

  Serial.print(F("Internal Temperature in Celsius: "));
  Serial.print(t);
  Serial.println(F("C"));
  Serial.print(F("Internal Humidity: "));
  Serial.println(humidity);
  
  if(hum == 0)
    return IntTemp;
  else
    return humidity;
}

/*
 * Takes in nothing.
 * Reads the pH value.
 * Returns the pH reading to the void loop function.
 */
float getPH()
{
  for(int i=0;i<10;i++) 
 { 
  buf[i]=analogRead(analogInPin);
  delay(10);
 }
 for(int i=0;i<9;i++)
 {
  for(int j=i+1;j<10;j++)
  {
   if(buf[i]>buf[j])
   {
    temp=buf[i];
    buf[i]=buf[j];
    buf[j]=temp;
   }
  }
 }
 avgValue=0;
 for(int i=2;i<8;i++)
 avgValue+=buf[i];
 float pHVol=(float)avgValue*5.0/1024/6;
 float phValue = 1.127 * pHVol ;
 Serial.print("pH sensor reading = ");
 Serial.println(phValue);

 return phValue;
}

/*
 * Takes in nothing.
 * Sets up the acclerometer to begin taking measurements.
 * Returns nothing.
 */
void accelSetup()
{
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_4_G);   // Can set to 2, 4, 8 or 16 G.

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  }
}

/*
 * Takes in nothing.
 * Reads the x-axis of the accelerometer.
 * Returns the x-axis value.
 */
float getAccelX()
{
  float AccelX = 0;
  lis.read();      // get X data
  // Then print out the raw data
  Serial.print("X:  "); Serial.print(lis.x);

  // new event to get acceleromter in m/s^2
  sensors_event_t event;
  lis.getEvent(&event);

  // Display the results (acceleration is measured in m/s^2) 
  Serial.print(" \tX: "); Serial.print(event.acceleration.x);
  Serial.println(" m/s^2 ");
  AccelX = event.acceleration.x;

  return AccelX;  
}

/*
 * Takes in nothing.
 * Reads the y-axis of the accelerometer.
 * Returns the y-axis value.
 */
float getAccelY()
{
  float AccelY = 0;
  lis.read();      // get Y data
  // Then print out the raw data
  Serial.print("Y:  "); Serial.print(lis.y);

  // new event to get acceleromter in m/s^2
  sensors_event_t event;
  lis.getEvent(&event);

  // Display the results (acceleration is measured in m/s^2) 
  Serial.print(" \tY: "); Serial.print(event.acceleration.y);
  Serial.println(" m/s^2 ");
  AccelY = event.acceleration.y;

  return AccelY;  
}

/*
 * Takes in nothing.
 * Reads the z-axis of the accelerometer.
 * Returns the z-axis value.
 */
float getAccelZ()
{
  float AccelZ = 0;
  lis.read();      // get Z data 
  // Then print out the raw data
  Serial.print("Z:  "); Serial.print(lis.z);

  // new event to get acceleromter in m/s^2
  sensors_event_t event;
  lis.getEvent(&event);

  // Display the results (acceleration is measured in m/s^2) 
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
  Serial.println(" m/s^2 ");
  AccelZ = event.acceleration.z;

  if ( lis.z <= 0)
  {
    bool Orient;
    Orient = true;
    File logfile;
    
    Serial.println("Drifter is upside down");
    char filename[15];
    strcpy(filename, "/DATALOG.CSV");
    String fileCopy = "/DATALOG.CSV";
    digitalWrite(33, HIGH);
    logfile = SD.open(filename, FILE_APPEND);
    logfile.print("Drifter is upside down\n");
    digitalWrite(33, LOW);
    outgoing = "Drifter is upside down\n";
    LoRa.setSpreadingFactor(7);
    onSend(outgoing);
  }
  else
    Serial.println("Drifter is right side up");

  return AccelZ;  
}

/*
 * Takes in takeCondData which is an incrementer variable that returns various points back to the void loop function.
 * Reads the Conductivity, total dissolved solids, and salinity values.
 * returns these values one at a time using the incrimenter variable to the void loop.
 */
float getConductivity(int takeCondData)
{
  int cond,
      TDS,
      salinity;
  inputString = 'r';                                                  // Set the conductivity sensor to always be in read mode.
    Wire.beginTransmission(address);                                  // Call the circuit by its ID number.    
    Wire.write(inputString.c_str());                                  // Transmit the command 'r' to tell the sensor to read the data.    
    Wire.endTransmission();                                           // End the I2C data transmission.


    if (strcmp(inputString.c_str(), "sleep") != 0) {                  // Wait the correct amount of time and request data.
                                                                      
      delay(time_);                                                   // Wait the correct amount of time for the circuit to complete its instruction.

      Wire.requestFrom(address, 32, 1);                               // Call the circuit and request 32 bytes (this could be too small, but it is the max i2c buffer size for an Arduino)
      code = Wire.read();                                             // The first byte is the response code, we read this separately.

      switch (code) {                                                 // Switch case based on what the response code is.
        case 1:                                                       // Decimal 1.
          Serial.println("Conductivity Read Success");                // Means the command was successful.
          break;                                                      // Exits the switch case.

        case 2:                                                       // Decimal 2.
          Serial.println("Conductivity Read Failed");             //means the command has failed.
          break;                                //exits the switch case.

        case 254:                               //decimal 254.
          Serial.println("Conductivity Read Pending");            //means the command has not yet been finished calculating.
          break;                                //exits the switch case.

        case 255:                               //decimal 255.
          Serial.println("Conductivity Read No Data");            //means there is no further data to send.
          break;                                //exits the switch case.

      }

      while (Wire.available()) {                 //are there bytes to receive.
        in_char = Wire.read();                   //receive a byte.
        ec_data[icount] = in_char;                    //load this byte into our array.
        icount += 1;                                  //incur the counter for the array element.
        if (in_char == 0) {                      //if we see that we have been sent a null command.
          icount = 0;                                 //reset the counter i to 0.
          Wire.endTransmission();                //end the I2C data transmission.
          break;                                 //exit the while loop.
        }
      }

      Serial.println(ec_data);                  //print the data.
      Serial.println();                         //this just makes the output easier to read by adding an extra blank line 
    }
   
  String sendData = ec_data;
  ec = strtok(ec_data, ",");          //let's pars the string at each comma.
  tds = strtok(NULL, ",");            //let's pars the string at each comma.
  sal = strtok(NULL, ",");            //let's pars the string at each comma.
  sg = strtok(NULL, ",");             //let's pars the string at each comma.

  Serial.print("EC:");                //we now print each value we parsed separately.
  Serial.println(ec);                 //this is the EC value.

  Serial.print("TDS:");               //we now print each value we parsed separately.
  Serial.println(tds);                //this is the TDS value.

  Serial.print("SAL:");               //we now print each value we parsed separately.
  Serial.println(sal);                //this is the salinity value.

  Serial.print("SG:");               //we now print each value we parsed separately.
  Serial.println(sg);                //this is the specific gravity.
  Serial.println();                  //this just makes the output easier to read by adding an extra blank line
  cond = atoi(ec);
  TDS = atoi(tds);
  if(atoi(&sal[0]) == 0)
      salinity = atoi(&sal[2]);
  else{salinity = atoi(sal);}
  cond = float(cond);
  TDS = float(TDS);
  salinity = float(salinity);
   
  if(takeCondData == 0) 
    return cond;
  if(takeCondData == 1)
    return TDS;
  if(takeCondData == 2)
    return salinity;
  else
    return 0;
}

/*
 * Takes in the ETC date and time variables.
 * Sets these values to the e-ink display screen.
 * returns nothing.
 */
void set_date_time(String date, String cTime, uint16_t color){
  digitalWrite(27, HIGH);
  display.setCursor(0, 0); 
  display.setTextSize(2);  //8 pixels per new line font size 1, 16 for size 2
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(date);
  display.setCursor(0, 18); 
  display.print(cTime);
  digitalWrite(27, LOW);
}

/*
 * Takes in nothing.
 * Reads the voltage value coming from the battery on GPIO pin 36.
 * returns the battery voltage as a percentage.
 */
float BatteryCharge(){
  float voltage = analogRead(36) / 4096.0 * 7.06;
  if (voltage > 1 ) { // Only display if there is a valid reading
    Serial.println("Voltage = " + String(voltage)+ "V");
    percentage = 2836.9625 * pow(voltage, 4) - 43987.4889 * pow(voltage, 3) + 255233.8134 * pow(voltage, 2) - 656689.7123 * voltage + 632041.7303;
    if (voltage >= 4.20) percentage = 100;
    if (voltage <= 3.50) percentage = 0;
    Serial.println("Battery = " + String(percentage) + "%");
  }
  return percentage;
}

/*
 * Takes in nothing.
 * Uses the percentage reading of the voltage to generate a battery graphic on the e-ink display.
 * Returns nothing.
 */
void DrawBattery() {
  uint8_t percentage = 100;
  display.setCursor(20, 200); // x then y
  float voltage = analogRead(36)/ 4096.0 * 7.46;
  if (voltage > 1 ) { // Only display if there is a valid reading
    Serial.println("Voltage = " + String(voltage));
    percentage = 2836.9625 * pow(voltage, 4) - 43987.4889 * pow(voltage, 3) + 255233.8134 * pow(voltage, 2) - 656689.7123 * voltage + 632041.7303;
    if (voltage >= 4.20) percentage = 100;
    if (voltage <= 3.50) percentage = 0;
    display.drawRect(210 + 15, 12 - 12, 19, 10, COLOR1);
    display.fillRect(210 + 34, 12 - 10, 2, 5, COLOR1);
    display.fillRect(210 + 17, 12 - 10, 15 * percentage / 100.0, 6, COLOR1);
  }
  display.setCursor(170, 2);
  display.setTextSize(1);
  display.print("     "); 
  display.print(percentage);
  display.print("%");
}

/*
 * Takes the sensor data, formats the values, and sets them to the e-ink display.
 * Error checks are made to determine if a number is out of range or comes back as not a number.
 * Returns nothing.
 */
void set_sensor_data(float ITemp, float iHumidity, float Conductivity, float ETemp, float Salinity, float PH, float xMotion, float yMotion, float zMotion, float Tds, uint16_t color){
  digitalWrite(27, HIGH);
  display.setCursor(0, 58);  // 8 pixels per new line, font size 1 
  display.setTextSize(2);   // 16  pixels per new line, font size 2
  display.setTextColor(color);
  display.setTextWrap(true);
  if(ITemp > 0 && ITemp < 100){ 
    display.print("InT: ");
    display.print(ITemp, 1);
    display.print("C");
  }
  else{
    if(isnan(ITemp) == 1){ // check if value is not a number
      display.print("InT: N");
      Serial.println("Value of internal temperature sensor is not a number.");
    }
    else{
      display.print("InT: X");
      Serial.println("Value out of range for internal temperature sensor.");
    }
  }
  display.setCursor(0, 42);
  display.setTextSize(2);
  if(iHumidity > 0 && iHumidity <= 100)
  {
    display.print("Hum: "); 
    display.print(iHumidity);
    display.print("%");
  }
  else{
    if(isnan(iHumidity) == 1){ // check if value is not a number
      display.print("Hum: N");
      Serial.println("Value of Humidity sensor is not a number.");
    }
    else{
      display.print("Hum: X");
      Serial.println("Value out of range for external temperature sensor.");
    }
  }
  display.setCursor(0, 74);
  display.setTextSize(2);
  if(ETemp > -10 && ETemp < 100){
    display.print("ExT: ");
    display.print(ETemp, 1);
    display.print("C");
  }
  else{
    if(isnan(ETemp) == 1){ // check if value is not a number
      display.print("ExT: N");
      Serial.println("Value of external temperature sensor is not a number.");
    }
    else{
      display.print("ExT: X");
      Serial.println("Value out of range for external temperature sensor.");
    }
  }
  display.setCursor(135, 58);
  if(PH >= 3 && PH < 10){
    display.print("pH:   ");
    display.print(PH, 1);
  }
  else{
    if(isnan(PH) == 1){ // check if value is not a number
      display.print("pH:   N");
      Serial.println("Value of pH sensor is not a number.");
    }
    else{
      display.print("Ph:   X");
      Serial.println("Value out of range for pH sensor.");
    }
  }
  display.setCursor(0, 90);
  if(Conductivity >= 0 && Conductivity < 1000){
    display.print("Con: ");
    display.print(Conductivity, 1);
  }
  else{
    if(isnan(Conductivity) == 1){ // check if value is not a number
      display.print("Con: N");
      Serial.println("Value of conductivity sensor is not a number.");
    }
    else{
      display.print("Con: X");
      Serial.println("Value out of range for conductivity sensor.");
    }
  }
 
  display.setCursor(135, 74);
  if(Salinity >= 0 && Salinity < 20){
    display.print("Sal:  ");
    display.print(Salinity, 1);
  }
  else{
    if(isnan(Salinity) == 1){ // check if value is not a number
      display.print("Sal:  N");
      Serial.println("Value of salinity sensor is not a number.");
    }
    else{
      display.print("Sal:  X");
      Serial.println("Value out of range for salinity sensor.");
    }
  }
  
  display.setCursor(0, 106);
  display.print("Acc: ");
  if(xMotion > -10 && xMotion < 10){
    display.print(xMotion, 1);
  }
  else{
    if(isnan(xMotion) == 1){ // check if value is not a number
      display.print("N");
      Serial.println("Value of X-Axis accelerometer sensor is not a number.");
    }
    else{
      display.print("X");
      Serial.println("Value out of range for X-Axis accelerometer sensor.");
    }
  }
    if(yMotion > -15 && yMotion < 15){
    display.print(" ");
    display.print(yMotion, 1);
    }
    else{
      if(isnan(yMotion) == 1){ // check if value is not a number
        display.print("   N");
        Serial.println("Value of Y-Axis accelerometer sensor is not a number.");
      }
      else{
        display.print("   X");
        Serial.println("Value out of range for Y-Axis accelerometer sensor.");
      }
    }
    if(zMotion > -15 && zMotion < 15){
    display.print("  ");
    display.print(zMotion, 1);
    }
    else{
      if(isnan(zMotion) == 1){ // check if value is not a number
        display.print("   N");
        Serial.println("Value of Z-Axis accelerometer sensor is not a number.");
      }
      else{
        display.print("   X");
        Serial.println("Value out of range for Z-Axis accelerometer sensor.");
      }
    }
 
  display.setCursor(135, 90);
  if(Tds >= 0 && Tds < 500){
    display.print("TDS:  ");
    display.print(Tds, 1);
  }
  else{
    if(isnan(Tds) == 1){ // check if value is not a number
      display.print("TDS:  N");
      Serial.println("Value of total dissolved solids sensor is not a number.");
    }
    else{
      display.print("TDS:  X");
      Serial.println("Value out of range for total dissolved solids sensor.");
    }
  }
  digitalWrite(27, LOW);
}

/*
 * Takes in the voltage value and current incrementer values to return all the sensor readings.
 * Calculates the voltage and current going across the solar panel, battery and solar charge board.
 * Returns the voltage and current from each sensor board.
 */
float CurrentSensor( float takeVoltage, float takeCurrent){
  voltage_1 = ina219_1.getBusVoltage_V();
  voltage_2 = ina219_2.getBusVoltage_V();
  voltage_3 = ina219_3.getBusVoltage_V();

  current_1 = (abs(ina219_1.getCurrent_mA() / 50));
  current_2 = (abs(ina219_2.getCurrent_mA() / 77));
  current_3 = (abs(ina219_3.getCurrent_mA() / 69));

  if(takeVoltage == 0)
  {
    Serial.print("This is voltage_1: ");
    Serial.println(voltage_1);
    return voltage_1;
  }
  if(takeVoltage == 1)
  {
    Serial.print("This is voltage_2: ");
    Serial.println(voltage_2);
    return voltage_2;
  }
  if(takeVoltage == 2)
  {
    Serial.print("This is voltage_3: ");
    Serial.println(voltage_3);
    return voltage_3;
  }
  if(takeVoltage == 3 && takeCurrent == 0)
  {
    Serial.print("This is current_1: ");
    Serial.println(current_1);
    return current_1;
  }
  if(takeVoltage == 3 && takeCurrent == 1)
  {
    Serial.print("This is current_2: ");
    Serial.println(current_2);
    return current_2;
  }
  if(takeVoltage == 3 && takeCurrent == 2)
  {
    Serial.print("This is current_3: ");
    Serial.println(current_3);
    return current_3;
  }
  else
  {
    Serial.print("\nSolar Panel:");
    Serial.print("       Voltage:   "); Serial.print(voltage_1); Serial.print(" V");
    Serial.print("       Current:   "); Serial.print(current_1); Serial.print(" mA");
    Serial.print("       Power:   "); Serial.print(abs(voltage_1 * current_1)); Serial.println(" mW");

    Serial.print("Charge Board:");
    Serial.print("      Voltage:   "); Serial.print(voltage_2); Serial.print(" V");
    Serial.print("       Current:   "); Serial.print(current_2); Serial.print(" mA");
    Serial.print("       Power:   "); Serial.print(abs(voltage_2 * current_2)); Serial.println(" mW");

    Serial.print("Backup Battery:");
    Serial.print("    Voltage:   "); Serial.print(voltage_3); Serial.print(" V");
    Serial.print("       Current:   "); Serial.print(current_3); Serial.print(" mA");
    Serial.print("       Power:   "); Serial.print(abs(voltage_3 * current_3)); Serial.println(" mW");
    return 0;
  }
  delay(3000);
}

/*
 * Takes in nothing.
 * Sends a format line into the .csv file to label each column of data.
 * Returns nothing.
 */
void formatFile()
{
  File logfile;

  // see if the SD card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
    // Add an LED that turns on for 2 seconds and then turns off to tell the user to take the card out, delete the csv file, then reinsert card
  }
  char filename[15];
  strcpy(filename, "/DATALOG.CSV");
  String fileCopy = "/DATALOG.CSV";
  logfile = SD.open(filename, FILE_APPEND);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
  } 

  Serial.print("Writing to "); 
  Serial.println(filename);
  digitalWrite(33, HIGH);
  logfile.print("Time");
  logfile.print(",");
  logfile.print("Date");
  logfile.print(",");
  logfile.print("External Temperature");
  logfile.print(",");
  logfile.print("pH Level");
  logfile.print(",");
  logfile.print("Internal Temperature");
  logfile.print(",");
  logfile.print("Internal Humidity");
  logfile.print(",");
  logfile.print("X-Axis");
  logfile.print(",");
  logfile.print("Y-Axis");
  logfile.print(",");
  logfile.print("Z-Axis");
  logfile.print(",");
  logfile.print("Conductivity");
  logfile.print(",");
  logfile.print("Salinity");
  logfile.print(",");
  logfile.print("Total Dissolved Solids");
  logfile.print(",");
  logfile.print("Battery Percentage");
  logfile.print(",");
  logfile.print("Solar Panel Voltage");
  logfile.print(",");
  logfile.print("Solar Panel Current");
  logfile.print(",");
  logfile.print("SCB Voltage");
  logfile.print(",");
  logfile.print("SCB Current");
  logfile.print(",");
  logfile.print("Battery Voltage");
  logfile.print(",");
  logfile.print("Battery Current");
  logfile.print(",");
  logfile.print("Reboot From Sleep Number\n");
  Serial.print("Set up successful\n");
  digitalWrite(33, LOW);
}

/*
 * Takes in the all the ETC data and tiime, sensor data, and bootcount number.
 * Logs all the sensor data to a micro SD card and saves the data as a .csv file.
 * returns a char array pointer to the main void loop for the LoRa and Bluetooth functions.
*/
String logFile(String cTime, String Date, float ITemp, float Humidity, float Conductivity, float ETemp, float Salinity, float PH, float xMotion, float yMotion, float zMotion, float Tds, int percentage, float voltage_1, float voltage_2, float voltage_3, float current_1, float current_2, float current_3, int bootCount)
  {
  File logfile;

  // see if the SD card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
  }
  char filename[15];
  strcpy(filename, "/DATALOG.CSV");
  String fileCopy = "/DATALOG.CSV";
  logfile = SD.open(filename, FILE_APPEND);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
  }
  else
  {
  Serial.print("Writing to "); 
  Serial.println(filename);
 
  Serial.println("Ready!");

  digitalWrite(33, HIGH);
     logfile.print(cTime);
     logfile.print(",");
     logfile.print(Date);
     logfile.print(",");
     logfile.print(ETemp);
     logfile.print(",");
     logfile.print(PH);
     logfile.print(",");
     logfile.print(ITemp);
     logfile.print(",");
     logfile.print(Humidity);
     logfile.print(",");
     logfile.print(xMotion);
     logfile.print(",");
     logfile.print(yMotion);
     logfile.print(",");
     logfile.print(zMotion);
     logfile.print(",");
     logfile.print(Conductivity);
     logfile.print(",");
     logfile.print(Salinity);
     logfile.print(",");
     logfile.print(Tds);
     logfile.print(",");
     logfile.print(percentage);
     logfile.print(",");
     logfile.print(voltage_1);
     logfile.print(",");
     logfile.print(current_1);
     logfile.print(",");
     logfile.print(voltage_2);
     logfile.print(",");
     logfile.print(current_2);
     logfile.print(",");
     logfile.print(voltage_3);
     logfile.print(",");
     logfile.print(current_3);
     logfile.print(",");
     logfile.print(bootCount);
     logfile.print(",");
     logfile.print("\n");
   Serial.print("Data sent successfully!\n");
   digitalWrite(33, LOW);
   logfile.close();     
  }

  return fileCopy;
}

/*
 * Takes in the name of the csv file and the current line count from the previous cycle.
 * Takes the csv file and initial count value as arguements and opens the SD card .csv file to find the final line.
 * Returns an integer value that represents the final line in the file
 */
int getCount(String fileName, int count)
{
  File logfile;
  
  logfile = SD.open(fileName);
  int  c = count;
  char n;

  while((c = logfile.read()) >= 0);
  {
    if(n == '\n');
    count++;
  } 
  count--;  //sets the count to the last line of the SD card
  Serial.print("Count Start equals: ");
  Serial.println(count);
  for(int i = 0; i <= count; i++)
  {
    if (i < count)
    {
      logfile.readStringUntil('\n');
    }
    else
    {
      count++;
      Serial.print("The count is: ");
      Serial.println(count);
      logfile.close();
      return count;
    }
  }
}

/*
 * Takes the .csv file name and the value of count from the count function
 * parses the data until it reaches the value of count.
 * saves the corresponding data line to a String and returns it.
 */
String parseData(String fileName, int count)
{
  File logfile;
  logfile = SD.open(fileName);
  String dataSave;
  for(int i = 0; i <= count; i++)
  {
    if (i < count)
    {
      dataSave = logfile.readStringUntil('\n');
    }
    else
    {
      Serial.print("Read Line: ");
      Serial.println(i); 
      count++;
      dataSave = logfile.readStringUntil('\n');
      Serial.print("The count is set to: ");
      Serial.println(count);
      Serial.print("Data Taken: ");
      Serial.println(dataSave);
      logfile.close();
      return dataSave;
    }
  }
}

/*
 * Takes in nothing.
 * Sets up the onboard ESP32 bluetooth for communication.
 * Returns nothing.
 */
void BLESetup()
{
    // Create the BLE Device
  BLEDevice::init("DrifterBLE");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

/*
 * Takes in deviceConnected and oldDeviceConnected booleans.
 * Checks if the BLE is connected to a device
 * If connected returns true.
 * If flase, continuously searches until a device is found
 * Returns true or false.
 */
bool connectBLE(bool deviceConnected, bool oldDeviceConnected)
{
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
      return true;
    }
    // connecting
  if (deviceConnected && !oldDeviceConnected) {
  // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
    }
    else 
    return false;
}

/*
 * Takes in the .csv filename, the last line of the file, and the string array of the sensor data.
 * Takes the last data line of the file and sends it out to a connected bluetooth device.
 * Returns a boolean that's true or false depending on the transmission status.
 */
bool sendBLE(String fileName, int count, char StrArray[])
{
  String dataSave;
  bool check,           
       sent;        
  char dataArray[128];
  File logfile;
  logfile = SD.open(fileName);
  
  for(int i = 0; i <= count; i++)
  {
    Serial.print("Count is: ");
    Serial.println(count);
    if (i <= count)
    {
      connectBLE(deviceConnected, oldDeviceConnected);
      Serial.print("Read Line: ");
      Serial.println(i); 
      Serial.print("Data Taken: ");
      Serial.println(dataSave);
      dataSave = logfile.readStringUntil('\n');
      dataSave = dataSave + "\n";
      dataSave.toCharArray(dataArray, 128);
      strcpy(StrArray, dataSave.c_str());
      pTxCharacteristic->setValue(&StrArrayB[128]); // DO NOT CHANGE EVER!
      pTxCharacteristic->notify();
      Serial.print("Data Sent: ");
      Serial.println(dataSave);
      sent = true;
      // else  Add logic tht saves the data for a future transmission
    }
  }
  logfile.close();
  return sent; // returns a boolean determining if the data was sent properly
}

/*
 * Takes the fileName and the final line of the .csv file as arguements.
 * dataToTrans = parseData() is equal to the last data line of the .csv file.
 * Returns a String used as an acknoledgement check in the void loop.
 */
String SendLoRa(String fileName, int count)
{
  File logfile;
  String dataToTrans;
  
  digitalWrite(5, HIGH);
  logfile = SD.open(fileName);
  dataToTrans = parseData(fileName, count);
  Serial.print("Data Sent: ");
  Serial.println(dataToTrans);
  logfile.close();             
  digitalWrite(5, LOW);

  return dataToTrans; 
}

/*
 * Takes in outgoing represented as the data being sent to the base station.
 * Sends the data line to the base station if it has successfully recieved a transmission.
 * Returns nothing.
 */
void onSend(String outgoing) {
  Serial.print("Sending packet: ");
  Serial.println(outgoing);

  //Send LoRa packet to receiver
  digitalWrite(5,LOW);
  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(outgoing.length());
  LoRa.print(outgoing);
  LoRa.endPacket();
  digitalWrite(5,HIGH);
}

/*
 * Checks to see if a packet was recieved at the base station.
 * If true, the base station sends an ackowledge back that equals "ACK".
 * If the transciever receives "ACK", return a string with data or empty.
 * Returns a blank string or the data line that was sent.
 */
String onReceive(int packetSize) {
  if(packetSize){
    Serial.println("Receiving");
    digitalWrite(5,LOW);
    int recipient = LoRa.read();
    byte sender = LoRa.read();
    byte incomingLength = LoRa.read();
    String loraCheck = "";
    while (LoRa.available()) {
      loraCheck = LoRa.readString();
      Serial.println("Received: " + loraCheck);
      Serial.println("RSSI: " + String(LoRa.packetRssi()));
    }    
    if(recipient != localAddress)
    {
      Serial.println("This message is not for me");
      return "";
    }
    if(sender != destination)
    {
      Serial.println("This message is not from the Drifter");
      return "";
    }
    if(incomingLength != loraCheck.length())
    {
      Serial.println("Error: message length does not match expected length");
      return "";
    }
    Serial.println("Message received from 0x" + String(sender,HEX));
    Serial.println("Sent to: 0x" + String(recipient,HEX));
    digitalWrite(5,HIGH);
    return loraCheck;
  }
  else {
    digitalWrite(5,HIGH);
    return "";}
}

/*
 * Takes in two booleans initialy set to false and the ink color of the display board.
 * Verifies if a connection to the Bluetooth and LoRa devices are made.
 * It then displays this data to the e-ink dislpay board. 
 * Returns nothing.
 */
void communications(bool acknowledgeL, bool bluetooth, uint16_t color){
  digitalWrite(27,HIGH);
  display.setCursor(135, 20); 
  display.setTextSize(4);  
  display.setTextColor(color);
  display.setTextWrap(true);
  if(bluetooth == true){
    display.print("B");
    Serial.println("Bluetooth connection is online");
  }
  else{
    display.print("X");
    Serial.println("Bluetooth connection is not establishing properly.");
  }
  display.setCursor(205, 20);
  if(acknowledgeL == true){
    display.print("L");
    Serial.println("LoRa connection is online");
  }
  else{
    display.print("X");
    Serial.println("LoRa connection is not establishing properly.");
  }
  digitalWrite(27, LOW);
}

/*
 * Takes in nothing.
 * Calculates the time it took the Drifter to complete one full cycle.
 * Sets the Drifter into a 30 minute sleep to conserve power.
 * Returns nothing.
 */
void BeginSleep() {
  TimeAwake = (millis() - StartTime);
  long SleepTimer = (((SleepDuration * 60 ) * 1000000LL)- TimeAwake);
  int TimeAwakeSec = ((TimeAwake / 1000), 0);
 
  esp_sleep_enable_timer_wakeup(SleepTimer);  
  
  Serial.println("\n\nEntering " + String(SleepDuration) + " minutes of sleep time");
  if(TimeAwake < 60000){
    Serial.println("Awake for : " + String(TimeAwake / 1000.0) + " seconds");}
  else{
    Serial.println("Awake for : " + String(TimeAwake / 60000.0) + " minutes");}
  
  Serial.println("\nGoing to sleep...");
  esp_deep_sleep_start();  // Sleep for e.g. 30 minutes
}
