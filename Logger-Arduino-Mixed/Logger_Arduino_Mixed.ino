///
/// @mainpage	Logger-Arduino-Mixed
///
/// @details	Arduino side - acccelerometer and PIR
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		3/22/17 6:05 PM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2017
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Logger_Arduino_Mixed.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		3/22/17 6:05 PM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2017
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#include "libpandora_types.h"
#include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#include "application.h"
#elif defined(ESP8266) // ESP8266 specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE

// Set parameters
// The SparkFun MMA8452 breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
#define SA0 1
#if SA0
#define MMA8452_ADDRESS 0x1D  // SA0 is high, 0x1C if low
#else
#define MMA8452_ADDRESS 0x1C
#endif
// There are some new pin assignments when using the new v9 and v10 boards
#define V910BOARD 1
#if V910BOARD                    // These are the pin assignments for the v9 and v10 boards
#define ALARMPIN 3         // This one will be used for the RTC Alarm in v9 and v10
#define INT2PIN 2         // This is the interrupt pin that registers taps
#define INTNUMBER 0         // so I don't have to use the lookup function
#define PIRPIN 5            // This is a pin which connects to the i2c header - future use
#define I2CPWR 8            // Turns the i2c port on and off
#define RESETPIN 16         // This a modification using a bodge wire
#define TALKPIN 14           // This is the open-drain line for signaling i2c mastery (A0 on the Uno is 14)
#define THE32KPIN 15      // This is a 32k squarewave from the DS3231 (A1 on the Uno is 15)
#else                      // These are the pin assignments for the old v8 board
#define SENSORPIN 2         // Not used now but wired for future use
#define PIRPIN 3         // This is the interrupt pin that registers taps
#define INTNUMBER 1
#define ALARMPIN 5         // This is the pin with the RTC Alarm clock - not used on Arduino side
#define I2CPWR 8            // Turns the i2c port on and off
#define RESETPIN 16         // This a modification using a bodge wire
#define TALKPIN 14           // This is the open-drain line for signaling i2c mastery (A0 on the Uno is 14)
#define THE32KPIN 15      // This is a 32k squarewave from the DS3231 (A1 on the Uno is 15)
#endif

//Time Period Definitions - used for debugging
#define HOURLYPERIOD hour(t)   // Normally hour(t) but can use minute(t) for debugging
#define DAILYPERIOD day(t) // Normally day(t) but can use minute(t) or hour(t) for debugging

//These defines let me change the memory map and configuration without hunting through the whole program
#define VERSIONNUMBER 7             // Increment this number each time the memory map is changed
#define WORDSIZE 8                  // For the Word size
#define PAGESIZE 4096               // Memory size in bytes / word size - 256kb FRAM
// First Word - 8 bytes for setting global values
#define DAILYOFFSET 2               // First word of daily counts
#define HOURLYOFFSET 30             // First word of hourly counts (remember we start counts at 1)
#define DAILYCOUNTNUMBER 28         // used in modulo calculations - sets the # of days stored
#define HOURLYCOUNTNUMBER 4064      // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
#define VERSIONADDR 0x0             // Memory Locations By Name not Number
#define SENSITIVITYADDR 0x1         // For the 1st Word locations
#define DEBOUNCEADDR 0x2            // One byte for debounce (stored in cSec mult by 10 for mSec)
#define MONTHLYREBOOTCOUNT 0x3      // This is where we store the reboots - indication of system health
#define DAILYPOINTERADDR 0x4        // One byte for daily pointer
#define HOURLYPOINTERADDR 0x5       // Two bytes for hourly pointer
#define CONTROLREGISTER 0x7         // This is the control register acted on by both Simblee and Arduino
//Second Word - 8 bytes for storing current counts
#define CURRENTHOURLYCOUNTADDR 0x8  // Current Hourly Count
#define CURRENTDAILYCOUNTADDR 0xA   // Current Daily Count
#define CURRENTCOUNTSTIME 0xC       // Time of last count
//These are the hourly and daily offsets that make up the respective words
#define DAILYDATEOFFSET 1           //Offsets for the value in the daily words
#define DAILYCOUNTOFFSET 2          // Count is a 16-bt value
#define DAILYBATTOFFSET 4           // Where the battery charge is stored
#define HOURLYCOUNTOFFSET 4         // Offsets for the values in the hourly words
#define HOURLYBATTOFFSET 6          // Where the hourly battery charge is stored
// LED Pin Value Variables
#define REDLED 4          // led connected to digital pin 4
#define YELLOWLED 6       // The yellow LED
#define LEDPWR 7          // This pin turns on and off the LEDs
// Finally, here are the variables I want to change often and pull them all together here
#define SOFTWARERELEASENUMBER "1.0.0"
#define PARKCLOSES 19
#define PARKOPENS 7


// Include application, user and local libraries
#include "i2c.h"                // not the wire library, can't use pull-ups
#include <avr/sleep.h>          // For Sleep Code
#include <avr/power.h>          // Power management
#include <avr/wdt.h>            // Watchdog Timer
#include <EEPROM.h>
#include "MAX17043.h"           // Drives the LiPo Fuel Gauge
#include <Wire.h>               //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <TimeLib.h>            //http://www.arduino.cc/playground/Code/Time
#include "DS3232RTC.h"          //http://github.com/JChristensen/DS3232RTC
#include "Adafruit_FRAM_I2C.h"  // Library for FRAM functions
#include "digitalWriteFast.h"   // Try to cut down on overhead for timing sensitive IO - https://github.com/NicksonYap/digitalWriteFast
#include "FRAMcommon.h"         // Where I put all the common FRAM read and write extensions


// Prototypes
MAX17043 batteryMonitor;        // Initialize the Fuel Gauge

// Prototypes for i2c functions
byte readRegister(uint8_t address); // Read an i2c device register
void writeRegister(unsigned char address, unsigned char data); // Writes to an i2c device register
void MMA8452Standby(); // Puts the i2c module into standby mode
void MMA8452Active();  // Bring the MMA8452 back on-line
void initMMA8452(byte fsr, byte dataRate);  // Initialize the MMA8452


// Prototypes for General Functions
void StartStopTest(boolean startTest); // Since the test can be started from the serial menu or the Simblee - created a function
void BlinkForever(); // Ends execution
void enable32Khz(uint8_t enable);  // Need to turn on the 32k square wave for bus moderation
void LogHourlyEvent(); // Log Hourly Event()
void LogDailyEvent(); // Log Daily Event()
void CheckForBump(); // Check for bump
void pinChangeISR();      // Thie is the Interrrupt Service Routine for the pin change interrupt
void SetPinChangeInterrupt(byte Pin);  // Here is where we set the pinchange interrupt
void ClearPinChangeInterrupt(byte Pin);  // Here is where we clear the pinchange interrupt
void sleepNow();  // Puts the Arduino to Sleep
void NonBlockingDelay(int millisDelay);  // Used for a non-blocking delay
int freeRam();  // Debugging code, to check usage of RAM


// Prototypes for Date and Time Functions
void SetTimeDate(); // Sets the RTC date and time
void PrintTimeDate(time_t t); // Prints to the console
void toArduinoTime(time_t unixT); //Converts to Arduino Time for use with the RTC and program


// FRAM and Unix time variables
time_t t;
byte lastHour = 0;  // For recording the startup values
byte lastDate = 0;   // These values make sure we record events if time has lapsed
unsigned int hourlyPersonCount = 0;  // hourly counter
unsigned int dailyPersonCount = 0;   //  daily counter
byte currentHourlyPeriod;    // This is where we will know if the period changed
byte currentDailyPeriod;     // We will keep daily counts as well as period counts
int countTemp = 0;          // Will use this to see if we should display a day or hours counts

// Variables for the control byte
// Control Register  (8 - 7 Reserved, 6 - Simblee Reset, 5-Clear Counts, 4-Simblee Sleep, 3-Start / Stop Test, 2-Set Sensitivity, 1-Set Delay)
byte signalDebounceChange = B00000001;      // These are the bit masks to set and clear the control register bits
byte clearDebounceChange = B11111110;
byte signalSentitivityChange = B00000010;
byte clearSensitivityChange = B11111101;
byte toggleStartStop = B00000100;
byte toggleSimbleeSleep = B00001000;
byte signalClearCounts = B00010000;
byte clearClearCounts = B11101111;
byte signalSimbleeReset = B00100000;        // Simblee will set this flag on disconnect
byte clearSimbleeReset = B11011111;         // The only thing the Arduino will do is clear the Simblee Reset Bit
byte controlRegisterValue;                  // Holds the current control register value
byte oldControlRegisterValue;               // Makes sure we can detect a change in the register value
unsigned long lastCheckedControlRegister;   // When did we last check the control register
int controlRegisterDelay = 1000;            // Ho often will we check the control register


// Accelerometer Variables
const byte accelFullScaleRange = 2;  // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
const byte dataRate = 5;             // output data rate - 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
byte accelInputValue = 1;            // Raw sensitivity input (0-9);
byte accelSensitivity;               // Hex variable for sensitivity
byte accelThreshold = 100;           // accelThreshold value to decide when the detected sound is a knock or not
unsigned int debounce;               // This is a minimum debounce value - additional debounce set using pot or remote terminal

// PIR Sensor Variables
unsigned long warmUpTime = 1000;    // PIR Sensors need 45-60 seconds to warm up
volatile bool PIRInt = false;       // A flag for the PIR Interrupt
boolean countEnable = false;        // Need to count only once for each time the sensor triggers

// Mixed Sensor Flags
boolean accelCount = false;         // Propose counting a person based on accelerometer
boolean pirCount = false;           // Propose counting a person based on PIR sensor

// Battery monitor
float stateOfCharge = 0;            // stores battery charge level value

//Menu and Program Variables
unsigned long lastBump = 0;         // set the time of an event
boolean ledState = LOW;                 // variable used to store the last LED status, to toggle the light
int delaySleep = 1000;               // Wait until going back to sleep so we can enter commands
int menuChoice=0;                   // Menu Selection
boolean refreshMenu = true;         //  Tells whether to write the menu
boolean inTest = false;             // Are we in a test or not
boolean LEDSon = true;              // Are the LEDs on or off
unsigned int LEDSonTime = 30000;    // By default, turn the LEDS on for 30 seconds - remember only awake time counts not real seconds
int numberHourlyDataPoints;         // How many hourly counts are there
int numberDailyDataPoints;          // How many daily counts are there
const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu
byte bootcount = 0;                 // Counts reboots
int bootCountAddr = 0;              // Address for Boot Count Number


// Add setup code
void setup()
{
    Serial.begin(9600);                     // Initialize communications with the terminal
    Serial.println("");                     // Header information
    Serial.print(F("Logger-Arduino-Mixed - release "));
    Serial.println(releaseNumber);
    pinModeFast(REDLED, OUTPUT);            // declare the Red LED Pin as an output
    pinModeFast(YELLOWLED, OUTPUT);         // declare the Yellow LED Pin as as OUTPUT
    pinModeFast(LEDPWR, OUTPUT);            // declare the Power LED pin as as OUTPUT
    digitalWriteFast(LEDPWR, LOW);          // Turn on the power to the LEDs at startup for as long as is set in LEDsonTime
    pinModeFast(I2CPWR, OUTPUT);            // This is for V10 boards which can turn off power to the external i2c header
    digitalWriteFast(I2CPWR, HIGH);         // Turns on the i2c port
    pinModeFast(RESETPIN,INPUT);            // Just to make sure - if set to output, you cant program the SIMBLEE
    pinModeFast(PIRPIN, INPUT);            // Set up the interrupt pins, they're set as active low with an external pull-up
    pinModeFast(INT2PIN, INPUT);            // Set up the interrupt pins, they're set as active low with an external pull-up
    pinModeFast(THE32KPIN,INPUT);           // These are the pins tha are used to negotiate for the i2c bus
    pinModeFast(TALKPIN,INPUT);             // These are the pins tha are used to negotiate for the i2c bus
    
    
    enable32Khz(1); // turns on the 32k squarewave - to moderate access to the i2c bus
    
    int rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
    if (rtn != 0)
    {
        Serial.println(F("I2C bus error. Could not clear"));
        if (rtn == 1) {
            Serial.println(F("SCL clock line held low"));
        } else if (rtn == 2) {
            Serial.println(F("SCL clock line held low by slave clock stretch"));
        } else if (rtn == 3) {
            Serial.println(F("SDA data line held low"));
        }
    }
    else    // Else the bus is clear - can restart Wire
    {
        Wire.begin();
    }
    
    
    TakeTheBus(); // Need th i2c bus for initializations
    if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
        Serial.println(F("Found I2C FRAM"));
    } else {
        Serial.println(F("No I2C FRAM found ... check your connections"));
        BlinkForever();
    }
    GiveUpTheBus(); // Done with i2c initializations Arduino gives up the bus here.
    
    
    if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {  // Check to see if the memory map in the sketch matches the data on the chip
        Serial.print(F("FRAM Version Number: "));
        Serial.println(FRAMread8(VERSIONADDR));
        Serial.read();
        Serial.println(F("Memory/Sketch mismatch! Erase FRAM? (Y/N)"));
        while (!Serial.available());
        switch (Serial.read()) {    // Give option to erase and reset memory
            case 'Y':
                ResetFRAM();
                break;
            case 'y':
                ResetFRAM();
                break;
            default:
                Serial.println(F("Cannot proceed"));
                BlinkForever();
        }
    }
    
    // Initialize the rest of the i2c devices
    TakeTheBus();
    batteryMonitor.reset();               // Initialize the battery monitor
    batteryMonitor.quickStart();
    setSyncProvider(RTC.get);              // Set up the clock as we will control it and the alarms here
    Serial.println(F("RTC Sync"));
    if (timeStatus() != timeSet) {
        Serial.println(F(" time sync fail!"));
        BlinkForever();
    }
    // We need to set an Alarm or Two in order to ensure that the Simblee is put to sleep at night
    RTC.squareWave(SQWAVE_NONE);            //Disable the default square wave of the SQW pin.
    RTC.alarm(ALARM_1);                     // This will clear the Alarm flags
    RTC.alarm(ALARM_2);                     // This will clear the Alarm flags
    RTC.setAlarm(ALM1_MATCH_HOURS,00,00,PARKCLOSES,0); // Set the evening Alarm
    RTC.setAlarm(ALM2_MATCH_HOURS,00,00,PARKOPENS,0); // Set the morning Alarm
    //RTC.setAlarm(ALM1_MATCH_MINUTES,00,45,00,0); // Start Alarm - for debugging
    //RTC.setAlarm(ALM2_MATCH_MINUTES,00,47,00,0); // Wake Alarm - for debugging
    RTC.alarmInterrupt(ALARM_2, true);      // Connect the Interrupt to the Alarms (or not)
    RTC.alarmInterrupt(ALARM_1, true);
    GiveUpTheBus();
    
    // Import the accelSensitivity and Debounce values from memory
    Serial.print(F("Sensitivity set to: "));
    accelSensitivity = FRAMread8(10-SENSITIVITYADDR);
    Serial.println(accelSensitivity);
    Serial.print(F("Debounce set to: "));
    debounce = FRAMread8(DEBOUNCEADDR)*10;     // We mulitply by ten since debounce is stored in 100ths of a second
    if (debounce > delaySleep) delaySleep = debounce;       // delaySleep must be bigger than debounce afterall
    Serial.println(debounce);
    
    FRAMwrite8(CONTROLREGISTER, toggleStartStop);       // Reset the control register and start the test
    
    TakeTheBus();  // Need to initialize the accelerometer
    // Read the WHO_AM_I register of the Accelerometer, this is a good test of communication
    byte c = readRegister(0x0D);  // Read WHO_AM_I register
    if (c == 0x2A) // WHO_AM_I should always be 0x2A
    {
        initMMA8452(accelFullScaleRange, dataRate);  // init the accelerometer if communication is OK
        Serial.println(F("MMA8452Q is online..."));
    }
    else
    {
        Serial.print(F("Could not connect to MMA8452Q: 0x"));
        Serial.println(c, HEX);
        BlinkForever() ; // Loop forever if communication doesn't happen
    }
    GiveUpTheBus(); // Done!
    
    Serial.print(F("Monthly reboot count is "));
    bootCountAddr = EEPROM.read(0);             // Here is where we will track reboots by month - offset stored in 0 byte
    bootcount = EEPROM.read(bootCountAddr);     // Use the offset to get to this month's boot count
    bootcount++;                                // Increment the boot count
    Serial.print(bootcount);
    EEPROM.write(bootCountAddr, bootcount);     // Write it back into the correct spot
    FRAMwrite8(MONTHLYREBOOTCOUNT, bootcount); // Store in FRAM for access by Simblee in user interface
    Serial.print(F(" with a monthly offset of: "));
    TakeTheBus();
    t = RTC.get();
    GiveUpTheBus();
    bootCountAddr = month(t);                   // Boot counts are offset by month to reduce burn - in risk
    EEPROM.update(0, bootCountAddr);            // Will update the month if it has changed but only at reboot
    Serial.println(EEPROM.read(0));             // Print so we can see if code is working
    
    
    Serial.print(F("Free memory: "));
    Serial.println(freeRam());
}

// Add loop code
void loop()
{
    if (refreshMenu) {
        refreshMenu = 0;
        Serial.println(F("Remote Trail Counter Program Menu"));
        Serial.println(F("0 - Display Menu"));
        Serial.println(F("1 - Display status"));
        Serial.println(F("2 - Set the clock"));
        Serial.println(F("3 - Change the sensitivitiy"));
        Serial.println(F("4 - Change the debounce"));
        Serial.println(F("5 - Reset the counter"));
        Serial.println(F("6 - Reset the memory"));
        Serial.println(F("7 - Start / stop counting"));
        Serial.println(F("8 - Dump hourly counts"));
        Serial.println(F("9 - Last 14 day's counts"));
        NonBlockingDelay(100);
    }
    if (Serial.available() >> 0) {      // Only enter if there is serial data in the buffer
        switch (Serial.read()) {          // Read the buffer
            case '0':
                refreshMenu = 1;
                break;
            case '1':   // Display Current Status Information
                Serial.print(F("Current Time:"));
                TakeTheBus();
                t = RTC.get();
                GiveUpTheBus();
                PrintTimeDate(t);  // Give and take the bus are in this function as it gets the current time
                TakeTheBus();
                stateOfCharge = batteryMonitor.getSoC();
                GiveUpTheBus();
                Serial.print(F("State of charge: "));
                Serial.print(stateOfCharge);
                Serial.println(F("%"));
                Serial.print(F("Sensitivity set to: "));
                Serial.println(10-FRAMread8(SENSITIVITYADDR));
                Serial.print(F("Debounce set to: "));
                Serial.println(FRAMread8(DEBOUNCEADDR)*10);        // We mulitply by 10 as debounce is stored in 100ths
                Serial.print(F("Hourly count: "));
                Serial.println(FRAMread16(CURRENTHOURLYCOUNTADDR));
                Serial.print(F("Daily count: "));
                Serial.println(FRAMread16(CURRENTDAILYCOUNTADDR));
                Serial.print(F("Free memory: "));
                Serial.println(freeRam());
                Serial.print(F("Reboots this month: "));
                Serial.println(bootcount);
                break;
            case '2':     // Set the clock
                SetTimeDate();
                PrintTimeDate(t);
                Serial.println(F("Date and Time Set"));
                break;
            case '3':  // Change the sensitivity
                Serial.println(F("Enter 0 (least) to 10 (most)"));
                while (Serial.available() == 0) {  // Look for char in serial queue and process if found
                    continue;
                }
                accelInputValue = (Serial.parseInt());
                Serial.print(F("accelSensitivity set to: "));
                Serial.println(accelInputValue);
                accelSensitivity = 10-accelInputValue;
                FRAMwrite8(SENSITIVITYADDR, accelSensitivity);
                TakeTheBus();
                initMMA8452(accelFullScaleRange, dataRate);  // init the accelerometer if communication is OK
                GiveUpTheBus();
                Serial.print(F("Accelsensitivity = "));
                Serial.print(accelSensitivity);
                Serial.println(F(" MMA8452Q is online..."));
                break;
            case '4':  // Change the debounce value
                Serial.println(F("Enter debounce in mSec"));
                while (Serial.available() == 0) {  // Look for char in serial queue and process if found
                    continue;
                }
                debounce = Serial.parseInt();
                if (debounce > delaySleep) delaySleep = debounce;       // delaySleep must be bigger than debounce afterall
                Serial.print(F("Debounce set to: "));
                Serial.println(debounce);
                FRAMwrite8(DEBOUNCEADDR, debounce/10);     // Remember we store debounce in cSec
                break;
            case '5':  // Reset the current counters
                Serial.println(F("Counter Reset!"));
                FRAMwrite16(CURRENTDAILYCOUNTADDR, 0);   // Reset Daily Count in memory
                FRAMwrite16(CURRENTHOURLYCOUNTADDR, 0);  // Reset Hourly Count in memory
                hourlyPersonCount = 0;
                dailyPersonCount = 0;
                Serial.println(F("Resetting Counters and Simblee"));
                pinMode(RESETPIN, OUTPUT);
                digitalWrite(RESETPIN, LOW);
                NonBlockingDelay(100);
                digitalWrite(RESETPIN, HIGH);
                pinMode(RESETPIN, INPUT);
                break;
            case '6': // Reset FRAM Memory
                ResetFRAM();
                break;
            case '7':  // Start or stop the test
                if (inTest == 0) {
                    FRAMwrite8(CONTROLREGISTER, toggleStartStop | controlRegisterValue);    // Toggle the start stop bit high
                    StartStopTest(1);
                }
                else {
                    FRAMwrite8(CONTROLREGISTER, toggleStartStop ^ controlRegisterValue);    // Toggle the start stop bit low
                    StartStopTest(0);
                    refreshMenu = 1;
                }
                break;
            case '8':   // Dump the hourly data to the monitor
                numberHourlyDataPoints = FRAMread16(HOURLYPOINTERADDR); // Put this here to reduce FRAM reads
                Serial.print("Retrieving ");
                Serial.print(HOURLYCOUNTNUMBER);
                Serial.println(" hourly counts");
                Serial.println(F("Hour Ending -   Count  - Battery %"));
                for (int i=0; i < HOURLYCOUNTNUMBER; i++) { // Will walk through the hourly count memory spots - remember pointer is already incremented
                    unsigned int address = (HOURLYOFFSET + (numberHourlyDataPoints + i) % HOURLYCOUNTNUMBER)*WORDSIZE;
                    countTemp = FRAMread16(address+HOURLYCOUNTOFFSET);
                    if (countTemp > 0) {
                        time_t unixTime = FRAMread32(address);
                        toArduinoTime(unixTime);
                        Serial.print(F(" - "));
                        Serial.print(countTemp);
                        Serial.print(F("  -  "));
                        Serial.print(FRAMread8(address+HOURLYBATTOFFSET));
                        Serial.println(F("%"));
                    }
                }
                Serial.println(F("Done"));
                break;
            case '9':  // Download all the daily counts
                numberDailyDataPoints = FRAMread8(DAILYPOINTERADDR);        // Put this here to reduce FRAM reads
                Serial.println(F("Date - Count - Battery %"));
                for (int i=0; i < DAILYCOUNTNUMBER; i++) {                  // Will walk through the 30 daily count memory spots - remember pointer is already incremented
                    int address = (DAILYOFFSET + (numberDailyDataPoints + i) % DAILYCOUNTNUMBER)*WORDSIZE;      // Here to improve readabiliy - with Wrapping
                    countTemp = FRAMread16(address+DAILYCOUNTOFFSET);       // This, again, reduces FRAM reads
                    if (countTemp > 0) {                                    // Since we will step through all 30 - don't print empty results
                        Serial.print(FRAMread8(address));
                        Serial.print(F("/"));
                        Serial.print(FRAMread8(address+DAILYDATEOFFSET));
                        Serial.print(F(" - "));
                        Serial.print(countTemp);
                        Serial.print(F("  -  "));
                        Serial.print(FRAMread8(address+DAILYBATTOFFSET));
                        Serial.println(F("%"));
                    }
                }
                Serial.println(F("Done"));
                break;
            default:
                Serial.println(F("Invalid choice - try again"));
        }
        Serial.read();  // Clear the serial buffer
    }
    if (inTest == 1) {
        CheckForBump();
        if (millis() >= lastBump + delaySleep) {
        sleepNow();     // sleep function called here
        }
    }
    if (millis() >= lastCheckedControlRegister + controlRegisterDelay) {
        controlRegisterValue = FRAMread8(CONTROLREGISTER);
        oldControlRegisterValue = controlRegisterValue;
        lastCheckedControlRegister = millis();
        if ((controlRegisterValue & toggleStartStop) >> 2 && !inTest)
        {
            StartStopTest(1);  // If the control says start but we are stopped
        }
        else if (!((controlRegisterValue & toggleStartStop) >> 2) && inTest)
        {
            StartStopTest(0); // If the control bit says stop but we have started
        }
        else if (controlRegisterValue & signalDebounceChange)   // If we changed the debounce value on the Simblee side
        {
            debounce = FRAMread8(DEBOUNCEADDR)*10;     // We multiply by 10 since debounce is stored in cSec
            if (debounce > delaySleep) delaySleep = debounce;       // delaySleep must be bigger than debounce afterall
            Serial.print(F("Updated debounce value to:"));
            Serial.println(debounce);
            controlRegisterValue &= clearDebounceChange;
            FRAMwrite8(CONTROLREGISTER, controlRegisterValue);
        }
        else if (controlRegisterValue & signalSentitivityChange)   // If we changed the debounce value on the Simblee side
        {
            accelSensitivity = FRAMread8(SENSITIVITYADDR);
            TakeTheBus();
            initMMA8452(accelFullScaleRange, dataRate);  // init the accelerometer if communication is OK
            GiveUpTheBus();
            Serial.print(F("MMA8452Q is online..."));
            Serial.print(F("Updated sensitivity value to:"));
            Serial.println(10-accelSensitivity);
            controlRegisterValue &= clearSensitivityChange;
            FRAMwrite8(CONTROLREGISTER, controlRegisterValue);
        }
        else if (controlRegisterValue & signalClearCounts)
        {
            TakeTheBus();
            t = RTC.get();
            GiveUpTheBus();
            hourlyPersonCount = 0;
            dailyPersonCount = 0;
            FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
            FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
            FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
            Serial.println(F("Current Counts Cleared"));
            controlRegisterValue &= clearClearCounts;
            FRAMwrite8(CONTROLREGISTER, controlRegisterValue);
        }
        
        else if (controlRegisterValue & signalSimbleeReset)  // If the reset flag is set
        {
            Serial.println("Resetting the Simblee");
            if (!(controlRegisterValue & toggleSimbleeSleep)) // Only reset if the Simblee is awake
            {
                pinMode(RESETPIN, OUTPUT);
                digitalWrite(RESETPIN, LOW);
                NonBlockingDelay(100);
                digitalWrite(RESETPIN, HIGH);
                pinMode(RESETPIN, INPUT);
            }
            FRAMwrite8(CONTROLREGISTER, controlRegisterValue & clearSimbleeReset);  // Reset the Simblee Sleep flag
        }
        
        else if (LEDSon && millis() >= LEDSonTime)
        {
            digitalWrite(LEDPWR,HIGH);
            LEDSon = false; // This keeps us from entering this conditional once we have turned off the lights
            Serial.println(F("Turn off the LEDs"));
        }
    }
}


void CheckForBump() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the Arduino
{
    if (digitalReadFast(INT2PIN)==0)    // If int2 goes LOW, either p/l has changed or there's been a single/double tap
    {
        TakeTheBus();
        byte source = readRegister(0x0C);  // Read the interrupt source reg.
        readRegister(0x22);  // Reads the PULSE_SRC register to reset it
        GiveUpTheBus();
        if ((source & 0x08)==0x08 && millis() >= lastBump + debounce)  // We are only interested in the TAP register and ignore debounced taps
        {
            accelCount = true;      // Nominate this event for a count
            lastBump = millis();    // Reset last bump timer
            TakeTheBus();
                t = RTC.get();
            GiveUpTheBus();
            if (t == 0) {
                Serial.println(F("t=0 throwing it out"));
                return;     // Error in reading the real time clock - very rare in testing so will simply throw out this count
            }
        }
    }
    if(digitalRead(PIRPIN) && countEnable)
    {
        lastBump = millis();    // Reset last bump timer
        countEnable = false;    // Make sure we only count once for each trigger
        ledState = !ledState;
        digitalWrite(REDLED,ledState);
        PIRInt = false; // Reset the flag
        TakeTheBus();
            t = RTC.get();
        GiveUpTheBus();
        if (t == 0) return;     // This means there was an error in reading the real time clock - very rare in testing so throws out this count
        pirCount = true;
    }
    else if (!digitalRead(PIRPIN)) countEnable = true;      // Reset the count enable trigger
    if (accelCount | pirCount)
    {
        // Scenarios we need to think through:
        if (accelCount && !pirCount) Serial.print(F("Biker - "));           // Biker - likely accelCount and !pirCount - covered
        if (pirCount && accelCount) Serial.print(F("Hiker - "));        // Walker who steps on accel sensor - both triggered - covered
        if (pirCount && !accelCount) Serial.print(F("Hiker - "));        // Walker who does not step on accel sensor - PIR sensor only - covered

        pirCount = false;
        accelCount = false;
        if (HOURLYPERIOD != currentHourlyPeriod) {
            LogHourlyEvent();
        }
        if (DAILYPERIOD != currentDailyPeriod) {
            LogDailyEvent();
        }
        hourlyPersonCount++;                    // Increment the PersonCount
        FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
        dailyPersonCount++;                    // Increment the PersonCount
        FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
        FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
        Serial.print(F("Hourly: "));
        Serial.print(hourlyPersonCount);
        Serial.print(F(" Daily: "));
        Serial.print(dailyPersonCount);
        Serial.print(F(" Reboots: "));
        Serial.print(bootcount);
        Serial.print(F("  Time: "));
        PrintTimeDate(t);
        ledState = !ledState;              // toggle the status of the LEDPIN:
        digitalWrite(REDLED, ledState);    // update the LED pin itself
    }
}


void StartStopTest(boolean startTest)  // Since the test can be started from the serial menu or the Simblee - created a function
{
    tmElements_t tm;
    if (startTest) {
        inTest = true;
        TakeTheBus();
        t = RTC.get();                    // Gets the current time
        GiveUpTheBus();
        currentHourlyPeriod = HOURLYPERIOD;   // Sets the hour period for when the count starts (see #defines)
        currentDailyPeriod = DAILYPERIOD;     // And the day  (see #defines)
        // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
        time_t unixTime = FRAMread32(CURRENTCOUNTSTIME);
        breakTime(unixTime, tm);
        lastHour = tm.Hour;
        lastDate = tm.Day;
        dailyPersonCount = FRAMread16(CURRENTDAILYCOUNTADDR);  // Load Daily Count from memory
        hourlyPersonCount = FRAMread16(CURRENTHOURLYCOUNTADDR);  // Load Hourly Count from memory
        if (currentDailyPeriod != lastDate) {
            LogHourlyEvent();
            LogDailyEvent();
        }
        else if (currentHourlyPeriod != lastHour) {
            LogHourlyEvent();
        }
        TakeTheBus();
        readRegister(0x22);     // Reads the PULSE_SRC register to reset it
        GiveUpTheBus();
        Serial.println(F("Test Started"));
    }
    else {
        inTest = false;
        TakeTheBus();
        readRegister(0x22);  // Reads the PULSE_SRC register to reset it
        t = RTC.get();
        GiveUpTheBus();
        FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
        FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
        FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
        hourlyPersonCount = 0;        // Reset Person Count
        dailyPersonCount = 0;         // Reset Person Count
        Serial.println(F("Test Stopped"));
    }
}

void LogHourlyEvent() // Log Hourly Event()
{
    tmElements_t timeElement;       // We will need to break down the current time
    time_t LogTime = FRAMread32(CURRENTCOUNTSTIME);     // This is the last event recorded - this sets the hourly period
    breakTime(LogTime, timeElement);                    // Break the time into its pieces
    unsigned int pointer = (HOURLYOFFSET + FRAMread16(HOURLYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
    LogTime -= (60*timeElement.Minute + timeElement.Second); // So, we need to subtract the minutes and seconds needed to take to the top of the hour
    FRAMwrite32(pointer, LogTime);   // Write to FRAM - this is the end of the period
    FRAMwrite16(pointer+HOURLYCOUNTOFFSET,hourlyPersonCount);
    TakeTheBus();
    stateOfCharge = batteryMonitor.getSoC();
    GiveUpTheBus();
    FRAMwrite8(pointer+HOURLYBATTOFFSET,stateOfCharge);
    unsigned int newHourlyPointerAddr = (FRAMread16(HOURLYPOINTERADDR)+1) % HOURLYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
    FRAMwrite16(HOURLYPOINTERADDR,newHourlyPointerAddr);
    hourlyPersonCount = 0;                    // Reset and increment the Person Count in the new period
    currentHourlyPeriod = HOURLYPERIOD;  // Change the time period
    Serial.println(F("Hourly Event Logged"));
}


void LogDailyEvent() // Log Daily Event()
{
    tmElements_t timeElement;
    time_t LogTime = FRAMread32(CURRENTCOUNTSTIME);// This is the last event recorded - this sets the daily period
    breakTime(LogTime, timeElement);
    int pointer = (DAILYOFFSET + FRAMread8(DAILYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
    FRAMwrite8(pointer,timeElement.Month); // The month of the last count
    FRAMwrite8(pointer+DAILYDATEOFFSET,timeElement.Day);  // Write to FRAM - this is the end of the period  - should be the day
    FRAMwrite16(pointer+DAILYCOUNTOFFSET,dailyPersonCount);
    TakeTheBus();
    stateOfCharge = batteryMonitor.getSoC();
    GiveUpTheBus();
    FRAMwrite8(pointer+DAILYBATTOFFSET,stateOfCharge);
    byte newDailyPointerAddr = (FRAMread8(DAILYPOINTERADDR)+1) % DAILYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
    FRAMwrite8(DAILYPOINTERADDR,newDailyPointerAddr);
    dailyPersonCount = 0;    // Reset and increment the Person Count in the new period
    currentDailyPeriod = DAILYPERIOD;  // Change the time period
    Serial.println(F("Logged a Daily Event"));
}


void SetTimeDate()  // Function to set the date and time from the terminal window
{
    tmElements_t tm;
    Serial.println(F("Enter Seconds (0-59): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Second = Serial.parseInt();
    Serial.println(F("Enter Minutes (0-59): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Minute = Serial.parseInt();
    Serial.println(F("Enter Hours (0-23): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Hour= Serial.parseInt();
    Serial.println(F("Enter Day of the Month (1-31): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Day = Serial.parseInt();
    Serial.println(F("Enter the Month (1-12): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Month = Serial.parseInt();
    Serial.println(F("Enter the Year (0-99): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Year = CalendarYrToTm(Serial.parseInt());
    t= makeTime(tm);
    TakeTheBus();
    RTC.set(t);             //use the time_t value to ensure correct weekday is set
    setTime(t);
    GiveUpTheBus();
}

void PrintTimeDate(time_t t)  // Prints time and date to the console
{
    Serial.print(year(t), DEC);
    Serial.print('/');
    Serial.print(month(t), DEC);
    Serial.print('/');
    Serial.print(day(t), DEC);
    Serial.print(F(" "));
    Serial.print(hour(t), DEC);
    Serial.print(':');
    if (minute(t) < 10) Serial.print(F("0"));
    Serial.print(minute(t), DEC);
    Serial.print(':');
    if (second(t) < 10) Serial.print(F("0"));
    Serial.print(second(t), DEC);
    Serial.println();
}

void initMMA8452(byte fsr, byte dataRate)   // Initialize the MMA8452 registers
{
    // See the many application notes for more info on setting all of these registers:
    // http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
    // Feel free to modify any values, these are settings that work well for me.
    MMA8452Standby();  // Must be in standby to change registers
    
    // Set up the full scale range to 2, 4, or 8g.
    if ((fsr==2)||(fsr==4)||(fsr==8))
        writeRegister(0x0E, fsr >> 2);
    else
        writeRegister(0x0E, 0);
    
    // Setup the 3 data rate bits, from 0 to 7
    writeRegister(0x2A, readRegister(0x2A) & ~(0x38));
    if (dataRate <= 7)
        writeRegister(0x2A, readRegister(0x2A) | (dataRate << 3));
    
    /* Set up single and double tap - 5 steps:
     1. Set up single and/or double tap detection on each axis individually.
     2. Set the accelThreshold - minimum required acceleration to cause a tap.
     3. Set the time limit - the maximum time that a tap can be above the accelThreshold
     4. Set the pulse latency - the minimum required time between one pulse and the next
     5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
     for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
    //writeRegister(0x21, 0x7F);  // 1. enable single/double taps on all axes
    writeRegister(0x21, 0x55);  // 1. single taps only on all axes
    // writeRegister(0x21, 0x6A);  // 1. double taps only on all axes
    writeRegister(0x23, accelSensitivity);  // 2. x thresh from 0 to 127, multiply the value by 0.0625g/LSB to get the accelThreshold
    writeRegister(0x24, accelSensitivity);  // 2. y thresh from 0 to 127, multiply the value by 0.0625g/LSB to get the accelThreshold
    writeRegister(0x25, accelSensitivity);  // 2. z thresh from 0 to 127, multiply the value by 0.0625g/LSB to get the accelThreshold
    writeRegister(0x26, 0xFF);  // 3. Max time limit at 100Hz odr, this is very dependent on data rate, see the app note
    writeRegister(0x27, 0x64);  // 4. 1000ms (at 100Hz odr) between taps min, this also depends on the data rate
    writeRegister(0x28, 0xFF);  // 5. 318ms (max value) between taps max
    
    // Set up interrupt 1 and 2
    writeRegister(0x2C, 0x02);  // Active high, push-pull interrupts
    writeRegister(0x2D, 0x19);  // DRDY, P/L and tap ints enabled
    writeRegister(0x2E, 0x01);  // DRDY on INT1, P/L and taps on INT2
    
    MMA8452Active();  // Set to active to start reading
}


void MMA8452Standby()   // Sets the MMA8452 to standby mode while we make register changes
{
    byte c = readRegister(0x2A);
    writeRegister(0x2A, c & ~(0x01));
}


void MMA8452Active()    // Sets the MMA8452 to active mode - once changes made
{
    byte c = readRegister(0x2A);
    writeRegister(0x2A, c | 0x01);
}


byte readRegister(uint8_t address)  // Read a single byte from address and return it as a byte
{
    byte data;
    
    i2cSendStart();
    i2cWaitForComplete();
    
    i2cSendByte((MMA8452_ADDRESS<<1)); // Write 0xB4
    i2cWaitForComplete();
    
    i2cSendByte(address);	// Write register address
    i2cWaitForComplete();
    
    i2cSendStart();
    
    i2cSendByte((MMA8452_ADDRESS<<1)|0x01); // Write 0xB5
    i2cWaitForComplete();
    i2cReceiveByte(TRUE);
    i2cWaitForComplete();
    
    data = i2cGetReceivedByte();	// Get MSB result
    i2cWaitForComplete();
    i2cSendStop();
    
    cbi(TWCR, TWEN);	// Disable TWI
    sbi(TWCR, TWEN);	// Enable TWI
    
    return data;
}


void writeRegister(unsigned char address, unsigned char data)   // Writes a single byte (data) into address
{
    i2cSendStart();
    i2cWaitForComplete();
    
    i2cSendByte((MMA8452_ADDRESS<<1)); // Write 0xB4
    i2cWaitForComplete();
    
    i2cSendByte(address);	// Write register address
    i2cWaitForComplete();
    
    i2cSendByte(data);
    i2cWaitForComplete();
    
    i2cSendStop();
}

void pinChangeISR()        // Sensor Interrupt Handler
{
    // execute code here after wake-up before returning to the loop() function
    // timers and code using timers (serial.print and more...) will not work here.
    // we don't really need to execute any special functions here, since we
    // just want the thing to wake up
    sleep_disable();         // first thing after waking from sleep is to disable sleep...
    detachInterrupt(INTNUMBER);      // disables Accelerometer interrupt
    ClearPinChangeInterrupt(PIRPIN);    // Disables PIR interrupt
}

void SetPinChangeInterrupt(byte Pin)  // Here is where we set the pinchange interrupt
{
    *digitalPinToPCMSK(Pin) |= bit (digitalPinToPCMSKbit(Pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(Pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(Pin)); // enable interrupt for the group
}

void ClearPinChangeInterrupt(byte Pin)  // Here is where we clear the pinchange interrupt
{
    *digitalPinToPCMSK(Pin) &= bit (digitalPinToPCMSKbit(Pin));  // disable pin
    PCIFR  != bit (digitalPinToPCICRbit(Pin)); // clear any outstanding interrupt
    PCICR  &= bit (digitalPinToPCICRbit(Pin)); // disnable interrupt for the group
}

ISR (PCINT2_vect)   // interrupt service routine in sleep mode for PIR PinChange Interrupt (D0-D7)
{
    // execute code here after wake-up before returning to the loop() function
    sleep_disable ();           // first thing after waking from sleep:
    detachInterrupt(INTNUMBER);      // disables Accelerometer interrupt
    ClearPinChangeInterrupt(PIRPIN);    // Disables PIR interrupt
}


void sleepNow()         // here we put the arduino to sleep
{
    // Here is a great tutorial on interrupts and sleep: http://www.gammon.com.au/interrupts
    Serial.print(F("Entering Sleep mode..."));
    Serial.flush();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
    noInterrupts ();          // make sure we don't get interrupted before we sleep
    sleep_enable();          // enables the sleep bit in the mcucr register
    attachInterrupt(INTNUMBER,pinChangeISR, LOW); // use interrupt and run function
    SetPinChangeInterrupt(PIRPIN);      // Attach the PinChange Interrupt
    interrupts ();           // interrupts allowed now, next instruction WILL be executed
    sleep_cpu();            // here the device is actually put to sleep!!
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
    delay(10);               // This small delay gives the i2c bus time to reinitialize
    Serial.println("Waking up");
}

void toArduinoTime(time_t unixT) // Puts time in format for reporting
{
    tmElements_t timeElement;
    breakTime(unixT, timeElement);
    Serial.print(timeElement.Month);
    Serial.print(F("/"));
    Serial.print(timeElement.Day);
    Serial.print(F("/"));
    Serial.print(1970+timeElement.Year);
    Serial.print(F(" "));
    Serial.print(timeElement.Hour);
    Serial.print(F(":"));
    if(timeElement.Minute < 10) Serial.print(F("0"));
    Serial.print(timeElement.Minute);
    Serial.print(F(":"));
    if(timeElement.Second < 10) Serial.print(F("0"));
    Serial.print(timeElement.Second);
}


void BlinkForever() // When something goes badly wrong...
{
    Serial.println(F("Error - Reboot"));
    while(1) {
        digitalWriteFast(REDLED,HIGH);
        delay(200);
        digitalWriteFast(REDLED,LOW);
        delay(200);
    }
}

void enable32Khz(uint8_t enable)  // Need to turn on the 32k square wave for bus moderation - could set the Talk line here
{
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.endTransmission();
    
    // status register
    Wire.requestFrom(0x68, 1);
    
    uint8_t sreg = Wire.read();
    
    sreg &= ~0b00001000; // Set to 0
    if (enable == true)
        sreg |=  0b00001000; // Enable if required.
    
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.write(sreg);
    Wire.endTransmission();
}

int freeRam ()  // Debugging code, to check usage of RAM
{
    // Example Call: Serial.println(freeRam());
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

