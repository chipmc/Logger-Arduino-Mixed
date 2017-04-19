
  Logger-Arduino-Mixed
  Project
  ----------------------------------
  Developed with embedXcode

  Project Logger-Arduino-Mixed
  Created by Charles McClelland on 3/22/17
  Copyright © 2017 Charles McClelland
  Licence GNU General Public Licence

/*
This is a mixed sensor implementation with both PIR and Accelerometer

Acknowledgements - I have benefitted greatly from the following contributions.
- MMA8452Q Example Code by: Jim Lindblom SparkFun Electronics November 17, 2011
- MAX1704 Library and code from Maxim
- Triembed.org - great group who answered my questions and showed me the path to this version

License - BSD Release 3

Hardware setup:
- Sparkfun Arduino Pro Mini - 3.3V / 8Mhz
- MMA8452 Breakout
- SDA - Pin A4 <- Pull ups are on the breakout board
- SCL - Pin A5 <- Ditto
- INT2 - D3 - Accelerometer interrupt
- INT1 - D2  - PIR Sensor Interrupt
- PIR Sensor for non-bikers
- TI FRAM Chip on i2c Bus
- DS3231 RTC Module on the i2c bus
- Indicator LED on pin 4
- accelSensitivity and debounce are set via terminal and stored in FRAM
- Inverter used to invert the interrupt from the accelerometer and send to INT2PIN


EEPROM Memory Map
0   Monthly Offset (value 1-12)
1-12 Monthly reboot counts for Jan (1) through December (12)


FRAM Memory Map v9
Memory Map - 256kb or 32kB divided into 4096 words - the  first one is reserved
Byte     Value
The first word is for system data
0        Memory Map Version (this program expects what is #defined in VERSIONNUMBER)
1        accelSensitivity
2        Delay (8-bit)  Delay is stored in 100ths of a second and multiplied by 10 for mSec in the program
3        Monthly Reboot Count - System Health
4        Daily Reading Pointer
5-6      Current Hourly Reading Pointer (16-bit number)
7        Control Register  (8 - 5 Reserved, 4- LEDs, 3-Start / Stop Test, 2-Warm Up, 1-LEDs)
The second and third words are for storing the current data
8-11     EPOCH Time when last counts recorded (32-bits)
12-13    Current Hourly Hiker Count (16-bits)
14-15    Current Hourly Biker Count (16-bits)
16-17    Current Daily Hiker Count (16-bits)
18-19    Current Daily Biker Count (16-bits)
20-23    Reserved
Words 4-30 are 28 days worth of daily counts - if this changes - need to change #offsets and DAILYCOUNTNUMBER
0        Month
1        Day
2-3      Daily Pedestrian Count
4-5      Daily Biker Count
6        Daily Battery Level
7        Reserved
Words from 31 to the end of the memory store hourly data - No battery level stored for hourly
0-3      EPOCH Time when hourly counts recorded (32-bits)
4-5      Hourly Hiker Count (16-bits)
6-7      Hourly Biker Count (16-bits)

The park is open for an average of 12 hours per day so about 340 days of hourly data on a 256k chip

Note: The MMA8452 is an I2C sensor, however this code does
not make use of the Arduino Wire library. Because the sensor
is not 5V tolerant, we can't use the internal pull-ups used
by the Wire library. Instead use the included i2c.h, defs.h and types.h files.
*/


  References
  ----------------------------------



  embedXcode
  embedXcode+
  ----------------------------------
  Embedded Computing on Xcode
  Copyright © Rei VILO, 2010-2017
  All rights reserved
  http://embedXcode.weebly.com

