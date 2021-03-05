
/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>    //temp, humidity, pressure
#include <Adafruit_MMA8451.h>   //accelerometer
#include <LiquidCrystal_I2C.h>  //LCD screen
#include <ADS1115_WE.h>         //amplifier
#include <Adafruit_DS3502.h>    //potentiometer
#include <DS3231.h>             //real time clock

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme_internal; // I2C
Adafruit_BME280 bme_external; // I2C
Adafruit_MMA8451 mma = Adafruit_MMA8451();

void printValues(Adafruit_BME280 &bme);
void getMMAData();

//for the real time clock
void startTime();
void getTimeInfo();
void GetDateStuff(byte& Year, byte& Month, byte& Day, byte& DoW, 
    byte& Hour, byte& Minute, byte& Second);
RTClib myRTC;
DS3231 Clock;
byte Year;
byte Month;
byte Date;
byte DoW;
byte Hour;
byte Minute;
byte Second;

//for the potentiometer
Adafruit_DS3502 ds3502 = Adafruit_DS3502();
#define WIPER_VALUE_PIN A0
    /* For this code, make the following connections:
      * DS3502 RH to 5V
      * DS3502 RL to GND
      * DS3502 RW to the pin specified by WIPER_VALUE_PIN
    */
void getPotetData();

//for the amplifier

//for the LCD screen


unsigned long delayTime;

void setup() {
    Serial.begin(9600);

    if (! bme_internal.begin(0x76, &Wire)) {
        Serial.println("Could not find a valid internal BME280 sensor, check wiring!");
    }

    if (! bme_external.begin(0x77, &Wire)) {
        Serial.println("Could not find a valid external BME280 sensor, check wiring!");
    }

    if (! mma.begin()) {
      Serial.println("Couldnt start");
    }
    Serial.print("Sealevelhpa,"); Serial.print(SEALEVELPRESSURE_HPA);
    mma.setRange(MMA8451_RANGE_2_G);
    Serial.print(",Range = "); Serial.print(2 << mma.getRange());  
    Serial.println("G");

    //start the real time clock
    startTime();
    //print the header for the rest of the data
    printCsvHeader();
}


void loop() {
    getTimeInfo(); //get the real time clock info
    //delay(delayTime); //not sure if this is needed here or not
    // Only needed in forced mode! In normal mode, you can remove the next line.
    bme_internal.takeForcedMeasurement(); // has no effect in normal mode
    bme_external.takeForcedMeasurement();
    printValues(bme_internal);
    printValues(bme_external);
    delay(delayTime);
    getMMAData();
    getPotetData(); //potentiometer data
    delay(delayTime);
    Serial.println();
}

void printCsvHeader() {
    Serial.print("RTC,");                     //real time clock
    Serial.print("C,hPa,m,%,");               //internal sensors
    Serial.print("C,hPa,m,%,");               //external sensors
    Serial.print(",,,m/s^2 ,m/s^2 ,m/s^2,,"); //accelerometer
    Serial.print("Wiper set to: 63");         //potentiometer - dont forget to change value in code for whatever value is needed
    Serial.println();
    Serial.print("hh:mm:ss,");                                           //real time clock
    Serial.print("internalTemp,internalPres,internalAlt,internalHum,");  //internal sensors
    Serial.print("externalTemp,externalPres,externalAlt,externalHum,");  //external sensors
    Serial.print("x,y,z,xAccel,yAccel,zAccel,orientation,");             //accelerometer
    Serial.print("V,");                                                  //potentiometer
    Serial.println();
}

void startTime(){
  //came from the "DS3231_set" example for the DS3231 library
    if (Serial.available()) {
    GetDateStuff(Year, Month, Date, DoW, Hour, Minute, Second);

    Clock.setClockMode(false);  // set to 24h
    //setClockMode(true); // set to 12h

    Clock.setYear(Year);
    Clock.setMonth(Month);
    Clock.setDate(Date);
    Clock.setDoW(DoW);
    Clock.setHour(Hour);
    Clock.setMinute(Minute);
    Clock.setSecond(Second);
}

void GetDateStuff(byte& Year, byte& Month, byte& Day, byte& DoW, 
    byte& Hour, byte& Minute, byte& Second) {  
  //came from the "DS3231_set" example for the DS3231 library
  // Call this if you notice something coming in on 
  // the serial port. The stuff coming in should be in 
  // the order YYMMDDwHHMMSS, with an 'x' at the end.
  boolean GotString = false;
  char InChar;
  byte Temp1, Temp2;
  char InString[20];

  byte j=0;
  while (!GotString) {
    if (Serial.available()) {
      InChar = Serial.read();
      InString[j] = InChar;
      j += 1;
      if (InChar == 'x') {
        GotString = true;
      }
    }
  }
  Serial.println(InString);
  // Read Year first
  Temp1 = (byte)InString[0] -48;
  Temp2 = (byte)InString[1] -48;
  Year = Temp1*10 + Temp2;
  // now month
  Temp1 = (byte)InString[2] -48;
  Temp2 = (byte)InString[3] -48;
  Month = Temp1*10 + Temp2;
  // now date
  Temp1 = (byte)InString[4] -48;
  Temp2 = (byte)InString[5] -48;
  Day = Temp1*10 + Temp2;
  // now Day of Week
  DoW = (byte)InString[6] - 48;   
  // now Hour
  Temp1 = (byte)InString[7] -48;
  Temp2 = (byte)InString[8] -48;
  Hour = Temp1*10 + Temp2;
  // now Minute
  Temp1 = (byte)InString[9] -48;
  Temp2 = (byte)InString[10] -48;
  Minute = Temp1*10 + Temp2;
  // now Second
  Temp1 = (byte)InString[11] -48;
  Temp2 = (byte)InString[12] -48;
  Second = Temp1*10 + Temp2;
}

void getTimeInfo() {
//came from the "now" example for the DS3231 library
    DateTime now = myRTC.now();
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    //Serial.println();
}

void getMMAData() {
    // Read the 'raw' data in 14-bit counts
    mma.read();
    Serial.print(mma.x); 
    Serial.print(",");
    Serial.print(mma.y); 
    Serial.print(",");
    Serial.print(mma.z);
    Serial.print(",");
  
    /* Get a new sensor event */ 
    sensors_event_t event; 
    mma.getEvent(&event);
  
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print(event.acceleration.x);
    Serial.print(",");
    Serial.print(event.acceleration.y);
    Serial.print(",");
    Serial.print(event.acceleration.z); 
    Serial.print(",");
    
    /* Get the orientation of the sensor */
    uint8_t o = mma.getOrientation();
    
    switch (o) {
      case MMA8451_PL_PUF: 
        Serial.print("Portrait Up Front,");
        break;
      case MMA8451_PL_PUB: 
        Serial.print("Portrait Up Back,");
        break;    
      case MMA8451_PL_PDF: 
        Serial.print("Portrait Down Front,");
        break;
      case MMA8451_PL_PDB: 
        Serial.print("Portrait Down Back,");
        break;
      case MMA8451_PL_LRF: 
        Serial.print("Landscape Right Front,");
        break;
      case MMA8451_PL_LRB: 
        Serial.print("Landscape Right Back,");
        break;
      case MMA8451_PL_LLF: 
        Serial.print("Landscape Left Front,");
        break;
      case MMA8451_PL_LLB: 
        Serial.print("Landscape Left Back,");
        break;
    }
}


void printValues(Adafruit_BME280 &bme) {
    Serial.print(bme.readTemperature());
    Serial.print(",");
    
    Serial.print(bme.readPressure() / 100.0F);
    Serial.print(",");

    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.print(",");

    Serial.print(bme.readHumidity());
    Serial.print(",");
}

void getPotetData(){
  //came from the "ds3502_test" example for the DS3502 library
    Serial.print("Wiper voltage with wiper set to 63: ");
    ds3502.setWiper(63);
    float wiper_value = analogRead(WIPER_VALUE_PIN);
    wiper_value *= 5.0;
    wiper_value /= 1024;
    Serial.print(wiper_value);
    //Serial.println(" V");

    //Serial.println();
    //delay(1000);
}
