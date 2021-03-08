
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


  address links for all sensors
i2c slave               instance              i2c Address
MMA8451 Accelerometer   external              0x1D
ADS1115 ADC             solar cells 1 and 2   0x48
ADS1115 ADC             solar cells 3 and 4   0x49
DS3231 RTC              internal              0x68
DS3502 Pot              solar cells 1 and 2   0x28
DS3502 Pot              solar cells 3 and 4   0x29
LCD Screen              external              0x27
BME280 TEMP             internal              0x76
BME280 TEMP             external              0x77
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
  RTClib myRTC;
  DS3231 Clock;
  byte Year;
  byte Month;
  byte Date;
  byte DoW;
  byte Hour;
  byte Minute;
  byte Second;
  void GetDateStuff(byte& Year, byte& Month, byte& Day, byte& DoW, byte& Hour, byte& Minute, byte& Second);

//for the potentiometers
  Adafruit_DS3502 ds3502 = Adafruit_DS3502();
  #define WIPER_VALUE_PIN1 0x28 //cells 1&2
  #define WIPER_VALUE_PIN2 0x29 //cells 3&4
    /* For this code, make the following connections:
      * DS3502 RH to 5V
      * DS3502 RL to GND
      * DS3502 RW to the pin specified by WIPER_VALUE_PIN
    */
  void getPotentData();

//for the amplifiers 
  ADS1115_WE adc = ADS1115_WE();
  #define acd1_I2C_ADDRESS 0x48 //cells 1&2
  #define acd2_I2C_ADDRESS 0x49 //cells 3&4
  void setADSData();
  void getADSData();

//for the LCD screen
  LiquidCrystal_I2C lcd(0x27,20,4); // set the LCD address to 0x27 for a 16 chars and 2 line display
  void lcdPrint();

  unsigned long delayTime;

//set up for the arduino
void setup() {
// initialize the lcd - came from  
    lcd.init();
    lcd.init();
// Print a message to the LCD.
    lcd.backlight();
    lcd.setCursor(3,0);
    lcd.print("System on!");     
    Serial.begin(9600);

//check for the bme sensors
    if (! bme_internal.begin(0x76, &Wire)) {
        Serial.println("Could not find a valid internal BME280 sensor, check wiring!");
    }
    if (! bme_external.begin(0x77, &Wire)) {
        Serial.println("Could not find a valid external BME280 sensor, check wiring!");
    }
//check for the mma sensor
    if (! mma.begin()) {
      Serial.println("Couldnt start");
    }
    Serial.print("Sealevelhpa,"); Serial.print(SEALEVELPRESSURE_HPA);
    mma.setRange(MMA8451_RANGE_2_G);
    Serial.print(",Range = "); Serial.print(2 << mma.getRange());  
    Serial.println("G");
//check for the amplifiers
  if(!adc.init()){
    Serial.println("ADS1115 not connected!");
  }
  setADSData();

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
    printValues(bme_internal);  //internal sensor data
    printValues(bme_external);  //external sensor data
    delay(delayTime);
    getMMAData();               //accelerometer data
    getPotentData();            //potentiometer data
    delay(delayTime);
    getADSData();               //amplifier values
    delay(delayTime);
    Serial.println();
    lcdPrint();                 //LCD screen print

}

void printCsvHeader() {
    Serial.print("RTC,");                     //real time clock
    Serial.print("C,hPa,m,%,");               //internal sensors
    Serial.print("C,hPa,m,%,");               //external sensors
    Serial.print(",,,m/s^2 ,m/s^2 ,m/s^2,,"); //accelerometer
    Serial.print("Wiper values: 63,63");      //potentiometer - dont forget to change wiper value in code for whatever value is needed
    Serial.print("Channels,for,ADS1,Volts,");      //amplifier1
    Serial.print("Channels,for,ADS2,Volts,");      //amplifier2
    Serial.println();
    Serial.print("hh:mm:ss,");                                           //real time clock
    Serial.print("internalTemp,internalPres,internalAlt,internalHum,");  //internal sensors
    Serial.print("externalTemp,externalPres,externalAlt,externalHum,");  //external sensors
    Serial.print("x,y,z,xAccel,yAccel,zAccel,orientation,");             //accelerometer
    Serial.print("V,V,");                                                //potentiometer
    Serial.print("0,1,2,3");//amplifier
    Serial.print("0,1,2,3");//amplifier
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
}

void GetDateStuff(byte& Year, byte& Month, byte& Day, byte& DoW, byte& Hour, byte& Minute, byte& Second) {  
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

void getPotentData(){
  //came from the "ds3502_test" example for the DS3502 library
    Serial.print("Wiper voltage with wiper set to 63: ");
    ds3502.setWiper(63);
    float wiper_value1 = analogRead(WIPER_VALUE_PIN1);
    wiper_value1 *= 5.0;
    wiper_value1 /= 1024;
    Serial.print(wiper_value1);
    float wiper_value2 = analogRead(WIPER_VALUE_PIN2);
    wiper_value2 *= 5.0;
    wiper_value2 /= 1024;
    Serial.print(wiper_value2);
    //Serial.println(" V");

    //Serial.println();
    //delay(1000);
}

void setADSData(){
//came from the "Continuous" example for the ADS1115_WE
    /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   * 
   * ADS1115_RANGE_6144  ->  +/- 6144 mV
   * ADS1115_RANGE_4096  ->  +/- 4096 mV
   * ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1115_RANGE_1024  ->  +/- 1024 mV
   * ADS1115_RANGE_0512  ->  +/- 512 mV
   * ADS1115_RANGE_0256  ->  +/- 256 mV
   */
  adc.setVoltageRange_mV(ADS1115_RANGE_6144); //comment line/change parameter to change range

  /* Set the inputs to be compared
   *  
   *  ADS1115_COMP_0_1    ->  compares 0 with 1 (default)
   *  ADS1115_COMP_0_3    ->  compares 0 with 3
   *  ADS1115_COMP_1_3    ->  compares 1 with 3
   *  ADS1115_COMP_2_3    ->  compares 2 with 3
   *  ADS1115_COMP_0_GND  ->  compares 0 with GND
   *  ADS1115_COMP_1_GND  ->  compares 1 with GND
   *  ADS1115_COMP_2_GND  ->  compares 2 with GND
   *  ADS1115_COMP_3_GND  ->  compares 3 with GND
   */
  adc.setCompareChannels(ADS1115_COMP_0_GND); //comment line/change parameter to change channel

  /* Set number of conversions after which the alert pin asserts
   * - or you can disable the alert 
   *  
   *  ADS1115_ASSERT_AFTER_1  -> after 1 conversion
   *  ADS1115_ASSERT_AFTER_2  -> after 2 conversions
   *  ADS1115_ASSERT_AFTER_4  -> after 4 conversions
   *  ADS1115_DISABLE_ALERT   -> disable comparator / alert pin (default) 
   */
  //adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1); //uncomment if you want to change the default

  /* Set the conversion rate in SPS (samples per second)
   * Options should be self-explaining: 
   * 
   *  ADS1115_8_SPS 
   *  ADS1115_16_SPS  
   *  ADS1115_32_SPS 
   *  ADS1115_64_SPS  
   *  ADS1115_128_SPS (default)
   *  ADS1115_250_SPS 
   *  ADS1115_475_SPS 
   *  ADS1115_860_SPS 
   */
  // adc.setConvRate(ADS1115_8_SPS); //uncomment if you want to change the default

  /* Set continuous or single shot mode:
   * 
   *  ADS1115_CONTINUOUS  ->  continuous mode
   *  ADS1115_SINGLE     ->  single shot mode (default)
   */
  adc.setMeasureMode(ADS1115_CONTINUOUS); //comment line/change parameter to change mode

   /* Choose maximum limit or maximum and minimum alert limit (window) in Volt - alert pin will 
   *  assert when measured values are beyond the maximum limit or outside the window 
   *  Upper limit first: setAlertLimit_V(MODE, maximum, minimum)
   *  In max limit mode the minimum value is the limit where the alert pin assertion will be  
   *  cleared (if not latched)  
   * 
   *  ADS1115_MAX_LIMIT
   *  ADS1115_WINDOW
   * 
   */
  //adc.setAlertModeAndLimit_V(ADS1115_MAX_LIMIT, 3.0, 1.5); //uncomment if you want to change the default
  
  /* Enable or disable latch. If latch is enabled the alert pin will assert until the
   * conversion register is read (getResult functions). If disabled the alert pin assertion will be
   * cleared with next value within limits. 
   *  
   *  ADS1115_LATCH_DISABLED (default)
   *  ADS1115_LATCH_ENABLED
   */
  //adc.setAlertLatch(ADS1115_LATCH_ENABLED); //uncomment if you want to change the default

  /* Sets the alert pin polarity if active:
   *  
   * ADS1115_ACT_LOW  ->  active low (default)   
   * ADS1115_ACT_HIGH ->  active high
   */
  //adc.setAlertPol(ADS1115_ACT_LOW); //uncomment if you want to change the default
 
  /* With this function the alert pin will assert, when a conversion is ready.
   * In order to deactivate, use the setAlertLimit_V function  
   */
  //adc.setAlertPinToConversionReady(); //uncomment if you want to change the default

  Serial.println("ADS1115 Example Sketch - Continuous Mode");
  Serial.println("All values in volts");
  Serial.println();
}

void getADSData(){
//came from the "Continuous" example for the ADS1115_WE
  float voltage = 0.0;

  voltage = readChannel(ADS1115_COMP_0_GND);
  Serial.print(voltage);

  voltage = readChannel(ADS1115_COMP_1_GND);
  Serial.print(voltage);
  
  voltage = readChannel(ADS1115_COMP_2_GND);
  Serial.print(voltage);

  voltage = readChannel(ADS1115_COMP_3_GND);
  Serial.println(voltage);
}

float readChannel(ADS1115_MUX channel) {
    float voltage = 0.0;
    adc.setCompareChannels(channel);
    voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
    return voltage;
}

void lcdPrint(){
 //came from the "" example for the 
  // when characters arrive over the serial port...
  if (Serial.available()) {
    // wait a bit for the entire message to arrive
    delay(100);
    // clear the screen
    lcd.clear();
    // read all the available characters
    while (Serial.available() > 0) {
      // display each character to the LCD
      lcd.write(Serial.read());
    }
  }
}
