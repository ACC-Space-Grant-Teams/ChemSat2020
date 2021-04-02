/***************************************************************************
Code for RTC, 2 BMEs, MMA, 4 potentiometers, ADC
Jillian Frimml
3/18/2021

Print out the H,M,S and temp of RTC,
Designed to read 2 BMEs, one for internal, one for external
Print out of the BMEs 
Print out of the MMA information


address links for all sensors
i2c slave               instance              i2c Address
MMA8451 Accelerometer   internal              0x1D
ADS1115 ADC             solar cells 1 and 2   0x48
DS3231 RTC              internal              0x68
DS3502 Pot              solar cells 1, pt2    0x28
DS3502 Pot              solar cells 1, pt1    0x29
DS3502 Pot              solar cells 2, pt2    0x2A
DS3502 Pot              solar cells 2, pt1    0x2B
LCD Screen              external              0x27
BME280 TEMP             internal              0x77
BME280 TEMP             external              0x76
***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DS3231.h>             //real time clock
#include <Adafruit_BME280.h>    //temp, humidity, pressure
#include <Adafruit_MMA8451.h>   //accelerometer
#include <Adafruit_DS3502.h>    //potentiometers
#include<ADS1115_WE.h>          //ADC
#include <LiquidCrystal_I2C.h>  //LCD screen

#define SEALEVELPRESSURE_HPA (1013.25)

//RTC
DS3231 clock;
bool h12Flag;
bool pmFlag;

//BME
Adafruit_BME280 bme_internal; // I2C 0x77 address
Adafruit_BME280 bme_external; // I2C 0x76 address

//MMA
Adafruit_MMA8451 mma = Adafruit_MMA8451();

//potentiometers
#define ds1a_address 0x28
#define ds1b_address 0x29
#define ds2a_address 0x2A
#define ds2b_address 0x2B

Adafruit_DS3502 ds1a = Adafruit_DS3502();  //I2C 0x28 address for ds number 0
Adafruit_DS3502 ds1b = Adafruit_DS3502();  //I2C 0x29 address for ds number 1
Adafruit_DS3502 ds2a = Adafruit_DS3502();  //I2C 0x2A address for ds number 2
Adafruit_DS3502 ds2b = Adafruit_DS3502();  //I2C 0x2B address for ds number 3

//ADC
#define adc_address 0x48
ADS1115_WE adc(adc_address);

//LED pin number
int LED = 12;

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,20,4);

unsigned long delayTime;

void setup() {
    // Start the I2C interface
    Wire.begin();

    //start up for the LED
    pinMode(LED, OUTPUT);
    
    lcd.init();
    lcd.backlight();
    lcd.setCursor(3,0);
    lcd.print("Hello, ChemSat!");
    for(int i = 0; i < 5; i++)
    {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
     }

    Serial.begin(9600);
    Serial1.begin(9600);

    delay(2000);
    
    printCSVHeader();

    //BME
    if (! bme_internal.begin(0x77, &Wire)) {
        Serial1.println("Could not find a valid internal BME280 sensor, check wiring!");
        lcd.clear();
        lcd.print("Internal BME sensor issue.");
        delay(1000);
        lcd.clear();
    }
    else
    {
      Serial1.println("Found internal BME280 sensor");
        lcd.clear();
        lcd.print("Found internal BME.");
        delay(1000);
        lcd.clear();
    }

    if (! bme_external.begin(0x76, &Wire)) {
        Serial1.println("Could not find a valid external BME280 sensor, check wiring!");
        lcd.print("External BME sensor issue.");
        delay(1000);
        lcd.clear();
    }
    else
    {
      Serial1.println("Found external BME280 sensor");
        lcd.print("Found external BME.");
        delay(1000);
        lcd.clear();
    }

    //MMA
    if (! mma.begin()) {
      Serial1.println("Couldnt start mma");
      lcd.print("MMA sensor issue.");
      delay(1000);
      lcd.clear();
    }
    else {
      Serial1.println("Found MMA sensor");
        lcd.print("Found MMA.");
        delay(1000);
        lcd.clear();
    }
    
    Serial1.print("Sealevelhpa,"); Serial.print(SEALEVELPRESSURE_HPA);
    mma.setRange(MMA8451_RANGE_2_G);
    Serial1.print(",Range = "); Serial.print(2 << mma.getRange());  
    Serial1.println("G");

    //potentiometers
    if (!ds1a.begin(ds1a_address, &Wire)) {
        Serial1.println("Couldn't find DS1a chip");
        lcd.println("DS 1 sensor issue.");
        delay(1000);
        lcd.clear();
        while (1);
    }
    else
    {
      Serial1.println("Found internal DS1a chip");
        lcd.print("Found DS 1 chip");
        delay(1000);
        lcd.clear();
    }

    if (!ds1b.begin(ds1b_address, &Wire)) {
        Serial1.println("Couldn't find DS1b chip");
        lcd.print("DS 2 sensor issue.");
        delay(1000);
        lcd.clear();
        while (1);
    }
    else
    {
      Serial1.println("Found internal DS1b chip");
        lcd.print("Found DS 2 chip");
        delay(1000);
        lcd.clear();
    }
    
    if (!ds2a.begin(ds2a_address, &Wire)) {
        Serial1.println("Couldn't find DS2a chip");
        lcd.print("DS 3 sensor issue.");
        delay(1000);
        lcd.clear();
        while (1);
    }
    else
    {
      Serial1.println("Found internal DSaa chip");
        lcd.print("Found DS 3 chip");
        delay(1000);
        lcd.clear();
    }

    if (!ds2b.begin(ds2b_address, &Wire)) {
        Serial1.println("Couldn't find DS2b chip");
        lcd.print("DS 4 sensor issue.");
        delay(1000);
        lcd.clear();
        while (1);
    }
    else
    {
      Serial1.println("Found internal DS2b chip");
        lcd.print("Found DS 4 chip");
        delay(1000);
        lcd.clear();
    }
    
    //for the ADC
    if(!adc.init()){
      Serial1.println("ADS1115 not connected!");
      lcd.print("ADS sensor issue.");
      delay(1000);
      lcd.clear();
    }
    else
    {
      Serial1.println("Found ADC");
        lcd.print("Found ADC");
        delay(1000);
        lcd.clear();
    }

    adc.setVoltageRange_mV(ADS1115_RANGE_6144); //comment line/change parameter to change range, want 1024
    adc.setCompareChannels(ADS1115_COMP_0_GND); //comment line/change parameter to change channel
    adc.setMeasureMode(ADS1115_CONTINUOUS); //comment line/change parameter to change mode

    Serial1.println();

    lcd.setCursor(0,0);
    lcd.print("Hi ChemSat,");
    lcd.setCursor(0,1);
    lcd.print("Sytem ready");a
    lcd.setCursor(0,2);
    lcd.print("Gathering data...");
    lcd.setCursor(0,4);
    lcd.print("Let's have fun!");
    for(int i = 0; i < 5; i++)
    {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
     }

    Serial1.println("Starting data gathering...");
    delay(1000);
    lcd.clear();
    
    //to be deleted once all the code is put together
    delayTime = 1000;
}


void loop() {
    getDSData();
    Serial.println("Getting data...");
    delay(delayTime);
    Serial1.println();
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_mV(); // alternative: getResult_V for Volt
  return voltage;
}

void getDSData(){
    float wiper_value1b;
    float wiper_value1a;
    uint8_t w_value1b;
    uint8_t w_value1a;
    float resistance = 78.0;
    float ohm_value1b = 0;
    float ohm_value1a = 0;
    float wiper_value2b;
    float wiper_value2a;
    uint8_t w_value2b;
    uint8_t w_value2a;
    float ohm_value2b = 0;
    float ohm_value2a = 0;

    ds1b.setWiper(0);
    ds1a.setWiper(0);
    ds2b.setWiper(0);
    ds2a.setWiper(0);

//for the pot number 1b, with 1a set to 0
    for (int i = 0; i <= 127; i+=4){
       ds1b.setWiper(i);
       w_value1b = i;
       delay(delayTime);
       getSensorData();
       Serial1.print(w_value1b);
       Serial1.print(",");
       ohm_value1b = w_value1b * resistance;
       Serial1.print(ohm_value1b);
       Serial1.print(",");
       Serial1.print(w_value1a);
       Serial1.print(",");
       ohm_value1a = w_value1a * resistance;
       Serial1.print(ohm_value1a);
       Serial1.print(",");
       
       ds2b.setWiper(i);
       w_value2b = i;
       Serial1.print(w_value2b);
       Serial1.print(",");
       ohm_value2b = w_value2b * resistance;
       Serial1.print(ohm_value2b);
       Serial1.print(",");
       Serial1.print(w_value2a);
       Serial1.print(",");
       ohm_value2a = w_value2a * resistance;
       Serial1.print(ohm_value2a);
       Serial1.print(",");
       getADCData();
       blinkLED();
    }


//for the pot number 1a, with 1b set to 127, continuing in steps of 10
    for (int j = 1; j <= 127; j+=4){
int i = 127;
ds1b.setWiper(i);
w_value1b = i;
ds2b.setWiper(i);
w_value2b = i;

       ds1a.setWiper(j);
       w_value1a = j;
       delay(delayTime);
       getSensorData();
       Serial1.print(w_value1b);
       Serial1.print(",");
       ohm_value1b = w_value1b * resistance;
       Serial1.print(ohm_value1b);
       Serial1.print(",");
       Serial1.print(w_value1a);
       Serial1.print(",");
       ohm_value1a = w_value1a * resistance;
       Serial1.print(ohm_value1a);
       Serial1.print(",");

       ds2a.setWiper(j);
       w_value2a = j;
       Serial1.print(w_value2b);
       Serial1.print(",");
       ohm_value2b = w_value2b * resistance;
       Serial1.print(ohm_value2b);
       Serial1.print(",");
       Serial1.print(w_value2a);
       Serial1.print(",");
       ohm_value2a = w_value2a * resistance;
       Serial1.print(ohm_value2a);
       Serial1.print(",");
       getADCData();
       blinkLED();
    }

//for the pot number 1a set to 127, with 1b set to 127, a half step
int j = 127;      
       ds1a.setWiper(j);
       w_value1a = j;
       delay(delayTime);
       getSensorData();
       Serial1.print(w_value1b);
       Serial1.print(",");
       ohm_value1b = w_value1b * resistance;
       Serial1.print(ohm_value1b);
       Serial1.print(",");
       Serial1.print(w_value1a);
       Serial1.print(",");
       ohm_value1a = w_value1a * resistance;
       Serial1.print(ohm_value1a);
       Serial1.print(",");

       ds2a.setWiper(j);
       w_value2a = j;
       Serial1.print(w_value2b);
       Serial1.print(",");
       ohm_value2b = w_value2b * resistance;
       Serial1.print(ohm_value2b);
       Serial1.print(",");
       Serial1.print(w_value2a);
       Serial1.print(",");
       ohm_value2a = w_value2a * resistance;
       Serial1.print(ohm_value2a);
       Serial1.print(",");
       getADCData();
       blinkLED();

    Serial1.println();
}

void getSensorData(){
    getRTCData();
    
    // Only needed in forced mode! In normal mode, you can remove the next line.
    bme_internal.takeForcedMeasurement(); // has no effect in normal mode
    bme_external.takeForcedMeasurement();
    printValues(bme_internal);
    printValues(bme_external);
    
    getMMAValues();
}

void getADCData(){
  float voltage = 0.0;

  float cOne = 0;
  float cTwo = 0;

  voltage = readChannel(ADS1115_COMP_0_1);
  Serial1.print(voltage);
  cOne = voltage;
  Serial1.print(",");
  voltage = 0.0;
    voltage = readChannel(ADS1115_COMP_2_3);
  cTwo = voltage;
  Serial1.println(voltage);

  //print to the LCD screen
  lcd.setCursor(0,1);
  lcd.print("Cell 1 mV: ");
  lcd.print(cOne);
  blinkLED();
  lcd.setCursor(0,2);
  lcd.print("Cell 2 mV: ");
  lcd.print(cTwo);
}

void printCSVHeader() {
    byte error, address;
    address = 39; //address byte for lcd screen
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) //if lcd is available
    {
      Serial1.println("For solar cells 1 and 2.");
    }
    else //if lcd is not available
    {
      Serial1.println("For solar cells 3 and 4, internal and external BME sensors (temp/hum/press) are switched.");
    }   
    
    Serial1.print("H,M,S,clockTemp,");                                   //rtc
    Serial1.print("internalTemp,internalPres,internalAlt,internalHum,"); //internal bme
    Serial1.print("externalTemp,externalPres,externalAlt,externalHum,"); //external bme
    Serial1.print("x,y,z,xAccel,yAccel,zAccel,orientation,");            //mma
    Serial1.print("bin2,ds1b,bin1,ds1a,bin4,ds2b,bin3,ds2a,");                               //pots
    Serial1.print("cell1,cell2,");                                 //adc
    Serial1.println();
    Serial1.print("Real,Time,Clock,C,");
    Serial1.print("C,hPa,m,%,");
    Serial1.print("C,hPa,m,%,");
    Serial1.print("rawData,rawData,rawData,m/s^2 ,m/s^2 ,m/s^2,facing,");
    Serial1.print("i,ohms,j,ohms,k,ohms,l,ohms,");
    Serial1.print("mV,mV,");
    Serial1.println();
}

void getRTCData(){
  // send what's going on to the serial monitor.
  //Print the hour, minute, and second
  int h;
  int m;
  int s;
  Serial1.print(clock.getHour(h12Flag, pmFlag), DEC);
  h = clock.getHour(h12Flag, pmFlag);
    Serial1.print(",");
  Serial1.print(clock.getMinute(), DEC);
  m = clock.getMinute();
    Serial1.print(",");
  Serial1.print(clock.getSecond(), DEC);
  s = clock.getSecond();
    Serial1.print(",");

  // Display the temperature
  Serial1.print(clock.getTemperature(), 2);
    Serial1.print(",");

    //print to the LCD screen
  lcd.setCursor(0,0);
  lcd.print("RTC: ");
  lcd.print(h);
  lcd.print(":");
  lcd.print(m);
  lcd.print(":");
  lcd.print(s);
}

void printValues(Adafruit_BME280 &bme) {
  float temp = 0;
    Serial1.print(bme.readTemperature());
        Serial1.print(",");
    Serial1.print(bme.readPressure() / 100.0F);
        Serial1.print(",");
    Serial1.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        Serial1.print(",");
    Serial1.print(bme.readHumidity());
        Serial1.print(",");
    temp = bme.readTemperature();

//print to the LCD screen
  lcd.setCursor(0,3);
  lcd.print("Ext. Temp C: ");
  lcd.print(temp);     //default prints out the external since it is the second reading that comes through with how the sensor code is set up
}

void getMMAValues()
{
    // Read the 'raw' data in 14-bit counts
    mma.read();
    Serial1.print(mma.x);
    Serial1.print(",");
    Serial1.print(mma.y);
    Serial1.print(",");
    Serial1.print(mma.z);
    Serial1.print(",");

    /* Get a new sensor event */
    sensors_event_t event;
    mma.getEvent(&event);

    /* Display the results (acceleration is measured in m/s^2) */
    Serial1.print(event.acceleration.x);
    Serial1.print(",");
    Serial1.print(event.acceleration.y);
    Serial1.print(",");
    Serial1.print(event.acceleration.z);
    Serial1.print(",");

    /* Get the orientation of the sensor */
    uint8_t o = mma.getOrientation();

    switch (o) {
    case MMA8451_PL_PUF:
        Serial1.print
        ("Portrait Up Front");
        break;
    case MMA8451_PL_PUB:
        Serial1.print("Portrait Up Back");
        break;
    case MMA8451_PL_PDF:
        Serial1.print("Portrait Down Front");
        break;
    case MMA8451_PL_PDB:
        Serial1.print("Portrait Down Back");
        break;
    case MMA8451_PL_LRF:
        Serial1.print("Landscape Right Front");
        break;
    case MMA8451_PL_LRB:
        Serial1.print("Landscape Right Back");
        break;
    case MMA8451_PL_LLF:
        Serial1.print("Landscape Left Front");
        break;
    case MMA8451_PL_LLB:
        Serial1.print("Landscape Left Back");
        break;
    }
        Serial1.print(",");
}

void blinkLED(){
    digitalWrite(LED, HIGH);
    delay(25);
    digitalWrite(LED, LOW);
}
