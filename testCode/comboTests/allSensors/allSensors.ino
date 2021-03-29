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
    lcd.print("Hello, world!");
    for(int i = 0; i < 5; i++)
    {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
     }

    
    Serial.begin(9600);

    delay(2000);
    
    printCSVHeader();

    //BME
    if (! bme_internal.begin(0x77, &Wire)) {
        Serial.println("Could not find a valid internal BME280 sensor, check wiring!");
        lcd.println("Internal BME sensor issue.");
        lcd.clear();
    }

    if (! bme_external.begin(0x76, &Wire)) {
        Serial.println("Could not find a valid external BME280 sensor, check wiring!");
        lcd.println("External BME sensor issue.");
        lcd.clear();
    }

    //MMA
    if (! mma.begin()) {
      Serial.println("Couldnt start mma");
      lcd.println("MMA sensor issue.");
      lcd.clear();
    }
    
    Serial.print("Sealevelhpa,"); Serial.print(SEALEVELPRESSURE_HPA);
    mma.setRange(MMA8451_RANGE_2_G);
    Serial.print(",Range = "); Serial.print(2 << mma.getRange());  
    Serial.println("G");

    //potentiometers
    if (!ds1a.begin(ds1a_address, &Wire)) {
        Serial.println("Couldn't find DS1a chip");
        lcd.println("DS 1 sensor issue.");
        lcd.clear();
        while (1);
    }

    if (!ds1b.begin(ds1b_address, &Wire)) {
        Serial.println("Couldn't find DS1b chip");
        lcd.println("DS 2 sensor issue.");
        lcd.clear();
        while (1);
    }
    
    if (!ds2a.begin(ds2a_address, &Wire)) {
        Serial.println("Couldn't find DS2a chip");
        lcd.println("DS 3 sensor issue.");
        lcd.clear();
        while (1);
    }

    if (!ds2b.begin(ds2b_address, &Wire)) {
        Serial.println("Couldn't find DS2b chip");
        lcd.println("DS 4 sensor issue.");
        lcd.clear();
        while (1);
    }
    
    //for the ADC
    if(!adc.init()){
      Serial.println("ADS1115 not connected!");
      lcd.println("ADS sensor issue.");
      lcd.clear();
    }

    adc.setVoltageRange_mV(ADS1115_RANGE_6144); //comment line/change parameter to change range, want 1024
    adc.setCompareChannels(ADS1115_COMP_0_GND); //comment line/change parameter to change channel
    adc.setMeasureMode(ADS1115_CONTINUOUS); //comment line/change parameter to change mode

    Serial.println();


    while(!Serial){
        ; // wait for serial port to connect. Needed for native USB port only
    }
    lcd.setCursor(2,1);
    lcd.print("Sytem ready");
    delay(1000);
    lcd.clear();
    
    //to be deleted once all the code is put together
    delayTime = 1000;
}


void loop() {
    getDSData();
    
    delay(delayTime);
    Serial.println();
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
    for (int i = 0; i <= 127; i+=10){
       getSensorData();
       ds1b.setWiper(i);
       w_value1b = i;
       Serial.print(w_value1b);
       Serial.print(",");
       ohm_value1b = w_value1b * resistance;
       Serial.print(ohm_value1b);
       Serial.print(",");
       Serial.print(w_value1a);
       Serial.print(",");
       ohm_value1a = w_value1a * resistance;
       Serial.print(ohm_value1a);
       Serial.print(",");
       
       ds2b.setWiper(i);
       w_value2b = i;
       Serial.print(w_value2b);
       Serial.print(",");
       ohm_value2b = w_value2b * resistance;
       Serial.print(ohm_value2b);
       Serial.print(",");
       Serial.print(w_value2a);
       Serial.print(",");
       ohm_value2a = w_value2a * resistance;
       Serial.print(ohm_value2a);
       Serial.print(",");
       getADCData();
       delay(delayTime);
       blinkLED();
    }


//for the pot number 1b set to 127, with 1a set to 0
/*int i = 127;

       getSensorData();
       ds1b.setWiper(i);
       w_value1b = i;
       Serial.print(w_value1b);
       Serial.print(",");
       ohm_value1b = w_value1b * resistance;
       Serial.print(ohm_value1b);
       Serial.print(",");
       Serial.print(w_value1a);
       Serial.print(",");
       ohm_value1a = w_value1a * resistance;
       Serial.print(ohm_value1a);
       Serial.print(",");

       ds2b.setWiper(i);
       w_value2b = i;
       Serial.print(w_value2b);
       Serial.print(",");
       ohm_value2b = w_value2b * resistance;
       Serial.print(ohm_value2b);
       Serial.print(",");
       Serial.print(w_value2a);
       Serial.print(",");
       ohm_value2a = w_value2a * resistance;
       Serial.print(ohm_value2a);
       Serial.print(",");
       getADCData();
       blinkLED();
       delay(delayTime);
*/
//for the pot number 1a, with 1b set to 127
    for (int j = 3; j <= 127; j+=10){
int i = 127;
ds1b.setWiper(i);
w_value1b = i;
ds2b.setWiper(i);
w_value2b = i;

      getSensorData();
       ds1a.setWiper(j);
       w_value1a = j;
       Serial.print(w_value1b);
       Serial.print(",");
       ohm_value1b = w_value1b * resistance;
       Serial.print(ohm_value1b);
       Serial.print(",");
       Serial.print(w_value1a);
       Serial.print(",");
       ohm_value1a = w_value1a * resistance;
       Serial.print(ohm_value1a);
       Serial.print(",");

       ds2a.setWiper(j);
       w_value2a = j;
       Serial.print(w_value2b);
       Serial.print(",");
       ohm_value2b = w_value2b * resistance;
       Serial.print(ohm_value2b);
       Serial.print(",");
       Serial.print(w_value2a);
       Serial.print(",");
       ohm_value2a = w_value2a * resistance;
       Serial.print(ohm_value2a);
       Serial.print(",");
       getADCData();
       blinkLED();
       delay(delayTime);
    }

//for the pot number 1a set to 127, with 1b set to 127
int j = 127;
      getSensorData();
       ds1a.setWiper(j);
       w_value1a = j;
       Serial.print(w_value1b);
       Serial.print(",");
       ohm_value1b = w_value1b * resistance;
       Serial.print(ohm_value1b);
       Serial.print(",");
       Serial.print(w_value1a);
       Serial.print(",");
       ohm_value1a = w_value1a * resistance;
       Serial.print(ohm_value1a);
       Serial.print(",");

       ds2a.setWiper(j);
       w_value2a = j;
       Serial.print(w_value2b);
       Serial.print(",");
       ohm_value2b = w_value2b * resistance;
       Serial.print(ohm_value2b);
       Serial.print(",");
       Serial.print(w_value2a);
       Serial.print(",");
       ohm_value2a = w_value2a * resistance;
       Serial.print(ohm_value2a);
       Serial.print(",");
       getADCData();
       blinkLED();
       delay(delayTime);

    Serial.println();
    //delay(delay_time);
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
  Serial.print(voltage);
  cOne = voltage;
  Serial.print(",");
  voltage = 0.0;
    voltage = readChannel(ADS1115_COMP_2_3);
  cTwo = voltage;
  Serial.println(voltage);

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
      Serial.println("For solar cells 1 and 2.");
    }
    else //if lcd is not available
    {
      Serial.println("For solar cells 3 and 4.");
    }   
    
    Serial.print("H,M,S,clockTemp,");                                   //rtc
    Serial.print("internalTemp,internalPres,internalAlt,internalHum,"); //internal bme
    Serial.print("externalTemp,externalPres,externalAlt,externalHum,"); //external bme
    Serial.print("x,y,z,xAccel,yAccel,zAccel,orientation,");            //mma
    Serial.print("bin2,ds1b,bin1,ds1a,bin4,ds2b,bin3,ds2a,");                               //pots
    Serial.print("cell1,cell2,");                                 //adc
    Serial.println();
    Serial.print("Real,Time,Clock,C,");
    Serial.print("C,hPa,m,%,");
    Serial.print("C,hPa,m,%,");
    Serial.print("rawData,rawData,rawData,m/s^2 ,m/s^2 ,m/s^2,facing,");
    Serial.print("i,ohms,j,ohms,mV,k,ohms,l,ohms,");
    Serial.print("mV,mV,");
    Serial.println();
}

void getRTCData(){
  // send what's going on to the serial monitor.
  //Print the hour, minute, and second
  int h = 0;
  int m = 0;
  int s = 0;
  int hms = 0;
  Serial.print(clock.getHour(h12Flag, pmFlag), DEC);
  h = clock.getHour(h12Flag, pmFlag) / (60*60);
    Serial.print(",");
  Serial.print(clock.getMinute(), DEC);
  m = clock.getMinute() / (60);
    Serial.print(",");
  Serial.print(clock.getSecond(), DEC);
  s = clock.getSecond();
    Serial.print(",");
  hms = h + m + s;

  // Display the temperature
  Serial.print(clock.getTemperature(), 2);
    Serial.print(",");

    //print to the LCD screen
  lcd.setCursor(0,0);
  lcd.print("RTC: ");
  lcd.print(h);
  lcd.print(":");
  lcd.print(m);
  lcd.print(":");
  lcd.print(s);
  //lcd.print(hms);
}

void printValues(Adafruit_BME280 &bme) {
  float temp = 0;
    Serial.print(bme.readTemperature());
        Serial.print(",");
    Serial.print(bme.readPressure() / 100.0F);
        Serial.print(",");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        Serial.print(",");
    Serial.print(bme.readHumidity());
        Serial.print(",");
    temp = bme.readTemperature();

//print to the LCD screen
  lcd.setCursor(0,3);
  lcd.print("Ext. Temp C: ");
  lcd.print(temp);     
}

void getMMAValues()
{
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
        Serial.print
        ("Portrait Up Front");
        break;
    case MMA8451_PL_PUB:
        Serial.print("Portrait Up Back");
        break;
    case MMA8451_PL_PDF:
        Serial.print("Portrait Down Front");
        break;
    case MMA8451_PL_PDB:
        Serial.print("Portrait Down Back");
        break;
    case MMA8451_PL_LRF:
        Serial.print("Landscape Right Front");
        break;a
    case MMA8451_PL_LRB:
        Serial.print("Landscape Right Back");
        break;
    case MMA8451_PL_LLF:
        Serial.print("Landscape Left Front");
        break;
    case MMA8451_PL_LLB:
        Serial.print("Landscape Left Back");
        break;
    }
        Serial.print(",");
}

void blinkLED(){
    digitalWrite(LED, HIGH);
    delay(50);
    digitalWrite(LED, LOW);
    delay(50);
}
