
/***************************************************************************
* Example sketch from the ADS1115_WE library
*
* Jillian Frimml
* 3/24/2021
* Compares the ADS1115 in continuous mode, comparing all pins to each other
*
* 
***************************************************************************/

#include<ADS1115_WE.h> 
#include<Wire.h>
#include <Adafruit_DS3502.h>

#define ds1a_address 0x28
#define ds1b_address 0x29
#define ds2a_address 0x2A
#define ds2b_address 0x2B

Adafruit_DS3502 ds1a = Adafruit_DS3502();  //I2C 0x28 address for ds number 0
Adafruit_DS3502 ds1b = Adafruit_DS3502();  //I2C 0x29 address for ds number 1
Adafruit_DS3502 ds2a = Adafruit_DS3502();  //I2C 0x28 address for ds number 2
Adafruit_DS3502 ds2b = Adafruit_DS3502();  //I2C 0x29 address for ds number 3

#define adc_address 0x48
ADS1115_WE adc(adc_address);

unsigned long delayTime;


void setup() {
  Wire.begin();
  Serial.begin(9600);

//for the DS
  while (!Serial) { delay(1); }


    Serial.println("Adafruit DS Series Test");
    
    if (!ds1a.begin(ds1a_address, &Wire)) {
        Serial.println("Couldn't find DS1a chip");
        while (1);
    }

    if (!ds1b.begin(ds1b_address, &Wire)) {
        Serial.println("Couldn't find DS1b chip");
        while (1);
    }
    
    if (!ds2a.begin(ds2a_address, &Wire)) {
        Serial.println("Couldn't find DS2a chip");
        while (1);
    }

    if (!ds2b.begin(ds2b_address, &Wire)) {
        Serial.println("Couldn't find DS2b chip");
        while (1);
    }
    
//for the ADC
  if(!adc.init()){
    Serial.println("ADS1115 not connected!");
  }

  adc.setVoltageRange_mV(ADS1115_RANGE_6144); //comment line/change parameter to change range, want 1024

  adc.setCompareChannels(ADS1115_COMP_0_GND); //comment line/change parameter to change channel

  adc.setMeasureMode(ADS1115_CONTINUOUS); //comment line/change parameter to change mode

  Serial.println("ADS1115  - Continuous Mode");
  Serial.println("All values in mvolts");
  Serial.println();
  
  Serial.println("bin2,ds1b,bin1,ds1a,bin4,ds2b,bin3,ds2a,cell1,cell2");
  Serial.println("i,ohms,j,ohms,mV,k,ohms,l,ohms,mV,mV");

  
  delayTime = 1000;
}



void loop() {
  getDSData();
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

//for the pot number 1
    for (int i = 0; i <= 127; i+=10){
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
    }

int i = 127;
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

//for the pot number 0
    for (int j = 0; j <= 127; j+=10){
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
       delay(delayTime);
    }

int j = 127;
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
       delay(delayTime);

    Serial.println();
    //delay(delay_time);
}

void getADCData(){
  float voltage = 0.0;

  voltage = readChannel(ADS1115_COMP_0_1);
  Serial.print(voltage);
  Serial.print(",");
  voltage = 0.0;
    voltage = readChannel(ADS1115_COMP_2_3);
  Serial.println(voltage);

}
