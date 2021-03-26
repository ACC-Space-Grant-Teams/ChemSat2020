
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

#define ds1_address 0x28
#define ds2_address 0x29

Adafruit_DS3502 ds1 = Adafruit_DS3502();  //I2C 0x28 address
Adafruit_DS3502 ds2 = Adafruit_DS3502();  //I2C 0x29 address
/* For this example, make the following connections:
    * DS3502 RH to 5V
    * DS3502 RL to GND
    * DS3502 RW to the pin specified by WIPER_VALUE_PIN
*/

#define WIPER_VALUE_PIN1 A0   //for ds1
#define WIPER_VALUE_PIN2 A1   //for ds2


#define I2C_ADDRESS 0x48
ADS1115_WE adc(I2C_ADDRESS);
// ADS1115_WE adc = ADS1115_WE(); // Alternative: uses default address 0x48

int delay_time = 500;


void setup() {
  Wire.begin();
  Serial.begin(9600);

//for the DS
  while (!Serial) { delay(1); }


    Serial.println("Adafruit DS Series Test");
    
    if (!ds1.begin(ds1_address, &Wire)) {
        Serial.println("Couldn't find DS1 chip");
        while (1);
    }

    if (!ds2.begin(ds2_address, &Wire)) {
        Serial.println("Couldn't find DS2 chip");
        while (1);
    }


//for the ADC
  if(!adc.init()){
    Serial.println("ADS1115 not connected!");
  }

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

  Serial.println("bin2,ds2,bin1,ds1,adc");
  Serial.println("i,ohms,n,ohms,mV");
}

  /* If you change the compare channels you can immediately read values from the conversion 
   * register, although they might belong to the former channel if no precautions are taken. 
   * It takes about the time needed for two conversions to get the correct data. In single 
   * shot mode you can use the isBusy() function to wait for data from the new channel. This 
   * does not work in continuous mode. 
   * To solve this issue the library adds a delay after change of channels if you are in contunuous
   * mode. The length of the delay is adjusted to the conversion rate. But be aware that the output 
   * rate will be much lower that the conversion rate if you change channels frequently. 
   */

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
    float wiper_value1;
    float wiper_value2;
    uint8_t w_value1;
    uint8_t w_value2;
    float resistance = 78.0;

    /*
    ds2.setWiper(127);
    w_value2 = ds2.getWiper();
    Serial.print("DS2 - Wiper voltage with wiper set to " );
    Serial.print(w_value2);
    Serial.print(" : ");
    wiper_value2 = analogRead(WIPER_VALUE_PIN2);
    wiper_value2 *= 5.0;
    wiper_value2 /= 1024;
    Serial.print(wiper_value2);
    Serial.println(" V");

    ds1.setWiper(127);
    w_value1 = ds1.getWiper();
    Serial.print("DS1 - Wiper voltage with wiper set to " );
    Serial.print(w_value1);
    Serial.print(" : ");
    wiper_value1 = analogRead(WIPER_VALUE_PIN1);
    wiper_value1 *= 5.0;
    wiper_value1 /= 1024;
    Serial.print(wiper_value1);
    Serial.println(" V");
    */

ds2.setWiper(0);
ds1.setWiper(0);
//w_value2 = ds2.getWiper() * resistance;
//w_value1 = ds1.getWiper() * resistance;
float ohm_value2;
float ohm_value1;

ohm_value2 = 0;
ohm_value1 = 0;

    for (int n = 0; n <= 127; n+=10){
       ds2.setWiper(n);
       w_value2 = n;
       Serial.print(w_value2);
       Serial.print(",");
       ohm_value2 = w_value2 * resistance;
       Serial.print(ohm_value2);
       Serial.print(",");
       Serial.print(w_value1);
       Serial.print(",");
       ohm_value1 = w_value1 * resistance;
       Serial.print(ohm_value1);
       Serial.print(",");
       getADSData();
       delay(delay_time);
    }

for (int n = 127; n <= 128; n+=10){
       ds2.setWiper(n);
       w_value2 = n;
       Serial.print(w_value2);
       Serial.print(",");
       ohm_value2 = w_value2 * resistance;
       Serial.print(ohm_value2);
       Serial.print(",");
       Serial.print(w_value1);
       Serial.print(",");
       ohm_value1 = w_value1 * resistance;
       Serial.print(ohm_value1);
       Serial.print(",");
       getADSData();
       delay(delay_time);
    }
    
    for (int n = 0; n <= 127; n+=10){
       ds1.setWiper(n);
       w_value1 = n;
       Serial.print(w_value2);
       Serial.print(",");
       ohm_value2 = w_value2 * resistance;
       Serial.print(ohm_value2);
       Serial.print(",");
       Serial.print(w_value1);
       Serial.print(",");
       ohm_value1 = w_value1 * resistance;
       Serial.print(ohm_value1);
       Serial.print(",");
       getADSData();
       delay(delay_time);
    }

for (int n = 127; n <= 128; n+=10){
       ds1.setWiper(n);
       w_value1 = n;
       Serial.print(w_value2);
       Serial.print(",");
       ohm_value2 = w_value2 * resistance;
       Serial.print(ohm_value2);
       Serial.print(",");
       Serial.print(w_value1);
       Serial.print(",");
       ohm_value1 = w_value1 * resistance;
       Serial.print(ohm_value1);
       Serial.print(",");
       getADSData();
       delay(delay_time);
    }

    

    /*
     for (int i = 0; i < 128; i+=13){
       ds1.setWiper(i);
       w_value1 = ds1.getWiper() * resistance;
       Serial.print(i);
       Serial.print(",");
       Serial.print(w_value2);
       Serial.print(",");
       Serial.print(w_value1);
       Serial.print(",");    
       getADSData();
       delay(delay_time);
    }
    */

    Serial.println();
    //delay(delay_time);
}

void getADSData(){
    float voltage = 0.0;
/*
  Serial.print("0: ");
  voltage = readChannel(ADS1115_COMP_0_GND);
  Serial.print(voltage);

  Serial.print(",   1: ");
  voltage = readChannel(ADS1115_COMP_1_GND);
  Serial.print(voltage);
  
  Serial.print(",   2: ");
  voltage = readChannel(ADS1115_COMP_2_GND);
  Serial.print(voltage);

  Serial.print(",   3: ");
  voltage = readChannel(ADS1115_COMP_3_GND);
  Serial.print(voltage);

  Serial.print("   0 to 1: ");
  voltage = readChannel(ADS1115_COMP_0_1);
  Serial.print(voltage);

  Serial.print(",   0 to 3: ");
  voltage = readChannel(ADS1115_COMP_0_3);
  Serial.print(voltage);
  
  Serial.print(",   1 to 3: ");
  voltage = readChannel(ADS1115_COMP_1_3);
  Serial.print(voltage);
*/

  //Serial.print(",   2 to 3: ");
  voltage = readChannel(ADS1115_COMP_2_3);
  Serial.println(voltage);

  //delay(delay_time);
}
