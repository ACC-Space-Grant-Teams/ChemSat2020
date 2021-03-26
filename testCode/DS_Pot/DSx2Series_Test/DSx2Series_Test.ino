/***************************************************************************
* uses 2 potentiometers in series
*
* Jillian Frimml
* 3/25/2021
* This set up has the following parameters:
* DS2 RH has the high voltage input
* DS2 RW is connected to DS1 RW
* DS1 RL is ground
* DS1 RW is connected to A0 for voltage readings
*
* 
***************************************************************************/

#include <Adafruit_DS3502.h>

#define ds1_address 0x28
#define ds2_address 0x29
//#define ds3_address 0x2A
//#define ds4_address 0x2B

Adafruit_DS3502 ds1 = Adafruit_DS3502();  //I2C 0x28 address
Adafruit_DS3502 ds2 = Adafruit_DS3502();  //I2C 0x29 address
/* For this example, make the following connections:
    * DS3502 RH to 5V
    * DS3502 RL to GND
    * DS3502 RW to the pin specified by WIPER_VALUE_PIN
*/

#define WIPER_VALUE_PIN1 A0   //for ds1
#define WIPER_VALUE_PIN2 A1   //for ds2

int delay_time = 5000;

void setup() {
    Serial.begin(9600);
    // Wait until serial port is opened
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

}

void loop() {
    float wiper_value1;
    float wiper_value2;
    uint8_t w_value1;
    uint8_t w_value2;

    ds2.setWiper(0);
    w_value2 = ds2.getWiper();
    Serial.print("DS2 - Wiper voltage with wiper set to " );
    Serial.println(w_value2);
    //Serial.print(" : ");
    //wiper_value2 = analogRead(WIPER_VALUE_PIN2);
    //wiper_value2 *= 5.0;
    //wiper_value2 /= 1024;
    //Serial.print(wiper_value2);
    //Serial.println(" V");

    ds1.setWiper(0);
    w_value1 = ds1.getWiper();
    Serial.print("DS1 - Wiper voltage with wiper set to " );
    Serial.print(w_value1);
    Serial.print(" : ");
    wiper_value1 = analogRead(WIPER_VALUE_PIN1);
    wiper_value1 *= 5.0;
    wiper_value1 /= 1024;
    Serial.print(wiper_value1);
    Serial.println(" V");

    Serial.println();
    delay(delay_time);

}
