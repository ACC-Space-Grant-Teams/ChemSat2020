#include <Adafruit_DS3502.h>

#define ds1_address 0x28
#define ds2_address 0x29
//#define ds3_address 0x2A
//#define ds4_address 0x2B

Adafruit_DS3502 ds3502 = Adafruit_DS3502();
/* For this example, make the following connections:
    * DS3502 RH to 5V
    * DS3502 RL to GND
    * DS3502 RW to the pin specified by WIPER_VALUE_PIN
*/

#define WIPER_VALUE_PIN A0


void setup() {
    Serial.begin(9600);
    // Wait until serial port is opened
    while (!Serial) { delay(1); }

    Serial.println("Adafruit DS3502 Test");

    if (!ds3502.begin(ds2_address, &Wire)) {
        Serial.println("Couldn't find DS3502 chip");
        while (1);
    }
    Serial.println("Found DS3502 chip");
}

void loop() {
    float wiper_value;
    uint8_t w_value;
    

    ds3502.setWiper(0);
    w_value = ds3502.getWiper();
    Serial.print("Wiper voltage with wiper set to " );
    Serial.print(w_value);
    Serial.print(" : ");
    wiper_value = analogRead(WIPER_VALUE_PIN);
    wiper_value *= 5.0;
    wiper_value /= 1024;
    Serial.print(wiper_value);
    Serial.println(" V");

    Serial.println();
    delay(1000);

    ds3502.setWiper(63);
    w_value = ds3502.getWiper();
    Serial.print("Wiper voltage with wiper set to ");
    Serial.print(w_value);
    Serial.print(" : ");
    wiper_value = analogRead(WIPER_VALUE_PIN);
    wiper_value *= 5.0;
    wiper_value /= 1024;
    Serial.print(wiper_value);
    Serial.println(" V");

    Serial.println();
    delay(1000);

    ds3502.setWiper(127);
    w_value = ds3502.getWiper();
    Serial.print("Wiper voltage with wiper set to  ");
    Serial.print(w_value);
    Serial.print(" : ");
    wiper_value = analogRead(WIPER_VALUE_PIN);
    wiper_value *= 5.0;
    wiper_value /= 1024;
    Serial.print(wiper_value);
    Serial.println(" V");

    Serial.println();
    delay(1000);
}
