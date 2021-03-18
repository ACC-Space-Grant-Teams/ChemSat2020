/*
Code came from Adafruit_BME280_Library, examples, bme280test and advancedsettings
Jillian Frimml
3/18/2021

Designed to read 2 BMEs, one for inside a payload, the other for outside the payload

*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

void printCSVHeader();

Adafruit_BME280 bme_internal; // I2C 0x77 address
//Adafruit_BME280 bme_external; // I2C 0x76 address

unsigned long delayTime;

void setup() {
    Serial.begin(9600);

    if (!bme_internal.begin(0x77, &Wire)) {
        Serial.println("Could not find a valid internal BME280 sensor, check wiring!");
    }
    /*
    if (!bme_external.begin(0x76, &Wire)) {
        Serial.println("Could not find a valid external BME280 sensor, check wiring!");
    }
    */

    delayTime = 1000;
    
    printCSVHeader();
}

void loop() {
    // Only needed in forced mode! In normal mode, you can remove the next line.
    bme_internal.takeForcedMeasurement(); // has no effect in normal mode
    //bme_external.takeForcedMeasurement(); // has no effect in normal mode
    printValues(bme_internal);
    //printValues(bme_external);
    delay(delayTime);
}

void printCSVHeader()
{
    Serial.print("C,hPa,m,%,");
    //Serial.print("C,hPa,m,%,");
    Serial.println();
    Serial.print("internalTemp,internalPres,internalAlt,internalHum,");
    //Serial.print("externalTemp,externalPres,externalAlt,externalHum,");
    Serial.println();
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
    Serial.println();
}
