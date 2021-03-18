/*
Code for 2 BMEs and an MMA
Jillian Frimml
3/18/2021

Designed to read 2 BMEs, one for internal, one for external
Print out of the BMEs information
Print out of the MMA information
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>    //temp, humidity, pressure
#include <Adafruit_MMA8451.h>   //accelerometer

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme_internal; // I2C 0x77 address
Adafruit_BME280 bme_external; // I2C 0x76 address
Adafruit_MMA8451 mma = Adafruit_MMA8451();

void printCSVHeader();
unsigned long delayTime;

void setup() {
    Serial.begin(9600);

    delay(5000);
    printCSVHeader();
    delay(5000);

    if (! bme_internal.begin(0x77, &Wire)) {
        Serial.println("Could not find a valid internal BME280 sensor, check wiring!");
    }

    if (! bme_external.begin(0x76, &Wire)) {
        Serial.println("Could not find a valid external BME280 sensor, check wiring!");
    }

    if (! mma.begin()) {
      Serial.println("Couldnt start");
    }
    
    Serial.print("Sealevelhpa,"); Serial.print(SEALEVELPRESSURE_HPA);
    mma.setRange(MMA8451_RANGE_2_G);
    Serial.print(",Range = "); Serial.print(2 << mma.getRange());  
    Serial.println("G");

    delayTime = 1000;
}


void loop() {
    // Only needed in forced mode! In normal mode, you can remove the next line.
    bme_internal.takeForcedMeasurement(); // has no effect in normal mode
    bme_external.takeForcedMeasurement();
    printValues(bme_internal);
    printValues(bme_external);
    
    getMMAValues();
    delay(delayTime);
    Serial.println();
}

void printCSVHeader() {
    Serial.print("C,hPa,m,%,");
    Serial.print("C,hPa,m,%,");
    Serial.print("rawData,rawData,rawData,m/s^2 ,m/s^2 ,m/s^2,facing,");
    Serial.println();
    Serial.print("internalTemp,internalPres,internalAlt,internalHum,");
    Serial.print("externalTemp,externalPres,externalAlt,externalHum,");
    Serial.print("x,y,z,xAccel,yAccel,zAccel,orientation,");
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
        break;
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
}
