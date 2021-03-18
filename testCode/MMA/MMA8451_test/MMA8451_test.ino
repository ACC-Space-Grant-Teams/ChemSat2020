/*
Code came from Adafruit_MMA8451_Library -> examples -> MMA8451demo
Jillian Frimml
3/18/2021

Designed to read the information from MMA sensor

*/

#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451(); //I2C 0x1D

unsigned long delayTime;

void setup() {
    Serial.begin(9600);

    if (!mma.begin()) {
        Serial.println("Couldnt start");
    }
    mma.setRange(MMA8451_RANGE_2_G);
    Serial.print("Range = "); Serial.print(2 << mma.getRange());
    Serial.println("G");

    delayTime = 1000;

    printCSVHeader();
}

void loop() {
    getMMAValues();
    delay(delayTime);
}

void printCSVHeader()
{
    Serial.print("rawX,rawY,rawZ,m/s^2 ,m/s^2 ,m/s^2,portrait,");
    Serial.println();
    Serial.print("x,y,z,xAccel,yAccel,zAccel,orientation,");
    Serial.println();
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
        Serial.println("Portrait Up Front");
        break;
    case MMA8451_PL_PUB:
        Serial.println("Portrait Up Back");
        break;
    case MMA8451_PL_PDF:
        Serial.println("Portrait Down Front");
        break;
    case MMA8451_PL_PDB:
        Serial.println("Portrait Down Back");
        break;
    case MMA8451_PL_LRF:
        Serial.println("Landscape Right Front");
        break;
    case MMA8451_PL_LRB:
        Serial.println("Landscape Right Back");
        break;
    case MMA8451_PL_LLF:
        Serial.println("Landscape Left Front");
        break;
    case MMA8451_PL_LLB:
        Serial.println("Landscape Left Back");
        break;
    }
}
