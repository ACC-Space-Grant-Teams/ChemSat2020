
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

unsigned long delayTime;

void setup() {
    Serial.begin(9600);

    if (! bme_internal.begin(0x76, &Wire)) {
        Serial.println("Could not find a valid internal BME280 sensor, check wiring!");
    }

    if (! bme_external.begin(0x77, &Wire)) {
        Serial.println("Could not find a valid external BME280 sensor, check wiring!");
    }

    if (! mma.begin()) {
      Serial.println("Couldnt start");
    }
    Serial.print("Sealevelhpa,"); Serial.print(SEALEVELPRESSURE_HPA);
    mma.setRange(MMA8451_RANGE_2_G);
    Serial.print(",Range = "); Serial.print(2 << mma.getRange());  
    Serial.println("G");

    printCsvHeader();
}


void loop() {
    // Only needed in forced mode! In normal mode, you can remove the next line.
    bme_internal.takeForcedMeasurement(); // has no effect in normal mode
    bme_external.takeForcedMeasurement();
    printValues(bme_internal);
    printValues(bme_external);
    delay(delayTime);
    getMMAData();
    delay(delayTime);
    Serial.println();
}

void printCsvHeader() {
    Serial.print("C,hPa,m,%,");
    Serial.print("C,hPa,m,%,");
    Serial.print(",,,m/s^2 ,m/s^2 ,m/s^2,,");
    Serial.println();
    Serial.print("internalTemp,internalPres,internalAlt,internalHum,");
    Serial.print("externalTemp,externalPres,externalAlt,externalHum,");
    Serial.print("x,y,z,xAccel,yAccel,zAccel,orientation,");
    Serial.println();
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
