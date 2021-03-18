/*
Test code for the OpenLog SD card reader on the Arduino Nano 33
Jillian Frimml
3/18/2021

Test code for the red SD card reader

SD card formated to FAT32, config file already set up on SD card

Pin numbers for the nano and SD card
SD:     BLK          GND      VCC           TXO     RXI     GRN
Nano:   not used     ground   3.3v or 5v    RX      TX      not used

*/

#include <Wire.h>

unsigned long delayTime;

void setup() {
    // Start the I2C interface
    Wire.begin();

    //Open serial communications
    //open the serial monitor
    Serial.begin(9600);
    //open the openlog serial sd card recording monitor
    Serial1.begin(9600);

    delay(2000);
    printCSVHeader();

    //to be deleted once all the code is put together
    delayTime = 1000;
}



void loop() {
    //print on the monitor
    Serial.println("Inside the loop");
    //print on the SD card
    Serial1.println("Inside the loop");
    delay(delayTime);
}

void printCSVHeader() {
    //print on the monitor
    Serial.print("Real,Time,Clock,C,");
    Serial.print("C,hPa,m,%,");
    Serial.print("C,hPa,m,%,");
    Serial.print("rawData,rawData,rawData,m/s^2 ,m/s^2 ,m/s^2,facing,");
    Serial.println();
    Serial.print("H,M,S,clockTemp,");
    Serial.print("internalTemp,internalPres,internalAlt,internalHum,");
    Serial.print("externalTemp,externalPres,externalAlt,externalHum,");
    Serial.print("x,y,z,xAccel,yAccel,zAccel,orientation,");
    Serial.println();
    //print to the SD card
    Serial1.print("Real,Time,Clock,C,");
    Serial1.print("C,hPa,m,%,");
    Serial1.print("C,hPa,m,%,");
    Serial1.print("rawData,rawData,rawData,m/s^2 ,m/s^2 ,m/s^2,facing,");
    Serial1.println();
    Serial1.print("H,M,S,clockTemp,");
    Serial1.print("internalTemp,internalPres,internalAlt,internalHum,");
    Serial1.print("externalTemp,externalPres,externalAlt,externalHum,");
    Serial1.print("x,y,z,xAccel,yAccel,zAccel,orientation,");
    Serial1.println();
}
