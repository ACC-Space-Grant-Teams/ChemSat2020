/*
Code came from DS3231_test.pde
Jillian Frimml
3/17/2021

Test/demo of read routines for a DS3231 RTC.

Turn on the serial monitor after loading this to check if things are
working as they should.

Code should only print out the hour, minute, second, and temp of the sensor

*/

#include <DS3231.h>
#include <Wire.h>

DS3231 clock;
bool h12Flag;
bool pmFlag;

unsigned long delayTime;

void setup() {
  // Start the I2C interface
  Wire.begin();
 
  // Start the serial interface
  Serial.begin(9600);

  delay(2000);
  printCSVHeader();
}

void loop() {
  getRTCData();
  delay(delayTime);
}

void printCSVHeader() {
    Serial.print("Real,Time,Clock,C,");
    Serial.println();
    Serial.print("H,M,S,clockTemp,");
    Serial.println();
}

void getRTCData(){
  // send what's going on to the serial monitor.
  //Print the hour, minute, and second
  Serial.print(clock.getHour(h12Flag, pmFlag), DEC);
    Serial.print(",");
  Serial.print(clock.getMinute(), DEC);
    Serial.print(",");
  Serial.print(clock.getSecond(), DEC);
    Serial.print(",");

  // Display the temperature
  Serial.print(clock.getTemperature(), 2);
    Serial.println(",");
}
