/*
Code came from LiquidCrystal I2C example -> SerialDisplay
Jillian Frimml
3/19/2021

Displays text sent over the serial port (e.g. from the Serial Monitor) on
an attached LCD.
YWROBOT
Compatible with the Arduino IDE 1.0
Library version:1.1
 */
 
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  lcd.init();          // initialize the lcd 
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("Hello, world!");
  Serial.begin(9600);
  while(!Serial){
        ; // wait for serial port to connect. Needed for native USB port only
  }
  lcd.setCursor(2,1);
  lcd.print("Sytem ready");

  delay(10000);
  lcd.clear();

}

void loop() {
  Serial.println("Printing out to LCD screen...");
  lcd.setCursor(0,0);
  lcd.print("Printing loop");
  lcd.setCursor(0,1);
  lcd.print("second line");
  lcd.setCursor(2,2);
  lcd.print("third line, 2 over");
  lcd.setCursor(4,3);
  lcd.print("4th line, 4 over");
  delay(1000);

}
