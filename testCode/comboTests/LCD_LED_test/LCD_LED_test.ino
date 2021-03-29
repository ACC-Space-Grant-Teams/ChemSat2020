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

//LED pin number
int LED = 12;

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,20,4);

void setup() {
  //start up for the LED
  pinMode(LED, OUTPUT);

  // initialize the lcd 
  lcd.init();         
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("Hello, world!");
  for(int i = 0; i < 5; i++)
  {
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
  }

  Serial.begin(9600);
  while(!Serial){
        ; // wait for serial port to connect. Needed for native USB port only
  }
  lcd.setCursor(2,1);
  lcd.print("Sytem ready");

  delay(1000);
  lcd.clear();

}

void loop() {
  printLCDScreen();
}

void printLCDScreen(){
  Serial.println("Printing out to LCD screen...");
  blinkLED();
  lcd.setCursor(0,0);
  lcd.print("Printing loop");
  blinkLED();
  lcd.setCursor(0,1);
  lcd.print("second line");
  blinkLED();
  lcd.setCursor(2,2);
  lcd.print("third line, 2 over");
  blinkLED();
  lcd.setCursor(4,3);
  lcd.print("4th line, 4 over");
  blinkLED();
  delay(1000);
}

void blinkLED(){
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
}
