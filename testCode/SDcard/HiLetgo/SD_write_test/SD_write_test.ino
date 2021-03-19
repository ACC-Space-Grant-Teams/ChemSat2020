/*
Test code for the SD card reader on the Arduino Nano 33 and the blue micro SD card reader, "HiLetgo Micro SD Carte TF Adater Reader"
Jillian Frimml
3/18/2021

Test code for the blue SD card reader
pin layout
SD:   MISO  MOSI  SCK   CS
Nano: D12   D11   D13   D10

original code found on https://create.arduino.cc/projecthub/electropeak/sd-card-module-with-arduino-how-to-read-write-data-37f390
*/

#include <SPI.h>
#include <SD.h>

File myFile;
const int chipSelect = 10;

unsigned long delayTime;

void setup() {
    // Open serial communications and wait for port to open:
    Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.print("Initializing SD card...");
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed or not present!");
        while (1);
    }
    Serial.println("card initialized.");

    delay(2000);
    String dataString = "";
    dataString += printCSVHeader();
    
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    myFile = SD.open("datalog.txt", FILE_WRITE);

    // if the file is available, write to it:
    if (myFile) {
      myFile.println(dataString);
      myFile.close();
      // print to the serial port too:
      Serial.println(dataString);
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }
    
    delayTime = 1000;
    
}

void loop() {
  // make a string for assembling the data to log:
  String dataString = "";

  //store the info in the string
    dataString += "Inside the loop";

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (myFile) {
    myFile.println(dataString);
    myFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

  delay(delayTime);
}

String printCSVHeader() {
    //print on the monitor
    String first = "";
    first += "Real,Time,Clock,C,";
    first += "C,hPa,m,%,";
    first += "C,hPa,m,%,";
    first += "rawData,rawData,rawData,m/s^2 ,m/s^2 ,m/s^2,facing,";
    first += "\n";
    first += "H,M,S,clockTemp,";
    first += "internalTemp,internalPres,internalAlt,internalHum,";
    first += "externalTemp,externalPres,externalAlt,externalHum,";
    first += "x,y,z,xAccel,yAccel,zAccel,orientation,";
    first += "\n";
    return first;
}
