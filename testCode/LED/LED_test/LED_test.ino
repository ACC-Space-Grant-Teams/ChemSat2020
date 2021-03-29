/***************************************************************************
* Example sketch for an LED
*
* Jillian Frimml
* 3/24/2021
* LED is connected to a 330kOhm resistor
* One end is connected to the input power, that is connected to the LED
* The other side of the LED is connected in to pin 12 directly
*
* 
***************************************************************************/

int LED = 12;

void setup() {
  //start up for the LED
  pinMode(LED, OUTPUT);

    for(int i = 0; i < 5; i++)
  {
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
  }
  
}

void loop() {
  //turn the LED on and off
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);

}
