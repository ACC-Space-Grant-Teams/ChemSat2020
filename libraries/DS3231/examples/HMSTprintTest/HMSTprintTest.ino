
DS3231 clock;
bool century = false;
bool h12Flag;
bool pmFlag;
byte alarmDay, alarmHour, alarmMinute, alarmSecond, alarmBits;
bool alarmDy, alarmH12Flag, alarmPmFlag;

void setup() {
  // Start the I2C interface
  Wire.begin();

  // Start the serial interface
  Serial.begin(9600);
}

void loop() {
  // Finally the hour, minute, and second
  Serial.print("H: ")
  Serial.print(clock.getHour(h12Flag, pmFlag), DEC);
  Serial.print("M: ");
  Serial.print(clock.getMinute(), DEC);
  Serial.print("S: ");
  Serial.print(clock.getSecond(), DEC);

  // Display the temperature
  Serial.print("T=");
  Serial.print(clock.getTemperature(), 2);
}
