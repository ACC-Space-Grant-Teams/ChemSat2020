#include <Adafruit_DS3502.h>

Adafruit_DS3502 ds3502_a = Adafruit_DS3502();

Adafruit_DS3502 ds3502_b = Adafruit_DS3502();

float resistance_a_max = 10000;
float wiper_a_max = 127;
float resistance_b_max = 10000;
float wiper_b_max = 127;

void setup() {
  Serial.begin(9600);
  // Wait until serial port is opened
  while (!Serial) { delay(1); }

  Serial.println("Adafruit DS3502 Test");

  if (!ds3502_a.begin(0x28, &Wire) || !ds3502_b.begin(0x29, &Wire)) {
    Serial.println("Couldn't find DS3502 chip");
    while (1);
  }
  Serial.println("Found DS3502 chip");
}

void loop() {
  float wiper_a = 0;
  float wiper_b = 0;
  float r1 = (resistance_a_max * wiper_a) / wiper_b_max
  float r2 = resistance_a_max - r1;
  float r3 = (resistance_b_max * wiper_b) / wiper_b_max
  float r4 = resistance_b_max - r3;
  float r_measurement = r2 + r4; // ignore r3 since we are taking our voltage measurement from the wiper pin
  float r_circuit = r2 + r3 + r4; // r1 isn't connected to anything, so no current can flow through it
  Serial.print("Wa: ");
  Serial.print(wiper_a);
  Serial.print(" R2: ");
  Serial.print(r2);
  Serial.print(" Wb: ");
  Serial.print(wiper_b);
  Serial.print(" R4: ");
  Serial.print(r4);
  Serial.print(" R_meas: ");
  Serial.print(r_measurement);
  
  ds3502_a.setWiper(wiper_a);
  ds3502_b.setWiper(wiper_b);
}
