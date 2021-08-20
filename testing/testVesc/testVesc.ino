/*
  Name:    getVescValues.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description:  This example is made using a Arduino Micro (Atmega32u4) that has a HardwareSerial port (Serial1) seperated from the Serial port. 
                A Arduino Nano or Uno that only has one Serial port will not be able to display the data returned.
*/

#include <VescUart.h>

/** Initiate VescUart class */
VescUart UART;

void setup() {
  Serial.begin(115200);
  /** Setup Serial port to display data */
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  delay(500);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  /** Setup UART port (Serial1 on Atmega32u4) */
  
  while (!Serial) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial);
}
boolean s = false;
void loop() {
  
  /** Call the function getVescValues() to acquire data from VESC */
  if ( UART.getVescValues() ) {

    digitalWrite(13, s);
    s = !s;
    delay(100);

  }
  else
  {
    Serial.println("Failed to get data!");
    digitalWrite(13, s);
    s = !s;
    delay(500);
  }

}
