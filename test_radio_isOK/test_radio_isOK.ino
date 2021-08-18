#include <SPI.h>
#include "printf.h"
#include "RF24.h"          //nRF2401 libarary found at https://github.com/tmrh20/RF24/


const int pinCE = 0;
const int pinCSN = 1;
const int pinCS = 4;

RF24 wirelessSPI(pinCE, pinCSN);
 
const uint64_t pAddress = 0xB00B1E5000LL;           


int delayT = 100;
void setup() 
{
  Serial.begin(115200); 
  while (!Serial){}
  //delay(5000);
  Serial.println("RadioTest_Start");
  printf_begin();
  digitalWrite(pinCS, LOW);
  if (!wirelessSPI.begin()) {
    Serial.println("Radio is having an issue (SPIfail) :(");      
  }
  wirelessSPI.setAutoAck(1);         
  wirelessSPI.enableAckPayload();             
  wirelessSPI.setRetries(5,15);                 
  wirelessSPI.openWritingPipe(pAddress);       
  wirelessSPI.stopListening();
  wirelessSPI.printDetails();  

  Serial.println("\n\n\n");
  wirelessSPI.setDataRate(RF24_2MBPS);
  delay(100);
  if (wirelessSPI.getDataRate() == RF24_2MBPS) { //is the readback right?
    delayT = 1000;
    Serial.println("Radio OK");
  } else {
    delayT = 50;
    Serial.println("Radio not working");
  }

  pinMode(13, OUTPUT); //bye bye SPI communication
}


void loop() 
{ 
  digitalWrite(13, HIGH);
  delay(delayT);
  digitalWrite(13, LOW);
  delay(delayT);
}
