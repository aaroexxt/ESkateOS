#include <SPI.h>
#include "printf.h"
    #include <nRF24L01.h>
    #include <RF24.h>
    RF24 radio(0, 1); // CE, CSN
    const byte addresses [][6] = {"00001", "00002"}; //write at addr 00002, read at addr 00001
    void setup() {
    Serial.begin(9600);
    printf_begin();
    radio.begin();
    radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]); //set address to recieve data   //Setting the address at which we will receive the data
    radio.setPALevel(RF24_PA_MAX);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
    radio.stopListening();              //This sets the module as receiver
    radio.printDetails();
    Serial.println("RadioTest2 starting");
    }
    int count = 0;
    void loop()
    {
      int dataTx[3] = {count, 0, 0};
      radio.write(&dataTx, sizeof(dataTx)); //send one back
      Serial.print("ping: ");
      Serial.println(count);
      delay(100);
      count++;
    }
