#include <SPI.h>
#include "printf.h"
    #include <nRF24L01.h>
    #include <RF24.h>
    RF24 radio(7,8); // CE, CSN
    const byte addresses [][6] = {"00001", "00002"}; //write at addr 00002, read at addr 00001
    void setup() {
      delay(200);
    Serial.begin(9600);
    printf_begin();
    radio.begin();
    radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]); //set address to recieve data   //Setting the address at which we will receive the data
    radio.setPALevel(RF24_PA_LOW);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
    radio.startListening();              //This sets the module as receiver
    radio.printDetails();
    Serial.println("RadioTest starting");
    }
    void loop()
    {
      //digitalWrite(5, LOW);
    if (radio.available())              //Looking for the data.
    {
    //digitalWrite(5, HIGH);
    int dataRx[3];                 //Saving the incoming data
    radio.read(&dataRx, sizeof(dataRx));    //Reading the data
    Serial.print("Data0: ");
    Serial.print(dataRx[0]);
    Serial.print("Data1: ");
    Serial.print(dataRx[1]);
    Serial.print("Data2: ");
    Serial.println(dataRx[2]);

//    if (dataRx[0] == 200) { //200 is "heartbeat" signal
//      Serial.println("Got first heartbeat signal from controller");
//
//      radioTransmitMode();
//      resetDataTx();
//      dataTx[0] = 200;
//      boolean ack = radio.write(&dataTx, sizeof(dataTx));
//      if (ack) { //did transmission go through?
//        Serial.println("hb ack first signal");
//        prevHBMillis = millis();
//        transitionState(1);
//      }
//
//    }
    delay(5);
    }}
