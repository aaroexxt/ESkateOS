/***************

 ______     ______     __  __     ______     ______   ______        ______     ______     __   __     ______   ______     ______     __         __         ______     ______    
/\  ___\   /\  ___\   /\ \/ /    /\  __ \   /\__  _\ /\  ___\      /\  ___\   /\  __ \   /\ "-.\ \   /\__  _\ /\  == \   /\  __ \   /\ \       /\ \       /\  ___\   /\  == \   
\ \  __\   \ \___  \  \ \  _"-.  \ \  __ \  \/_/\ \/ \ \  __\      \ \ \____  \ \ \/\ \  \ \ \-.  \  \/_/\ \/ \ \  __<   \ \ \/\ \  \ \ \____  \ \ \____  \ \  __\   \ \  __<   
 \ \_____\  \/\_____\  \ \_\ \_\  \ \_\ \_\    \ \_\  \ \_____\     \ \_____\  \ \_____\  \ \_\\"\_\    \ \_\  \ \_\ \_\  \ \_____\  \ \_____\  \ \_____\  \ \_____\  \ \_\ \_\ 
  \/_____/   \/_____/   \/_/\/_/   \/_/\/_/     \/_/   \/_____/      \/_____/   \/_____/   \/_/ \/_/     \/_/   \/_/ /_/   \/_____/   \/_____/   \/_____/   \/_____/   \/_/ /_/ 
                                                                                                                                                                                
****************


  Heavily adapted from ElectroNoobs ESC Controller by Aaron Becker
  EbikeOS by Aaron Becker. Let's get it
  V1 May/Jun 2019, V2 Oct/Nov 2019
*/

#include <ServoTimer2.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <FastLED.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//PIN DEFS

//Esc pins
ServoTimer2 ESC_LEFT; //Create FSESC "servo" output
ServoTimer2 ESC_RIGHT; //Create FSESC "servo" output
#define ESC_R_PIN 5
#define ESC_L_PIN 6

#define ESC_MIN 800
#define ESC_MAX 2000
#define ESC_STOP (ESC_MIN+ESC_MAX)/2;
float realPPM = ESC_STOP;
float targetPPM = ESC_STOP;

//Relay pins
#define RELAY_PIN0 9
#define RELAY_PIN1 10

//Led pins/defs
#define LED_DATA_PIN    3
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    64
CRGB leds[NUM_LEDS];
#define LED_BRIGHTNESS 96
#define LED_FPS 120
uint8_t gHue = 0; // rotating "base color"
int ledPosition = 0; //current position in strip for pattern
int ledState = 0;
/*
0 initial or disconnect leds (chasing blue)
1 rainbow
2 follow throttle
*/

//Radio pins/defs
RF24 radio(7, 8);
const byte addresses [][6] = {"00001", "00002"}; //write at addr 00002, read at addr 00001
//Send 6 bytes (because each int is 2 bytes) per rx/tx
/*Data structure:
First int is command number
Second int is value 1
Third int is value 2 (so you can send up to four bytes of data if you want)
*/
int dataRx[3];
int dataTx[3];
unsigned long prevHBMillis = 0;
const int HBTimeoutMax = 4000; //max time between signals before board cuts the motors
boolean radioListening = true;

//IMU pins/defs
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
const char ACCEL_AXIS = 'x'; //axis that board accelerates along; used for accel math

//General pins/defs
int MASTER_STATE = 0;

void setup() {
  Serial.begin(57000);
  Serial.println("ESKATEINIT_setup begin");
  delay(1000);
  //Setup ESC
  ESC_LEFT.attach(ESC_L_PIN);
  ESC_RIGHT.attach(ESC_R_PIN);
  ESC_LEFT.write(realPPM); //set them to be basically off (middle position)
  ESC_RIGHT.write(realPPM);
  Serial.println("Setup esc: ok");

  //Setup LEDS
  FastLED.addLeds<LED_TYPE,LED_DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear();
  for (int i=0; i<NUM_LEDS; i++) {
    leds[i] = CRGB:Black; //set all leds to be off
  }
  FastLED.show();
  Serial.println("Setup leds: ok");

  //Setup accelerometer
  if (!accel.begin()) {
    Serial.println("Setup accel: fail. not detected :(");
    while(1){}
  }
  accel.setRange(LSM303_RANGE_4G);
  accel.setMode(LSM303_MODE_NORMAL);
  Serial.println("Setup accel: ok");

  //Setup radio
  radio.begin();
  radio.setPALevel(RF24_PA_MAX); //max because we don't want to lose connection
  radio.setRetries(3,3); // delay, count
  //SKATEBOARD Writes to addr 2, reads from addr 1
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]); //set address to recieve data
  radioRecieveMode();
  Serial.println("Setup radio: ok");



  pinMode(potentiometer, INPUT);
  ESC.attach(9);    
  Serial.begin(9600);  
  curval=0;
  ESC.setMinimumPulse(800);
  ESC.setMaximumPulse(2000);
}

void loop() {
  switch (MASTER_STATE) {
    case 0: //case 0 waiting for first hb signal from controller
      radioRecieveMode(); //ensure we're recieving
      dataRx = {0, 0, 0};
      if (radio.available()) {
        radio.read(&dataRx, sizeof(dataRx));
        if (dataRx[0] == 200) { //200 is "heartbeat" signal
          Serial.println("Got first heartbeat signal from controller");

          radioTransmitMode();
          dataTx = {200, 0, 0};
          boolean ack = radio.send(&dataTx, sizeof(dataTx));
          if (ack) { //did transmission go through?
            Serial.println("hb ack first signal");
            prevHBMillis = millis();
            transitionState(1);
          }

        }
      }
    case 1: //standard operation
      radioRecieveMode();
      if (radio.available()) {
        dataRx = {0, 0, 0};
        radio.read(&dataRx, sizeof(dataRx));
        Serial.println("Got event #: "+dataRx[0]+", value1: "+dataRx[1]+", value2: "+dataRx[2]);
        switch (dataRx[0]) {
          case 1: //1 is throttle update
            targetPPM = dataRx[1];
            break;
          case 2: //2 is led update
            if (dataRx[1] < 3) { //sanity check
              ledState = dataRx[1];
            }
            break;
          case 200: //heartbeat. if we get one, we should send one
            Serial.println("controller hb recieved");
            radioTransmitMode();
            dataTx = {200, 0, 0};
            boolean ack = radio.send(&dataTx, sizeof(dataTx));
            if (!ack) {
              transitionState(2); //state 2 is when we fail to get or send a heartbeat signal from the controller
            } else {
              prevHBMillis = millis();
            }
            break;
        }
      }
      break;
      case 2: //lost connection case
        radioRecieveMode(); //ensure we're recieving
        dataRx = {0, 0, 0};
        if (radio.available()) {
          radio.read(&dataRx, sizeof(dataRx));
          if (dataRx[0] == 200) { //200 is "heartbeat" signal
            Serial.println("Got heartbeat signal from controller after disconnection");

            radioTransmitMode();
            dataTx = {200, 0, 0};
            boolean ack = radio.send(&dataTx, sizeof(dataTx));
            if (ack) { //did transmission go through?
              Serial.println("hb ack reconnect");
              prevHBMillis = millis();
              transitionState(1); //go back to normal operation
            }
          }
        }
        break;
    default:
      Serial.println("Undefined state; resetting");
      transitionState(0);
  }

  switch (ledState) {
    case 0: //blue chase
      for (int i=0; i<NUM_LEDS; i++) {
        if (i == ledPosition) {
          leds[i] = CRGB::Blue;
        } else {
          leds[i] = CRGB::Black;
        }
      }
      ledPosition++;
      if (ledPosition > NUM_LEDS-1) {
        ledPosition = 0;
      }
      break;
    case 1: //rainbow
      fill_rainbow(leds, NUM_LEDS, CRGB::Black, 7);
      break;
    case 2: //color changes based on throttle


  }

  unsigned long currentMillis = millis();
  if (currentMillis-prevHBMillis>=HBTimeoutMax) { //have we lost the controller
    transitionState(2);
  }

  updateESC();
}


  //Sensor update code
  sensors_event_t event;
  accel.getEvent(&event);
  switch (ACCEL_AXIS) {
    case 'x':
      Serial.print("X: ");
      Serial.print(event.acceleration.x);
      break;
    case 'y':
      Serial.print("Y: ");
      Serial.print(event.acceleration.y);
      break;
    case 'z':
      Serial.print("Z: ");
      Serial.print(event.acceleration.z);
      break;
  }
  delay(500);




  potval=analogRead(potentiometer);
  potval=map(potval,0,1023,0,180);
  
  while(curval<potval){
    potval=analogRead(potentiometer);
    potval=map(potval,0,1023,0,180);
    curval=curval+1;
    ESC.write(curval);
    SoftwareServo::refresh();
    Serial.println(curval);
    delay(50);}

  while(curval>potval){
    potval=analogRead(potentiometer);
    potval=map(potval,0,1023,0,180);
    curval=curval-1;
    ESC.write(curval);
    SoftwareServo::refresh();
    Serial.println(curval);
    delay(50);}

    ESC.write(curval);
    SoftwareServo::refresh();
    Serial.println(curval);
}

void transitionState(int newState) {
  MASTER_STATE = newState;
  switch (newState) {
    case 2: //uhoh we are going into remote disconnect mode
      Serial.println("Uhoh we've lost connection to the remote :(");
      ledState = 0; //go back into disconnected mode
      targetPPM = ESC_STOP; //set target to 0 speed to bring us back down to 0 speed
      break;
  }  
}

void updateESCPercent(float percent) {
  if ()
}

void radioRecieveMode() {
  if (!radioListening) { //if we're not listening
    radio.startListening();
    radioListening = true;
  }
}

void radioTransmitMode() {
  if (radioListening) {
    radio.stopListening();
    radioListening = false;
  }
}
