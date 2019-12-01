/***************
 ______     ______     __  __     ______     ______   ______        ______     ______     __   __     ______   ______     ______     __         __         ______     ______    
/\  ___\   /\  ___\   /\ \/ /    /\  __ \   /\__  _\ /\  ___\      /\  ___\   /\  __ \   /\ "-.\ \   /\__  _\ /\  == \   /\  __ \   /\ \       /\ \       /\  ___\   /\  == \   
\ \  __\   \ \___  \  \ \  _"-.  \ \  __ \  \/_/\ \/ \ \  __\      \ \ \____  \ \ \/\ \  \ \ \-.  \  \/_/\ \/ \ \  __<   \ \ \/\ \  \ \ \____  \ \ \____  \ \  __\   \ \  __<   
 \ \_____\  \/\_____\  \ \_\ \_\  \ \_\ \_\    \ \_\  \ \_____\     \ \_____\  \ \_____\  \ \_\\"\_\    \ \_\  \ \_\ \_\  \ \_____\  \ \_____\  \ \_____\  \ \_____\  \ \_\ \_\ 
  \/_____/   \/_____/   \/_/\/_/   \/_/\/_/     \/_/   \/_____/      \/_____/   \/_____/   \/_/ \/_/     \/_/   \/_/ /_/   \/_____/   \/_____/   \/_____/   \/_____/   \/_/ /_/ 
                                                                                                                                                                                                                                                                                             
****************

  By Aaron Becker
  V1 Nov/Dec 2019
*/

#include <nRF24L01.h>
#include <RF24.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h> //light sensor

#include "joystickHelper.h" //joystick library

#include <SPI.h>
#include <Wire.h>
#include "SSD1306Ascii.h" //Thank god for Bill Greiman who came up with a library that uses less memory
#include "SSD1306AsciiWire.h"

int MASTER_STATE = 0;
boolean throttleEnabled = false;
boolean boostEnabled = false;
boolean oldBoostEnabled = false;
int ledMode = -1;
int oldLedMode = -1;

//PIN DEFS

//Escs (from skateboard)
#define ESC_MIN 800
#define ESC_MAX 2000
#define THROTTLE_NONBOOST_MAX 70 //in percent
#define ESC_STOP (ESC_MIN+ESC_MAX)/2;

//Buzzer
#define BUZZER_PIN 9

//OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
SSD1306AsciiWire oled;

//TSL9521 Lux sensor
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
#define LUX_ENABLE_THRESHOLD 7

//Joystick pins/setup
#define JOYSTICK_X A1
#define JOYSTICK_Y A2
#define JOYSTICK_SW 6
joystickHelper joystick(JOYSTICK_X, JOYSTICK_Y, JOYSTICK_SW);
int joystickPrevPos = 0;

//Batt pins
#define VBATT A0

//LED & boost pins
#define BOOST_SW 3
unsigned long lastBoostDebounceTime = 0;
#define LED_1_SW 5
#define LED_2_SW 4
unsigned long lastLEDDebounceTime = 0;
int debounceDelay = 50;

//Radio pins/defs
RF24 radio(7, 8);
const byte addresses [][6] = {"00001", "00002"}; //write at addr 00001, read at addr 00002
//Send 6 bytes (because each int is 2 bytes) per rx/tx
/*Data structure:
First int is command number
Second int is value 1
Third int is value 2 (so you can send up to four bytes of data if you want)
*/
int dataRx[3];
int dataTx[3];
unsigned long prevHBMillis = 0;
const int HBInterval = 1000; //send a heartbeat every 1000ms
boolean radioListening = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Eskate controller setup begin");

  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_SW, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BOOST_SW, INPUT_PULLUP);
  pinMode(LED_1_SW, INPUT_PULLUP);
  pinMode(LED_2_SW, INPUT_PULLUP);
  Serial.println("Pin conf: ok");

  if (!tsl.begin()) {
    Serial.println("Lux sensor: failed :(");
    while(1){}
  }
  tsl.setGain(TSL2561_GAIN_16X); //enable high gain to retain good performance in the dark
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
  Serial.println("Lux sensor: ok");

  //Setup Wire lib
  Wire.begin();
  Wire.setClock(400000L);
  //Actually init display
  oled.begin(&Adafruit128x64, 0x3C); //connect to display
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.set2X();
  oled.println("EskateOS V1");
  oled.println("By:");
  oled.println("AaronBecker");
  oled.println();
  oled.println();
  oled.set1X();
  oled.println("Hope you enjoy :)");
  
  Serial.println("OLED display: ok");

  //Setup radio
  if (!radio.begin()) {
    Serial.println("Radio: failed :(");
    while(1){}
  }
  radio.setPALevel(RF24_PA_MAX); //max because we don't want to lose connection
  radio.setRetries(3,3); // delay, count
  //CONTROLLER Writes to addr 1, reads from addr 2
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]); //set address to recieve data
  radioTransmitMode();
  Serial.println("Setup radio: ok");

  delay(1500); //keep splash screen up for a bit
  transitionState(0); //make sure to call transitionState to update screen
}

void loop() {
  switch (MASTER_STATE) {
    case 0: //0 is waiting for board response because hb signals are being sent constantly
      radioRecieveMode();
      resetDataRx();
      if (radio.available()) {
        radio.read(&dataRx, sizeof(dataRx));
        if (dataRx[0] == 200) { //200 is "heartbeat" signal
          Serial.println("Got first heartbeat signal from board");
                  //Play a silly pitch
          tone(9, 880); //A5 note
          delay(200);
          /*tone(9, 262); //C4 note
          delay(200);*/
          noTone(9);
          transitionState(1);
        }
      }
      break;

    case 1: //Normal operation
      //Update joystick
      int bMaxThrott = (boostEnabled) ? 100 : THROTTLE_NONBOOST_MAX;
      int curPos = map(analogRead(JOYSTICK_X), 0, 880, -100, bMaxThrott);
      curPos = constrain(curPos, -100, bMaxThrott); //make sure we have an actual value of curPos
      if (!throttleEnabled || abs(curPos) < 5) { //use deadzone of 5%
        curPos = 0; //just set it to 0 if it's not enabled
      }
      if (curPos != joystickPrevPos) {
        int ppm = map(curPos, -100, 100, ESC_MIN, ESC_MAX);
        //Update display
        if (abs(curPos-joystickPrevPos) > 3) { //because display updates are kinda annoying, try to prevent as many as we can
          oledUpdateDisplay();
          Serial.print("JoyChgState:");
          Serial.println(curPos);
        }

        //Send position to board
        radioTransmitMode();
        resetDataTx();
        dataTx[0] = 1; //led update
        dataTx[1] = ppm;
        radio.write(&dataTx, sizeof(dataTx));

        joystickPrevPos = curPos;
      }

      //Update peripherals - lux sensor, boost switch and led mode switch with debouncing
      sensors_event_t event;
      tsl.getEvent(&event);
      if (event.light) { //make sure sensor isn't overloaded
        if (event.light < LUX_ENABLE_THRESHOLD) { //cool finger is covering sensor, let's go!
          throttleEnabled = true;
          // Serial.println("ThrottleEnable");
        } else {
          throttleEnabled = false;
          // Serial.println("ThrottleDisable");
        }
      }

      boostEnabled = !digitalRead(BOOST_SW);
      if (boostEnabled != oldBoostEnabled) {
        oledUpdateDisplay();
        Serial.print("BoostChgState:");
        Serial.println(boostEnabled);
        //No need to send - handled on the controller only

        oldBoostEnabled = boostEnabled;
      }

      unsigned long currentMillis = millis(); //this switch needs debouncing

      int LEDReading1 = !digitalRead(LED_1_SW); //because of input pullup, invert inputs (since it'll be pulled to ground if high)
      int LEDReading2 = !digitalRead(LED_2_SW);
      if ((ledMode == 2 && LEDReading2 && !LEDReading1) || (ledMode == 3 && LEDReading1 && !LEDReading2) || (ledMode == 0 && !LEDReading2 && !LEDReading1)) {
        lastLEDDebounceTime = currentMillis;
      }
      if ((currentMillis - lastLEDDebounceTime) > debounceDelay) { //it's been there longer than the 50ms, so just write it
        ledMode = (LEDReading1) ? 2 : (LEDReading2) ? 3 : 0; //switch states

        if (oldLedMode != ledMode) {
          //Update display
          oledUpdateDisplay();
          Serial.print("LEDChgState:");
          Serial.println(ledMode);

          //Now send the data since there's been an update
          radioTransmitMode();
          resetDataTx();
          dataTx[0] = 2; //led update
          dataTx[1] = ledMode;
          radio.write(&dataTx, sizeof(dataTx));

          oldLedMode = ledMode;
        }
      }
      break;
  }

  unsigned long currentMillis = millis();
  if (currentMillis-prevHBMillis >= HBInterval) { //every HBInterval ms send a new heartbeat to board
    radioTransmitMode();
    resetDataTx();
    dataTx[0] = 200;
    radio.write(&dataTx, sizeof(dataTx));
    prevHBMillis = currentMillis;
  }
}

void transitionState(int newState) {
  MASTER_STATE = newState;
  Serial.print("New state: ");
  Serial.println(newState);
  switch (newState) {
    case 0:
      oled.clear();
      oled.set2X();
      oled.println("Waiting for");
      oled.println("connection");
      oled.println("...");
      break;
    case 1:
      oledUpdateDisplay();
      break;
  }  
}

void oledUpdateDisplay() {
  oled.clear();
  oled.set2X();
  oled.println("________________");
  oled.set1X();
  oled.print("BOOST: ");
  oled.println((boostEnabled)?"Enabled":"Disabled");
  oled.print("LED Mode: ");
  oled.println((ledMode == 2) ? "Rainbow" : (ledMode == 3) ? "ChgThrott" : "Off");
  oled.set2X();
  oled.println("________________");
  oled.print(joystickPrevPos);
  oled.println("%");


//Old display update code:
  /*
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("BOOST: Disabled");
  display.setCursor(0,10);
  display.println("LED Mode: Disabled");
  display.drawLine(0,20,SCREEN_WIDTH,20,WHITE);
  display.setCursor(0,30);
  display.setTextSize(24);
  display.println("0%");
  display.display();

  display.fillRect(0,0, SCREEN_WIDTH, 10, BLACK); //clear display here
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("BOOST: "+(boostEnabled)?"Enabled":"Disabled");
  display.display();

  display.fillRect(0,10, SCREEN_WIDTH, 10, BLACK); //clear display here
  display.setTextSize(1);
  display.setCursor(0,10);
  display.println("LED Mode: "+ledMode);
  display.display();

  display.setCursor(0,30);
  display.fillRect(0,30, SCREEN_WIDTH, SCREEN_HEIGHT-30, BLACK); //clear display here
  display.setTextSize(24);
  display.println(curY+"%");
  display.display();*/
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

void resetDataRx() {
  dataRx[0] = 0;
  dataRx[1] = 0;
  dataRx[2] = 0;
}

void resetDataTx() {
  dataTx[0] = 0;
  dataTx[1] = 0;
  dataTx[2] = 0;
}
