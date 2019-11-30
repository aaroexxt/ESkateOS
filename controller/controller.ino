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

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h> //light sensor

#include "joystickHelper.h" //joystick library

int MASTER_STATE = 0;
boolean throttleEnabled = false;
boolean boostEnabled = false;
int ledMode = -1;

//PIN DEFS

//Escs (from skateboard)
#define ESC_MIN 800
#define ESC_MAX 2000
#define ESC_NONBOOST_MAX 1600
#define ESC_STOP (ESC_MIN+ESC_MAX)/2;

//Buzzer
#define BUZZER_PIN 9

//OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//TSL9521 Lux sensor
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
#define LUX_ENABLE_THRESHOLD 300

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
boolean radioListening = true;

void setup() {
  Serial.begin(57600);
  Serial.println("Eskate controller setup begin");

  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_SW, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BOOST_SW, INPUT_PULLUP);
  pinMode(LED_1_SW, INPUT_PULLUP);
  pinMode(LED_2_SW, INPUT_PULLUP);
  Serial.println("Pin conf: ok");

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println("OLED display: failed :(");
    while(1){}
  }
  display.clearDisplay();
  display.setTextSize(24);
  display.setCursor(SCREEN_WIDTH/2-5, 0);
  display.println("A");
  display.display();
  
  Serial.println("OLED display: ok");

  if (!tsl.begin()) {
    Serial.println("Lux sensor: failed :(");
    while(1){}
  }
  tsl.setGain(TSL2561_GAIN_16X); //enable high gain to retain good performance in the dark
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
  Serial.println("Lux sensor: ok");

  //Setup radio
  radio.begin();
  radio.setPALevel(RF24_PA_MAX); //max because we don't want to lose connection
  radio.setRetries(3,3); // delay, count
  //CONTROLLER Writes to addr 1, reads from addr 2
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]); //set address to recieve data
  radioTransmitMode();
  Serial.println("Setup radio: ok");

  //Play a silly pitch
  tone(9, 262); //C4 note
  delay(200);
  tone(9, 880); //A5 note
  delay(200);
  noTone(9);
  delay(600);

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
          transitionState(1);
        }
      }

    case 1: //Normal operation
      //Update joystick
      joystickPosition pos = joystick.getPosition();
      int curY = pos.y;
      if (!throttleEnabled) {
        curY = 0; //just set it to 0 if it's not enabled
      }
      if (curY != joystickPrevPos) {
        int ppm = map(curY, -100, 100, ESC_MIN, (boostEnabled) ? ESC_MAX : ESC_NONBOOST_MAX);
        //Update display
        display.setCursor(0,30);
        display.fillRect(0,30, SCREEN_WIDTH, SCREEN_HEIGHT-30, BLACK); //clear display here
        display.setTextSize(24);
        display.println(curY+"%");

        //Send position to board
        radioTransmitMode();
        resetDataTx();
        dataTx[0] = 1; //led update
        dataTx[1] = ppm;
        radio.write(&dataTx, sizeof(dataTx));

        joystickPrevPos = curY;
      }

      //Update peripherals - lux sensor, boost switch and led mode switch with debouncing
      sensors_event_t event;
      tsl.getEvent(&event);
      if (event.light) { //make sure sensor isn't overloaded
        if (event.light < LUX_ENABLE_THRESHOLD) { //cool finger is covering sensor, let's go!
          throttleEnabled = true;
        } else {
          throttleEnabled = false;
        }
      }

      unsigned long currentMillis = millis();
      int boostReading = digitalRead(BOOST_SW);
      if (boostReading != boostEnabled) {
        lastBoostDebounceTime = currentMillis;
      }
      if ((currentMillis - lastBoostDebounceTime) > debounceDelay) { //it's been there longer than the 50ms, so just write it
        boostEnabled = boostReading;
        display.fillRect(0,0, SCREEN_WIDTH, 10, BLACK); //clear display here
        display.setTextSize(1);
        display.setCursor(0,0);
        display.println("BOOST: "+(boostEnabled)?"Enabled":"Disabled");
        //No need to send - handled on the controller only
      }

      int LEDReading1 = !digitalRead(LED_1_SW); //because of input pullup, invert inputs (since it'll be pulled to ground if high)
      int LEDReading2 = !digitalRead(LED_2_SW);
      if ((ledMode == 2 && LEDReading2) || (ledMode == 3 && LEDReading1) || (ledMode == 1 && !LEDReading2 && !LEDReading1)) {
        lastLEDDebounceTime = currentMillis;
      }
      if ((currentMillis - lastLEDDebounceTime) > debounceDelay) { //it's been there longer than the 50ms, so just write it
        ledMode = (LEDReading1) ? 2 : (LEDReading2) ? 3 : 1; //switch states

        //Update display
        display.fillRect(0,10, SCREEN_WIDTH, 10, BLACK); //clear display here
        display.setTextSize(1);
        display.setCursor(0,10);
        display.println("LED Mode: "+ledMode);

        //Now send the data since there's been an update
        radioTransmitMode();
        resetDataTx();
        dataTx[0] = 2; //led update
        dataTx[1] = ledMode;
        radio.write(&dataTx, sizeof(dataTx));
      }
  }

  unsigned long currentMillis = millis();
  if (currentMillis-prevHBMillis >= HBInterval) { //every HBInterval ms send a new heartbeat to board
    radioTransmitMode();
    resetDataTx();
    dataTx[0] = 200;
    radio.write(&dataTx, sizeof(dataTx));
  }
}

void transitionState(int newState) {
  MASTER_STATE = newState;
  Serial.println("New state: "+newState);
  switch (newState) {
    case 0:
      display.clearDisplay();
      display.drawRoundRect(0, SCREEN_HEIGHT/2, SCREEN_WIDTH, SCREEN_HEIGHT/2, 2, WHITE);
      display.setTextSize(1);
      display.setCursor(0, SCREEN_HEIGHT/2);
      display.println("Waiting for connection...");
      display.display();
      break;
    case 1:
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
      break;
  }  
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
