/***************
 ______     ______     __  __     ______     ______   ______        ______     ______     __   __     ______   ______     ______     __         __         ______     ______    
/\  ___\   /\  ___\   /\ \/ /    /\  __ \   /\__  _\ /\  ___\      /\  ___\   /\  __ \   /\ "-.\ \   /\__  _\ /\  == \   /\  __ \   /\ \       /\ \       /\  ___\   /\  == \   
\ \  __\   \ \___  \  \ \  _"-.  \ \  __ \  \/_/\ \/ \ \  __\      \ \ \____  \ \ \/\ \  \ \ \-.  \  \/_/\ \/ \ \  __<   \ \ \/\ \  \ \ \____  \ \ \____  \ \  __\   \ \  __<   
 \ \_____\  \/\_____\  \ \_\ \_\  \ \_\ \_\    \ \_\  \ \_____\     \ \_____\  \ \_____\  \ \_\\"\_\    \ \_\  \ \_\ \_\  \ \_____\  \ \_____\  \ \_____\  \ \_____\  \ \_\ \_\ 
  \/_____/   \/_____/   \/_/\/_/   \/_/\/_/     \/_/   \/_____/      \/_____/   \/_____/   \/_/ \/_/     \/_/   \/_/ /_/   \/_____/   \/_____/   \/_____/   \/_____/   \/_/ /_/ 
                                                                                                                                                                                                                                                                                             
****************

  By Aaron Becker
  V2 Dec 2019/Jan 2020
*/

#include <nRF24L01.h>
#include <RF24.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h> //light sensor

#include <SPI.h>
#include <Wire.h>
#include "SSD1306Ascii.h" //Thank god for Bill Greiman who came up with a library that uses less memory
#include "SSD1306AsciiWire.h"

int MASTER_STATE = 0;
boolean throttleEnabled = false;
boolean oldThrottleEnabled = false;
boolean boostEnabled = false;
boolean oldBoostEnabled = false;
boolean updateDisplay = false;
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
unsigned long toneExpire = 0;
boolean toneActive = false;

//OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
SSD1306AsciiWire oled;
unsigned long prevDispUpdateMillis = 0;
int dispMinUpdate = 200; //minimum time between display updates in ms to make sure we don't update faster than what the screen can handle

//TSL9521 Lux sensor
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
#define LUX_ENABLE_THRESHOLD 20
int oldLux = -1;
unsigned long prevLuxUpdateMillis = 0;
int luxMinUpdate = 200;

//Joystick pins/setup
#define JOYSTICK_X A1
#define JOYSTICK_Y A2
#define JOYSTICK_SW 6
int hallPrevPos = 0;
unsigned long lastJoySWDebounceTime = 0;

//Batt pins
#define VBATT A0

//LED & boost pins
#define BOOST_SW 3
unsigned long lastBoostDebounceTime = 0;
#define LED_1_SW 5
#define LED_2_SW 4
unsigned long lastLEDDebounceTime = 0;
int debounceDelay = 50;

//Vesc data
struct VREALTIME {
  float speed;
  float distanceTravelled;
  float temp;
  float inputVoltage;
  float battPercent;
};
struct VREALTIME vesc_values_realtime;

//Radio pins/defs
RF24 radio(7, 8);
const byte addresses [][6] = {"00001", "00002"}; //write at addr 00001, read at addr 00002
//Send 6 bytes (because each int is 2 bytes) per rx/tx
/*Data structure:
First int is command number
Second int is value 1
Third int is value 2 (so you can send up to four bytes of data if you want)
*/

/* RADIO COMMANDS MASTER
ID 200: heartbeat. Data: [0, 0]
ID 1: ThrottleVal update. Data: [raw value, 0]
ID 2: ThrottleSW update. Data: [sw, 0]
ID 3: LED mode update. Data: [ledMode (0, 1, 2), 0]
ID 4: BOOST switch update. Data: [boostMode (0, 1), 0]
ID 5: lux sensor update. Data: [luxVal, passingEnableThreshold (0, 1)]

ID 10: Ask controller to send state of all peripherals
ID 11: Controller force screen update
ID 12: VESC data [id, value]. ID 0 is speed, ID 1 is distance travelled, ID 2 is input voltage, ID 3 is fet temp, ID 4 is batt percent
ID 13: Play tone
*/

typedef enum {
  HEARTBEAT = 200,

  //Controller -> Board
  THROTTLE_VAL = 1,
  THROTTLE_SW = 2,
  LEDMODE = 3,
  BOOSTMODE = 4,
  LUX_VAL = 5,

  //Board -> Controller
  SENDALLDATA = 10,
  SCREENUPDATE = 11,
  VESCDATA = 12,
  TONE = 13
} RADIO_COMMANDS;

double dataRx[3];
double dataTx[3];
unsigned long prevHBMillis = 0;
const int HBInterval = 125; //send a heartbeat every 125ms, 8x per second
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
  unsigned long currentMillis = millis(); //store millis value (current value) for reference

  switch (MASTER_STATE) {
    case 0: //0 is waiting for board response because hb signals are being sent constantly
      radioRecieveMode();
      resetDataRx();
      if (radio.available()) {
        radio.read(&dataRx, sizeof(dataRx));
        if (dataRx[0] == 200) { //200 is "heartbeat" signal
          Serial.println("Got first heartbeat signal from board");
          //Play a silly pitch
          asynchTone(880, 200); //tone, time
          transitionState(1);
        }
      }
      break;

    case 1: //Normal operation
      //Update hall effect sensor
      
      if (!throttleEnabled || abs(curPos-127) < 5) { //use deadzone of 5%
        throttlePos = 127; //just set it to 0 if it's not enabled or within deadzone
      }
      if (throttlePos != hallPrevPos) {
        //Update display
        if (abs(throttlePos-hallPrevPos) > 3) { //because display updates are kinda annoying, try to prevent as many as we can. make sure difference is at least 3%
          updateDisplay = true; //set display update flag for next timer cycle
          Serial.print("HallChgState:");
          Serial.println(throttlePos);
        }

        //Send position to board
        radioTransmitMode();
        resetDataTx();
        dataTx[0] = THROTTLE_VAL; //throttle update
        dataTx[1] = curPos;
        radio.write(&dataTx, sizeof(dataTx));

        hallPrevPos = curPos;
      }

      //Update peripherals - lux sensor, boost switch and led mode switch with debouncing
      sensors_event_t event;
      tsl.getEvent(&event);
      if (event.light) { //make sure sensor isn't overloaded
        if (oldLux != event.light && (currentMillis - prevLuxUpdateMillis) > luxMinUpdate) { //send lux to board if min time has elapsed
          radioTransmitMode();
          resetDataTx();
          dataTx[0] = LUX_VAL; //lux update
          dataTx[1] = event.light;
          dataTx[2] = (event.light < LUX_ENABLE_THRESHOLD) ? 1 : 0;
          radio.write(&dataTx, sizeof(dataTx));

          prevLuxUpdateMillis = currentMillis;
          oldLux = event.light;
        }
      }

      throttleEnabled = !digitalRead(JOYSTICK_SW); //because of input pullup, invert inputs (since it'll be pulled to ground if high)
      if (throttleEnabled != oldThrottleEnabled) {
        lastJoySWDebounceTime = currentMillis;
      }
      if ((currentMillis - lastJoySWDebounceTime) > debounceDelay) { //give it time to settle
        if (throttleEnabled != oldThrottleEnabled) { //Ensure it's still actually different
          updateDisplay = true; //set display update flag for next timer cycle
          Serial.print("ThrottleSWState:");
          Serial.println(throttleEnabled);
          
          //Now send the data since there's been an update
          radioTransmitMode();
          resetDataTx();
          dataTx[0] = THROTTLE_SW; //sw update
          dataTx[1] = (throttleEnabled) ? 1 : 0;
          radio.write(&dataTx, sizeof(dataTx));

          oldThrottleEnabled = throttleEnabled;
        }
      }

      boostEnabled = !digitalRead(BOOST_SW); //because of input pullup, invert inputs (since it'll be pulled to ground if high)
      if (boostEnabled != oldBoostEnabled) {
        lastBoostDebounceTime = currentMillis;
      }
      if ((currentMillis - lastBoostDebounceTime) > debounceDelay) { //give it time to settle
        if (boostEnabled != oldBoostEnabled) { //Ensure it's still actually different
          updateDisplay = true; //set display update flag for next timer cycle
          Serial.print("BoostChgState:");
          Serial.println(boostEnabled);
          
          //Now send the data since there's been an update
          radioTransmitMode();
          resetDataTx();
          dataTx[0] = BOOSTMODE; //boost update
          dataTx[1] = (boostEnabled) ? 1 : 0;
          radio.write(&dataTx, sizeof(dataTx));

          oldBoostEnabled = boostEnabled;
        }
      }

      int LEDReading1 = !digitalRead(LED_1_SW); //because of input pullup, invert inputs (since it'll be pulled to ground if high)
      int LEDReading2 = !digitalRead(LED_2_SW);
      if ((ledMode == 2 && LEDReading2 && !LEDReading1) || (ledMode == 3 && LEDReading1 && !LEDReading2) || (ledMode == 0 && !LEDReading2 && !LEDReading1)) {
        lastLEDDebounceTime = currentMillis;
      }
      if ((currentMillis - lastLEDDebounceTime) > debounceDelay) { //it's been there longer than the 50ms, so just write it
        ledMode = (LEDReading1) ? 2 : (LEDReading2) ? 3 : 0; //switch states

        if (oldLedMode != ledMode) {
          //Update display
          updateDisplay = true; //set display update flag for next timer cycle
          Serial.print("LEDChgState:");
          Serial.println(ledMode);

          //Now send the data since there's been an update
          radioTransmitMode();
          resetDataTx();
          dataTx[0] = LEDMODE; //led update
          dataTx[1] = ledMode;
          radio.write(&dataTx, sizeof(dataTx));

          oldLedMode = ledMode;
        }
      }
      break;
  }

  if (currentMillis-prevHBMillis >= HBInterval) { //every HBInterval ms send a new heartbeat to board
    radioTransmitMode();
    resetDataTx();
    dataTx[0] = HEARTBEAT;
    radio.write(&dataTx, sizeof(dataTx));
    prevHBMillis = currentMillis;
  }

  radioRecieveMode(); //Check for any data from the board
  if (radio.available()) {
    resetDataRx();
    radio.read(&dataRx, sizeof(dataRx));
    switch (dataRx[0]) {
      case SENDALLDATA:
        radioTransmitMode();
        resetDataTx();
        dataTx[0] = HEARTBEAT;
        radio.write(&dataTx, sizeof(dataTx));
        resetDataTx();
        dataTx[0] = LEDMODE; //led update
        dataTx[1] = ledMode;
        radio.write(&dataTx, sizeof(dataTx));
        resetDataTx();
        dataTx[0] = BOOSTMODE; //boost update
        dataTx[1] = (boostEnabled) ? 1 : 0;
        radio.write(&dataTx, sizeof(dataTx));
        resetDataTx();
        dataTx[0] = THROTTLE_SW; //sw update
        dataTx[1] = (throttleEnabled) ? 1 : 0;
        radio.write(&dataTx, sizeof(dataTx));
        resetDataTx();
        dataTx[0] = LUX_VAL; //lux update
        dataTx[1] = oldLux;
        dataTx[2] = (oldLux < LUX_ENABLE_THRESHOLD);
        radio.write(&dataTx, sizeof(dataTx));
        resetDataTx();
        dataTx[0] = THROTTLE_VAL; //throttle update
        dataTx[1] = hallPrevPos;
        radio.write(&dataTx, sizeof(dataTx));
        break;
      case SCREENUPDATE:
        updateDisplay = true; //set flag so as not to refresh too fast
        break;
      case VESCDATA: //ID 0 is speed, ID 1 is distance travelled, ID 2 is input voltage, ID 3 is fet temp, ID 4 is batt percent
        switch (dataRx[1]) {
          case 0:
            vesc_values_realtime.speed = dataRx[2];
            break;
          case 1:
            vesc_values_realtime.distanceTravelled = dataRx[2];
            break;
          case 2:
            vesc_values_realtime.inputVoltage = dataRx[2];
            break;
          case 3:
            vesc_values_realtime.temp = dataRx[2];
            break;
          case 4:
            vesc_values_realtime.battPercent = dataRx[2];
            break;
        }
        break;
      case TONE:
        asynchTone(dataRx[1], dataRx[2]); //Tone, time
        break;

    }
  }

  //Perform display update if enough time has elapsed
  if (currentMillis-prevDispUpdateMillis >= dispMinUpdate && updateDisplay) {
    updateDisplay = false;
    prevDispUpdateMillis = currentMillis;
    
    oledUpdateDisplay();
  }

  //End tone if it's expired
  if (currentMillis > toneExpire && toneActive) {
    noTone(BUZZER_PIN);
    toneActive = false;
  }

  radioRecieveMode(); //default to recieve mode so as to not drop transmissions
}

void asynchTone(int pitch, int time) { //time in ms
  //if (!toneActive) { //make sure tone has expired before playing a new one (DISABLED)
  tone(BUZZER_PIN, pitch); //A5 note
  toneExpire = millis()+time;
  toneActive = true;
  //}
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
  oled.print(hallPrevPos); //'recent enough' ig it's ok
  oled.println("%");
}

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
