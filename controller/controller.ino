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

#include <SPI.h>
#include <U8g2lib.h>
#include "printf.h"

int MASTER_STATE = 0;
boolean throttleEnabled = false;
boolean oldThrottleEnabled = false;
boolean boostEnabled = false;
boolean oldBoostEnabled = false;
boolean updateDisplayFlag = false;
int ledMode = -1;
int oldLedMode = -1;

//PIN DEFS

const bool displayVESCData = false;
const bool displaySENSData = false;

//Buzzer
#define BUZZER_PIN 9
unsigned long toneExpire = 0;
boolean toneActive = false;

//OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Defining the type of display used (128x64)
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); //No rotation
unsigned long prevDispUpdateMillis = 0;
int dispMinUpdate = 20; //minimum time between display updates in ms to make sure we don't update faster than what the screen can handle

typedef enum {
  DISPU_FULL = 0,
  DISPU_CONN_WAIT = 1,
  DISPU_START = 2,
  DISPU_SENSDATA = 3,
  CLEAR = 4,
  DISPU_VERSION = 5
} DISPLAY_UPDATE_TYPES;

String displayString;
char displayBuffer[20];
boolean displayStateOne = false; //boolean controls what data is displayed during normal display
boolean displaySensorData = false;
unsigned long lastDispStateOneTime = 0;
#define displayStateOneChangeTime 4000

//All bitmaps are in XBM format
const unsigned char logo_bits[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x3c, 0x01,
  0xe0, 0x00, 0x07, 0x70, 0x18, 0x0e, 0x30, 0x18, 0x0c, 0x98, 0x99, 0x19,
  0x80, 0xff, 0x01, 0x04, 0xc3, 0x20, 0x0c, 0x99, 0x30, 0xec, 0xa5, 0x37,
  0xec, 0xa5, 0x37, 0x0c, 0x99, 0x30, 0x04, 0xc3, 0x20, 0x80, 0xff, 0x01,
  0x98, 0x99, 0x19, 0x30, 0x18, 0x0c, 0x70, 0x18, 0x0e, 0xe0, 0x00, 0x07,
  0x80, 0x3c, 0x01, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char signal_transmitting_bits[] = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x0f,
  0x33, 0x0f, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char signal_connected_bits[] = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x09,
  0x33, 0x09, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char signal_noconnection_bits[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x09,
  0x00, 0x09, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

//Throttle pins/setup
#define HALLEFFECT A1
#define THROTT_ENABLE_SW 6
#define throttleDeadzone 4 //About 1.5% intrinsic deadzone, can be bigger on skateboard but want to minimize deadzone from controller side because it's harder to modify
#define THROTTLE_MIN 0
#define THROTTLE_MAX 255
#define THROTTLE_STOP (THROTTLE_MIN+THROTTLE_MAX)/2

//Hall middle 633
//Hall real low 481
#define HALL_MIN 500
#define HALL_MAX 1100
#define HALL_CENTER 700//(HALL_MIN+HALL_MAX)/2

int prevThrottle = THROTTLE_STOP;
int throttle = THROTTLE_STOP;

//Click tracking of enable switch
int ENclicks = 0;
unsigned long ENLastClickTime = 0;
#define ENClickTimeout 1000 //Max time to register clicks

//Batt pins
#define VBATT_PIN A0

//LED & boost pins
#define BOOST_SW 3
#define LED_1_SW 5
#define LED_2_SW 4
unsigned long lastLEDDebounceTime = 0;
#define debounceDelay 50

//Vesc data
struct VREALTIME {
  float speed;
  float distanceTravelled;
  float inputVoltage;
  float battPercent;
};
struct VREALTIME vesc_values_realtime;

//Gyro/accel/temp data
struct SREALTIME {
  float pitch;
  float roll;
  float heading;
  float acceleration;
  float temperature;
  float altitude;
};
struct SREALTIME sensor_values_realtime;

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
ID 5: CLICK event. Data: [clickCount (0, 1, ...)]

ID 10: Ask controller to send state of all peripherals
ID 11: Controller force screen update
ID 12: VESC data [id, value]. ID 0 is speed, ID 1 is distance travelled, ID 2 is input voltage, ID 3 is fet temp, ID 4 is batt percent
ID 13: Sensor data [id, value]. ID 0 is pitch, ID 1 is roll, ID 2 is heading, ID 3 is acceleration, ID 4 is temperature, ID 5 is altitude
ID 14: Play tone
*/

typedef enum {
  HEARTBEAT = 200,

  //Controller -> Board
  THROTTLE_VAL = 1,
  THROTTLE_SW = 2,
  LEDMODE = 3,
  BOOSTMODE = 4,
  CLICK = 5,

  //Board -> Controller
  SENDALLDATA = 10,
  SCREENUPDATE = 11,
  VESCDATA = 12,
  SENSDATA = 13,
  TONE = 14
} RADIO_COMMANDS;

double dataRx[3];
double dataTx[3];
unsigned long prevHBMillis = 0;
unsigned long prevRadioResendMillis = 0;
unsigned long lastHBTime = 0; //time when heartbeat signal was last recieved
#define HBInterval 125 //send a heartbeat every 125ms, 8x per second
#define radioResendInterval 250 //send all radio commands every 250ms, ~4x per secodn
#define HBTimeoutMax 750 //max time between signals before board cuts the motors in ms
boolean radioListening = false;
boolean connected = false; //check if connected
boolean oldConnected = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("Eskate controller setup begin"));

  pinMode(HALLEFFECT, INPUT_PULLUP);
  pinMode(THROTT_ENABLE_SW, INPUT_PULLUP);
  pinMode(VBATT_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BOOST_SW, INPUT_PULLUP);
  pinMode(LED_1_SW, INPUT_PULLUP);
  pinMode(LED_2_SW, INPUT_PULLUP);
  Serial.println(F("Pin conf: ok"));

  u8g2.begin(); //Initialize display
  
  Serial.println(F("OLED display: ok"));

  //Setup radio
  if (!radio.begin()) {
    Serial.println(F("Radio: failed :("));
    while(1){}
  }
  radio.setPALevel(RF24_PA_MAX); //max because we don't want to lose connection
  radio.setRetries(3,3); // delay, count
  //CONTROLLER Writes to addr 1, reads from addr 2
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]); //set address to recieve data
  radioTransmitMode();
  Serial.println(F("Radio details:"));
  printf_begin();
  radio.printDetails();
  Serial.println(F("Setup radio: ok"));

  updateDisplay(DISPU_START);
  Serial.println(F("Start screen going up"));
  asynchTone(3830, 100); //play a c note
  delay(100);
  asynchTone(3400, 100); //play a d note
  delay(100);
  asynchTone(3038, 100); //play a e note
  delay(1300); //keep splash screen up for a bit
  updateDisplay(DISPU_VERSION);
  delay(1000);
  transitionState(0); //make sure to call transitionState to update screen
}

void loop() {
  unsigned long currentMillis = millis(); //store millis value (current value) for reference

  //Check/update throttle state, since that should happen no matter what because of safety
  throttleEnabled = !digitalRead(THROTT_ENABLE_SW); //because of input pullup, invert inputs (since it'll be pulled to ground if high)
  if (throttleEnabled != oldThrottleEnabled) {
    updateDisplayFlag = true; //set display update flag for next timer cycle
    Serial.print(F("ThrottleEnChgState:"));
    Serial.println(throttleEnabled);

    if (throttleEnabled == HIGH) { //on rising state change
      if (ENclicks == 0) { //First click just set the last click time to currentMillis
        ENLastClickTime = currentMillis;
        ENclicks++;
      } else if (ENclicks > 0 && currentMillis < (ENLastClickTime + ENClickTimeout)) { //clicked within time interval
        ENclicks++;
        Serial.print(F("Clicked "));
        Serial.print(ENclicks);
        Serial.println(F(" times"));

        //Now send the data since there's been an update
        radioTransmitMode();
        resetDataTx();
        dataTx[0] = CLICK; //click event
        dataTx[1] = ENclicks;
        radio.write(&dataTx, sizeof(dataTx));

        if (ENclicks == 3) { //clicked 3 times
          if (MASTER_STATE == 0) { //If we're in state 0, triple click should transition to normal display (for testing reasons for example)
            transitionState(1);
          } else {
            updateDisplayFlag = true;
            displaySensorData = !displaySensorData; //toggle display sensor data state
          }
        }
      }
    }
    
    //Now send the data since there's been an update
    radioTransmitMode();
    resetDataTx();
    dataTx[0] = THROTTLE_SW; //sw update
    dataTx[1] = (throttleEnabled) ? 1 : 0;
    radio.write(&dataTx, sizeof(dataTx));

    oldThrottleEnabled = throttleEnabled;
  }

  if (currentMillis >= (ENLastClickTime + ENClickTimeout) && ENLastClickTime != 0) { //Reset click count; time interval expired
    ENclicks = 0;
    ENLastClickTime = 0;
  }

  switch (MASTER_STATE) {
    case 0: //0 is waiting for board response because hb signals are being sent constantly
      radioRecieveMode();
      resetDataRx();
      if (radio.available()) {
        radio.read(&dataRx, sizeof(dataRx));
        if (dataRx[0] == 200) { //200 is "heartbeat" signal
          Serial.println(F("Got first heartbeat signal from board"));
          connected = true; //set connected flag
          transitionState(1);
        }
      }
      break;

    case 1: //Normal operation
      //Update hall effect sensor
      int measurement = 0;
      for (int i=0; i<10; i++) { //take average reading over 10 samples to reduce noise
        measurement += analogRead(HALLEFFECT);
      }
      measurement /= 10;

      //Serial.println(measurement);

      if (measurement >= HALL_CENTER) { //if true, we're going forward = >127 value
        int forwardVal = map(measurement, HALL_CENTER, HALL_MAX, THROTTLE_STOP, THROTTLE_MAX); //map from middle to max (127-255)
        throttle = constrain(forwardVal, THROTTLE_STOP, THROTTLE_MAX); //make sure it's in range
      } else { //if false, we're going backward
        int backwardVal = map(measurement, HALL_MIN, HALL_CENTER, THROTTLE_MIN, THROTTLE_STOP); //map from min to middle (0, 127)
        throttle = constrain(backwardVal, THROTTLE_MIN, THROTTLE_STOP); //make sure it's in range
      }
      
      if (abs(throttle-THROTTLE_STOP) < throttleDeadzone) { //use deadzone of throttleDeadzone%
        throttle = THROTTLE_STOP; //just set it to 0 if it's not enabled or within deadzone
      }
      if (throttle != prevThrottle) {
        //Update display
        if (abs(throttle-prevThrottle) > 2) { //because display updates are kinda annoying, try to prevent as many as we can. make sure difference is at least 5
          updateDisplayFlag = true; //set display update flag for next timer cycle
          Serial.print(F("HallChgState:"));
          Serial.println(throttle);
        }

        //Send position to board
        radioTransmitMode();
        resetDataTx();
        dataTx[0] = THROTTLE_VAL; //throttle update
        dataTx[1] = throttle;
        radio.write(&dataTx, sizeof(dataTx));

        prevThrottle = throttle;
      }

      boostEnabled = !digitalRead(BOOST_SW); //because of input pullup, invert inputs (since it'll be pulled to ground if high)
      if (boostEnabled != oldBoostEnabled) {
        updateDisplayFlag = true; //set display update flag for next timer cycle
        Serial.print(F("BoostChgState:"));
        Serial.println(boostEnabled);
        
        //Now send the data since there's been an update
        radioTransmitMode();
        resetDataTx();
        dataTx[0] = BOOSTMODE; //boost update
        dataTx[1] = (boostEnabled) ? 1 : 0;
        radio.write(&dataTx, sizeof(dataTx));

        oldBoostEnabled = boostEnabled;
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
          updateDisplayFlag = true; //set display update flag for next timer cycle
          Serial.print(F("LEDChgState:"));
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

  radioRecieveMode(); //Check for any data from the board
  if (radio.available()) { //we have data available!
    boolean vvNew = false; //keep track of new data from vesc; did it actually arrive
    boolean ssNew = false; //keep track of new data from sensor; did it actually arrive

    resetDataRx();
    radio.read(&dataRx, sizeof(dataRx));
    switch ((int)dataRx[0]) {
      case HEARTBEAT: //board -> remote heartbeats
        lastHBTime = currentMillis;
        break;
      case SENDALLDATA:
        sendAllRadioCommands();
        break;
      case SCREENUPDATE:
        updateDisplayFlag = true; //set flag so as not to refresh too fast
        break;
      case VESCDATA: //ID 0 is speed, ID 1 is distance travelled, ID 2 is input voltage, ID 3 is batt percent
        switch ((int)dataRx[1]) {
          case 0:
            vesc_values_realtime.speed = dataRx[2];
            vvNew = true;
            break;
          case 1:
            vesc_values_realtime.distanceTravelled = dataRx[2];
            vvNew = true;
            break;
          case 2:
            vesc_values_realtime.inputVoltage = dataRx[2];
            vvNew = true;
            break;
          case 3:
            vesc_values_realtime.battPercent = dataRx[2];
            vvNew = true;
            break;
        }
        break;
      case SENSDATA: //ID 13: Sensor data [id, value]. ID 0 is pitch, ID 1 is roll, ID 2 is heading, ID 3 is acceleration, ID 4 is temperature, ID 5 is altitude
        switch ((int)dataRx[1]) {
          case 0:
            sensor_values_realtime.pitch = dataRx[2];
            ssNew = true;
            break;
          case 1:
            sensor_values_realtime.roll = dataRx[2];
            ssNew = true;
            break;
          case 2:
            sensor_values_realtime.heading = dataRx[2];
            ssNew = true;
            break;
          case 3:
            sensor_values_realtime.acceleration = dataRx[2];
            ssNew = true;
            break;
          case 4:
            sensor_values_realtime.temperature = dataRx[2];
            ssNew = true;
            break;
          case 5:
            sensor_values_realtime.altitude = dataRx[2];
            ssNew = true;
            break;
        }
        break;
      case TONE:
        asynchTone(dataRx[1], dataRx[2]); //Tone, time
        break;

    }
    if (vvNew && displayVESCData) {
      Serial.print(F("New VESC data recieved\nSpeed: "));
      Serial.println(String(vesc_values_realtime.speed));
      Serial.print(F("Distance travelled: "));
      Serial.println(String(vesc_values_realtime.distanceTravelled));
      Serial.print(F("Input voltage: "));
      Serial.println(String(vesc_values_realtime.inputVoltage));
      Serial.print(F("Batt percent: "));
      Serial.println(String(vesc_values_realtime.battPercent));
    }
    if (ssNew && displaySENSData) {
      Serial.print(F("New sensor data recieved\n(Pitch, roll, heading): "));
      Serial.println(String(sensor_values_realtime.pitch)+","+String(sensor_values_realtime.roll)+","+String(sensor_values_realtime.heading));
      Serial.print(F("Acceleration: "));
      Serial.println(String(sensor_values_realtime.acceleration));
      Serial.print(F("Temperature: "));
      Serial.println(String(sensor_values_realtime.temperature));
      Serial.print(F("Altitude: "));
      Serial.println(String(sensor_values_realtime.altitude));
    }
  }

  if (currentMillis-prevRadioResendMillis >= radioResendInterval) { //check if we should send all radio commands
    sendAllRadioCommands(); //don't need to send second HB signal because it was already send in sendAllRadioCommands
    prevHBMillis = currentMillis;
    prevRadioResendMillis = currentMillis;
  } else if (currentMillis-prevHBMillis >= HBInterval) { //every HBInterval ms send a new heartbeat to board (if it isn't already sent
    //Serial.println("board ping");
    radioTransmitMode();
    resetDataTx();
    dataTx[0] = HEARTBEAT;
    radio.write(&dataTx, sizeof(dataTx));
    prevHBMillis = currentMillis;
  }

  //Check if controller is still connected (hearbeat signal present within time?)
  connected = (currentMillis-lastHBTime >= HBTimeoutMax) ? false : true;
  if (oldConnected != connected) { //only update on change
    updateDisplayFlag = true;
  }
  oldConnected = connected;

  //Perform display update if enough time has elapsed
  if (currentMillis-prevDispUpdateMillis >= dispMinUpdate && (updateDisplayFlag || currentMillis-lastDispStateOneTime > displayStateOneChangeTime) && MASTER_STATE != 0) {
    updateDisplayFlag = false;
    prevDispUpdateMillis = currentMillis;
    
    if (displaySensorData) { //update with proper display mode
      updateDisplay(DISPU_SENSDATA);
    } else {
      updateDisplay(DISPU_FULL);
    }
  }

  //End tone if it's expired
  if (currentMillis > toneExpire && toneActive) {
    noTone(BUZZER_PIN);
    toneActive = false;
  }

  radioRecieveMode(); //default to recieve mode so as to not drop transmissions
}

void sendAllRadioCommands() { //sends all commands to board
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
  dataTx[0] = THROTTLE_VAL; //throttle update
  dataTx[1] = prevThrottle;
  radio.write(&dataTx, sizeof(dataTx));
  resetDataTx();
  dataTx[0] = CLICK; //click event
  dataTx[1] = ENclicks;
  radio.write(&dataTx, sizeof(dataTx));
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
  Serial.print(F("New state: "));
  Serial.println(newState);
  switch (newState) {
    case 0:
      updateDisplay(DISPU_CONN_WAIT);
      break;
    case 1:
      updateDisplay(DISPU_FULL);
      break;
  }  
}

void updateDisplay(DISPLAY_UPDATE_TYPES d) { //A lot of help for this: https://github.com/olikraus/u8glib/wiki/tpictureloop and this http://henrysbench.capnfatz.com/henrys-bench/u8glib-graphics-library-user-guide/u8glib-arduino-oled-tutorial-1-hello-world-on-steroids/
  int x = 0;
  int y = 0;
  //draw throttle, page, batt, signal
  u8g2.firstPage();
  do {
    switch (d) {
      case DISPU_CONN_WAIT:
        u8g2.setFont(u8g2_font_helvB12_tr);
        u8g2.drawStr(0, 13, "Waiting for");
        u8g2.drawStr(0, 28, "connection...");
        u8g2.setFont(u8g2_font_helvR10_tr);
        u8g2.drawStr(0, 48, "Autoconnect or");
        u8g2.drawStr(0, 64, "triple click to unlock");

        break;
      case DISPU_START:
          u8g2.setFont(u8g2_font_helvR10_tr);
          u8g2.drawXBM(4, 4, 24, 24, logo_bits);
          u8g2.drawStr(34, 22, "EskateOS V2");
          u8g2.drawStr(5, 50, "By Aaron Becker");
        break;
      case DISPU_VERSION:
        u8g2.setFont(u8g2_font_logisoso22_tn);
        u8g2.drawStr(5, 50, "V5.3.3");
        break;
      case DISPU_SENSDATA:
        x = 0;
        y = 0;

        u8g2.setFont(u8g2_font_helvR10_tr);
        u8g2.drawStr(x+50, y+12, "Sensor Data");
        u8g2.setFont(u8g2_font_profont12_tr);

        y+=12; //Pitch
        displayString = "P:";
        displayString += String(sensor_values_realtime.pitch);
        displayString.toCharArray(displayBuffer, 8);
        u8g2.drawStr(x, y, displayBuffer);

        y+=12; //Roll
        displayString = "R:";
        displayString += String(sensor_values_realtime.roll);
        displayString.toCharArray(displayBuffer, 8);
        u8g2.drawStr(x, y, displayBuffer);

        y+=12; //Heading
        displayString = "H:";
        displayString += String(sensor_values_realtime.heading);
        displayString.toCharArray(displayBuffer, 8);
        u8g2.drawStr(x, y, displayBuffer);

        y+=12; //Temperature
        displayString = "T:";
        displayString += String(sensor_values_realtime.temperature);
        displayString.toCharArray(displayBuffer, 8);
        u8g2.drawStr(x, y, displayBuffer);

        y+=12; //Altitude
        displayString = "A:";
        displayString += String(sensor_values_realtime.altitude);
        displayString.toCharArray(displayBuffer, 8);
        u8g2.drawStr(x, y, displayBuffer);
        break;
      case DISPU_FULL:
        x = 0;
        y = 0;
        /*
        * BATTERY LEVEL
        */

        // Position on OLED
        x = 108; y = 4;

        u8g2.drawFrame(x + 2, y, 18, 9);
        u8g2.drawBox(x, y + 2, 2, 5);

        for (int i = 0; i < 5; i++) {
          int p = round((100 / 5) * i);
          if (p <= vesc_values_realtime.battPercent)
          {
            u8g2.drawBox(x + 4 + (3 * i), y + 2, 2, 5);
          }
        }

        /*
        * SIGNAL INDICATOR
        */
        x = 114; y = 17;

        if (connected == true) {
          if (throttleEnabled) {
            u8g2.drawXBM(x, y, 12, 12, signal_transmitting_bits);
          } else {
            u8g2.drawXBM(x, y, 12, 12, signal_connected_bits);
          }
        } else {
          u8g2.drawXBM(x, y, 12, 12, signal_noconnection_bits);
        }

        /*
        * ENABLED INDICATOR
        */

        x = 110;
        y = 40;

        u8g2.setFont(u8g2_font_profont12_tr);
        if (throttleEnabled) {
          u8g2.drawStr(x, y, "T:E");
        } else {
          u8g2.drawStr(x, y, "T:D");
        }

        /*
        * BOOST INDICATOR
        */

        x = 110;
        y = 50;

        u8g2.setFont(u8g2_font_profont12_tr);
        if (boostEnabled) {
          u8g2.drawStr(x, y, "B:E");
        } else {
          u8g2.drawStr(x, y, "B:D");
        }

        /*
        * LED MODE INDICATOR
        */

        x = 110;
        y = 60;
        u8g2.setFont(u8g2_font_profont12_tr);
        displayString = "L:";
        displayString += String(ledMode);
        displayString.toCharArray(displayBuffer, 4);
        u8g2.drawStr(x, y, displayBuffer);

        /*
        * THROTTLE INDICATOR
        */
        x = 0;
        y = 0;

        // Draw throttle
        u8g2.drawHLine(x, y, 52);
        u8g2.drawVLine(x, y, 10);
        u8g2.drawVLine(x + 52, y, 10);
        u8g2.drawHLine(x, y + 10, 5);
        u8g2.drawHLine(x + 52 - 4, y + 10, 5);

        if (throttle >= 127) {
          int width = map(throttle, 127, 255, 0, 49);

          for (int i = 0; i < width; i++) {
            u8g2.drawVLine(x + i + 2, y + 2, 7);
          }
        } else {
          int width = map(throttle, 0, 126, 49, 0);
          for (int i = 0; i < width; i++) {
            u8g2.drawVLine(x + 50 - i, y + 2, 7);
          }
        }


        /*
        * VESC DATA INDICATOR
        */

        x = 0;
        y = 26;
        String prefix;
        String suffix;
        float value;
        int decimals;
        int first, last;

        if (millis()-lastDispStateOneTime > displayStateOneChangeTime) {
          displayStateOne = !displayStateOne;
          lastDispStateOneTime = millis();
        }

        for (int i=0; i<2; i++) {
          switch (i) {
            case 0: //>--- Speed
              prefix = F("SPEED");
              suffix = F("MPH");
              value = vesc_values_realtime.speed;
              decimals = 1;
              break;
            case 1: //>--- Distance
              if (displayStateOne) {
                prefix = F("DISTANCE");
                suffix = F("MI");
                value = vesc_values_realtime.distanceTravelled;
                decimals = 2;
              } else { //>--- Batt Voltage
                prefix = F("BATTV");
                suffix = F("V");
                value = vesc_values_realtime.inputVoltage;
                decimals = 1;
              }
              break;
            // case 3: //>--- Mosfet Temp (VESC)
            //   prefix = F("FTEMP");
            //   suffix = F("F");
            //   value = vesc_values_realtime.temp;
            //   decimals = 2;
            //   break;
          }

          // Display prefix (title)
          displayString = prefix;
          displayString.toCharArray(displayBuffer, 10);
          u8g2.setFont(u8g2_font_profont12_tr);
          u8g2.drawStr(x, y - 1, displayBuffer);

          // Split up the float value: a number, b decimals.
          first = abs(floor(value));
          last = value * pow(10, 3) - first * pow(10, 3);

          // Add leading zero
          if (first <= 9) {
            displayString = "0" + (String)first;
          } else {
            displayString = (String)first;
          }

          // Display numbers
          displayString.toCharArray(displayBuffer, 10);
          u8g2.setFont(u8g2_font_logisoso22_tn);
          u8g2.drawStr(x + 55, y + 13, displayBuffer);

          // Display decimals
          displayString = "." + (String)last;
          displayString.toCharArray(displayBuffer, decimals + 2);
          u8g2.setFont(u8g2_font_profont12_tr);
          u8g2.drawStr(x + 86, y - 1, displayBuffer);

          // Display suffix
          displayString = suffix;
          displayString.toCharArray(displayBuffer, 10);
          u8g2.setFont(u8g2_font_profont12_tr);
          u8g2.drawStr(x + 86 + 2, y + 13, displayBuffer);

          y+=25;
        }
        break;
      case CLEAR:
      default:
        break;
    }
  } while ( u8g2.nextPage() );
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
