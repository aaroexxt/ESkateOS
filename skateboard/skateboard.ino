/***************
 ______     ______     __  __     ______     ______   ______        ______     ______     ______     ______     _____    
/\  ___\   /\  ___\   /\ \/ /    /\  __ \   /\__  _\ /\  ___\      /\  == \   /\  __ \   /\  __ \   /\  == \   /\  __-.  
\ \  __\   \ \___  \  \ \  _"-.  \ \  __ \  \/_/\ \/ \ \  __\      \ \  __<   \ \ \/\ \  \ \  __ \  \ \  __<   \ \ \/\ \ 
 \ \_____\  \/\_____\  \ \_\ \_\  \ \_\ \_\    \ \_\  \ \_____\     \ \_____\  \ \_____\  \ \_\ \_\  \ \_\ \_\  \ \____- 
  \/_____/   \/_____/   \/_/\/_/   \/_/\/_/     \/_/   \/_____/      \/_____/   \/_____/   \/_/\/_/   \/_/ /_/   \/____/ 
                                                                                                                                                                                 
****************

  By Aaron Becker
  V2 Dec 2019/Jan 2020
*/

#include <ServoTimer2.h>
#include <Wire.h>
#include <FastLED.h>
#include <RF24.h>
#include <vesc.h> //Thank you to Gian Marcov for this awesome library: https://github.com/gianmarcov/arduino_vesc. Modified by me (Aaron Becker) to use SoftwareSerial instead of HardwareSerial
#include <SoftwareSerial.h>

//PIN DEFS

//ESC pins
ServoTimer2 ESC_LEFT; //Create FSESC "servo" output
ServoTimer2 ESC_RIGHT; //Create FSESC "servo" output
#define ESC_R_PIN 5
#define ESC_L_PIN 6

#define ESC_MIN 800
#define ESC_NONBOOST_MAX 1700
#define ESC_MAX 2000
#define ESC_STOP 1400 //(ESC_MIN+ESC_MAX)/2;

#define JOY_MIN 0
#define JOY_MAX 880
int realPPM = ESC_STOP;
int targetPPM = ESC_STOP;
int realRAW = 0;
unsigned long prevSpeedMillis = 0;
boolean throttleEnabled = false;
boolean boostEnabled = false;

//VESC pins/defs
#define VESC_UART_RX A0
#define VESC_UART_TX A1
SoftwareSerial vescSerial(VESC_UART_RX, VESC_UART_TX); //RX, TX
Vesc VUART; //instantiate vescUart object
float VRATIO_RPM_SPEED;
float VRATIO_TACHO_KM;
float VBATT_MAX;
float VBATT_MIN;
unsigned long prevVUpdateMillis = 0;
#define VUpdateMillis 500 //time between vesc updates

//Led pins/defs
#define LED_DATA_PIN    3
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    30
CRGB leds[NUM_LEDS];
#define LED_BRIGHTNESS 128
#define LED_FPS 120
unsigned long prevLEDMillis = 0;
#define LEDdelayShort 10
#define LEDdelayLong 100
int LEDdelay = LEDdelayLong;
int ledPosition = 0; //current position in strip for pattern
int ledState = 1;
/* LED STATES MASTER
0 off
1 initial or disconnect leds (chasing blue)
2 rainbow
3 follow throttle
*/

//Radio pins/defs
RF24 radio(7, 8);
const PROGMEM byte addresses [][6] = {"00001", "00002"}; //write at addr 00002, read at addr 00001
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

double dataRx[3]; //double takes up 8 bytes, each payload is 32 bytes, so this will use 24 of the 32 bytes
double dataTx[3];
unsigned long prevHBMillis = 0;
#define HBTimeoutMax 275 //max time between signals before board cuts the motors in ms
boolean radioListening = false;

//General pins/defs
int MASTER_STATE = 0;

void setup() {
  while (!Serial){;}

  Serial.begin(9600);
  Serial.println(F("ESKATEINIT_setup begin"));

  //Setup ESC
  ESC_LEFT.attach(ESC_L_PIN);
  ESC_RIGHT.attach(ESC_R_PIN);
  ESC_LEFT.write(realPPM); //set them to be basically off (middle position)
  ESC_RIGHT.write(realPPM);
  Serial.println(F("Setup esc: ok"));

  //Setup LEDS
  FastLED.addLeds<LED_TYPE,LED_DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear();
  for (int i=0; i<NUM_LEDS; i++) {
    leds[i] = CRGB::Black; //set all leds to be off
  }
  FastLED.show();
  Serial.println(F("Setup leds: ok"));

  //Setup VESC UART
  pinMode(VESC_UART_TX, OUTPUT); //Set pins correct direction
  pinMode(VESC_UART_RX, INPUT);
  vescSerial.begin(115200);
  while (!vescSerial) {;}
  VUART.init(&vescSerial, &Serial); //Serial for debug port, vescSerial for communciations
  delay(50);

  calculateRatios();

  Serial.println(F("Setup VESC: ok")); //TODO add check if it's null or 0 so it can fail properly

  //Setup radio
  radio.begin();
  radio.setPALevel(RF24_PA_MAX); //max because we don't want to lose connection
  radio.setRetries(3,3); // delay, count
  //SKATEBOARD Writes to addr 2, reads from addr 1
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]); //set address to recieve data
  radioRecieveMode();
  Serial.println(F("Setup radio: ok"));

  transitionState(0);
}

void loop() {
  switch (MASTER_STATE) {
    case 0: //case 0 waiting for first hb signal from controller
      radioRecieveMode(); //ensure we're recieving
      resetDataRx();
      if (radio.available()) {
        radio.read(&dataRx, sizeof(dataRx));
        if (dataRx[0] == HEARTBEAT) { //200 is "heartbeat" signal
          Serial.println(F("Got first heartbeat signal from controller; we're go"));
          ledState = 0; //Make sure LEDs are off

          radioTransmitMode();
          resetDataTx();
          dataTx[0] = HEARTBEAT;
          radio.write(&dataTx, sizeof(dataTx)); //send one back

          delay(200);
          dataTx[0] = TONE;
          dataTx[1] = 250; //tone
          dataTx[2] = 200; //time
          radio.write(&dataTx, sizeof(dataTx)); //send pitch command

          prevHBMillis = millis();
          transitionState(1);
        }
      }
      break;
    case 1: //standard operation
      radioRecieveMode();
      if (radio.available()) {
        resetDataRx();
        radio.read(&dataRx, sizeof(dataRx));
        /*if (dataRx[0] != HEARTBEAT) {
          Serial.print("Got event #: ");
          Serial.print(dataRx[0]);
          Serial.print(", value1: ");
          Serial.print(dataRx[1]);
          Serial.print(", value2: ");
          Serial.println(dataRx[2]);
        }*/
        switch ((int)dataRx[0]) {
          case THROTTLE_VAL: //1 is throttle update
            realRAW = dataRx[1];
            break;
          case THROTTLE_SW:
            throttleEnabled = dataRx[1];
          case BOOSTMODE:
            boostEnabled = dataRx[1];
          case LEDMODE: //2 is led mode update
            if (dataRx[1] <= 3) { //sanity check
              if (dataRx[1] == 0) {
                FastLED.clear();
                FastLED.show();
              }
              ledState = dataRx[1];
            }
            break;
          case HEARTBEAT: //heartbeat. if we get one, we should send one
            // Serial.println("controller hb recieved");
            radioTransmitMode();
            resetDataTx();
            dataTx[0] = HEARTBEAT;
            radio.write(&dataTx, sizeof(dataTx));

            prevHBMillis = millis();
            break;
        }
      }
      break;
    case 2: //lost connection case
      radioRecieveMode(); //ensure we're recieving
      resetDataRx();
      if (radio.available()) {
        radio.read(&dataRx, sizeof(dataRx));
        if (dataRx[0] == HEARTBEAT) { //200 is "heartbeat" signal
          Serial.println(F("Got heartbeat signal from controller after disconnection"));
          ledState = 0; //Make sure LEDs are off

          radioTransmitMode();
          resetDataTx();
          dataTx[0] = SENDALLDATA; //Set sendAllData flag (SAD flag) on controller to poll all values
          radio.write(&dataTx, sizeof(dataTx));
          
          prevHBMillis = millis();
          transitionState(1); //go back to normal operation
        }
      }
      break;
    default:
      Serial.println(F("Undefined state; resetting"));
      transitionState(0);
  }

  unsigned long currentMillis = millis();
  
  if (currentMillis-prevLEDMillis>=LEDdelay && ledState > 0) { //make sure the leds are enabled
    prevLEDMillis = currentMillis;
    switch (ledState) {
      case 1: //blue chase
        LEDdelay = LEDdelayLong;
        for (int i=0; i<NUM_LEDS; i++) {
          if ((i+ledPosition)%5 == 0) {
            leds[i] = CRGB::Green;
          } else {
            leds[i] = CRGB::Black;
          }
        }
        ledPosition++;
        if (ledPosition > 4) {
          ledPosition = 0;
        }
        FastLED.show();
        break;
      case 2: //rainbow
        LEDdelay = LEDdelayShort;
      //TODO MAKE RAINBOW FADE IN/OUT BASED ON THROTTLE
      //https://github.com/marmilicious/FastLED_examples/blob/master/rainbow_brightness_and_saturation.ino
        fill_rainbow(leds, NUM_LEDS, millis()/10, 7);
        FastLED.show();
        break;
      case 3: //color changes based on throttle (chaser again)
        LEDdelay = LEDdelayShort;
        int greenChannel = map(realPPM, ESC_MIN, ESC_MAX, 0, 255);
        int redChannel = map(ESC_MAX-realPPM, ESC_MIN, ESC_MAX, 0, 255);
        for (int i=0; i<NUM_LEDS; i++) {
          leds[i] = CRGB(redChannel, 0, greenChannel);
        }
        FastLED.show();
        break;
    }
  }

  if (currentMillis-prevHBMillis>=HBTimeoutMax && MASTER_STATE == 1) { //have we lost connection with the controller while operating normally? welp then we should prolly cut motors
    transitionState(2);
  }

  if (currentMillis-prevVUpdateMillis >= VUpdateMillis && MASTER_STATE == 1) { //Time for VESC update
    prevVUpdateMillis = currentMillis;
    radioTransmitMode();
    resetDataTx();
    //ID 12: VESC data [id, value]. ID 0 is speed, ID 1 is distance travelled, ID 2 is input voltage, ID 3 is fet temp, ID 4 is batt percent
    float distance = (float)VUART.getTachometerAbsValue()*VRATIO_TACHO_KM;
    Serial.print(F("Distance travelled (tachometer): "));
    Serial.println(String(distance));

    float speed = (float)VUART.getRPM()*VRATIO_RPM_SPEED;
    Serial.print(F("Current speed (rpm): "));
    Serial.println(String(speed));

    float fetT = VUART.getFetTemperature();
    Serial.print(F("Current FETTemp: "));
    Serial.println(String(fetT));

    float inpVoltage = VUART.getInputVoltage();
    Serial.print(F("Current inpVoltage: "));
    Serial.println(String(inpVoltage));

    float battPercent = mapFloat(inpVoltage, VBATT_MIN, VBATT_MAX, 0, 100);
    battPercent = (battPercent < VBATT_MIN) ? VBATT_MIN : (battPercent > VBATT_MAX) ? VBATT_MAX : battPercent;
    Serial.print(F("Current battPercent: "));
    Serial.println(String(battPercent));

    dataTx[0] = 12;
    dataTx[1] = 0;
    dataTx[2] = speed;
    radio.write(&dataTx, sizeof(dataTx));

    dataTx[1] = 1;
    dataTx[2] = distance;
    radio.write(&dataTx, sizeof(dataTx)); 
    
    dataTx[1] = 2;
    dataTx[2] = inpVoltage;
    radio.write(&dataTx, sizeof(dataTx)); 

    dataTx[1] = 3;
    dataTx[2] = fetT;
    radio.write(&dataTx, sizeof(dataTx));

    dataTx[1] = 4;
    dataTx[2] = battPercent;
    radio.write(&dataTx, sizeof(dataTx));     
  }

  updateESC(); //Update ESC with throttle value

  radioRecieveMode(); //Set radio back to recieve mode at end of loop
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calculateRatios() { //seperate function to save ram
  mc_configuration VCONFIG = VUART.getMotorConfiguration(); //takes all data directly from VESC
  float gearRatio = VCONFIG.si_gear_ratio;
  int wheelDiameter = VCONFIG.si_wheel_diameter;
  int motorPoles = VCONFIG.si_motor_poles;
  int batteryCells = VCONFIG.si_battery_cells;

  vesc_version VVERSION = VUART.getFirmwareVersion();
  Serial.println(F("VESC Connection---"));
  Serial.print(F("FW major: "));
  Serial.println(String(VVERSION.major));
  Serial.print(F("FW minor: "));
  Serial.println(String(VVERSION.minor));

  //The following code yoinked basically directly from https://github.com/SolidGeek/nRF24-Esk8-Remote/blob/master/transmitter/transmitter.ino. Thanks SolidGeek :)
  VRATIO_RPM_SPEED = (gearRatio * 60.0 * (float)wheelDiameter * 3.14156) / (((float)motorPoles / 2.0) * 1000000.0); // ERPM to Km/h
  VRATIO_TACHO_KM = (gearRatio * (float)wheelDiameter * 3.14156) / (((float)motorPoles * 3.0) * 1000000.0); // Pulses to km travelled
  VBATT_MAX = (float)batteryCells*4.2; //assuming max charge of 4.2V per cell
  VBATT_MIN = (float)batteryCells*3.2; //assuming min charge of 3.2V per cell
}

void transitionState(int newState) {
  MASTER_STATE = newState;
  Serial.print("New state: ");
  Serial.println(newState);
  switch (newState) {
    case 1:
      if (ledState == 0) { //enable/disable the leds based on what's going on
        FastLED.clear();
        FastLED.show();
      }
      break;
    case 2: //uhoh we are going into remote disconnect mode
      Serial.println(F("Uhoh we've lost connection to the remote :("));
      ledState = 1; //go back into disconnected mode
      realPPM = ESC_STOP; //set target to 0 speed to bring us back down to 0 speed
      break;
  }  
}

void updateESC() {
  if (abs(targetPPM-ESC_STOP) < 50) {
    targetPPM = ESC_STOP;
  }

  if (throttleEnabled) {
    realRAW = constrain(realRAW, JOY_MIN, JOY_MAX); //constrain raw value
    targetPPM = map(realRAW, JOY_MIN, JOY_MAX, ESC_MIN, (boostEnabled) ? ESC_MAX : ESC_NONBOOST_MAX); //calculate ppm
    targetPPM = constrain(realPPM, ESC_MIN, ESC_MAX); //make sure we're within limits for safety even tho it should never be an issue
  } else {
    targetPPM = ESC_STOP;
  }

  unsigned long currentMillis = millis();
  int deltaMillis = currentMillis-prevSpeedMillis; //ensure that acceleration rate is constant no matter what loop rate is
  int deltaPPM = targetPPM-realPPM;
    //RATE: 5 ppm/ms

  if (targetPPM > realPPM) { //we need to go
    realPPM += deltaMillis*5;//(targetPPM-realPPM)/20;
  } else if (targetPPM < realPPM) { //brake faster!
    realPPM -= deltaMillis*5;//(targetPPM-realPPM)/10;
  }
  prevSpeedMillis = currentMillis;

  realPPM = constrain(realPPM, ESC_MIN, ESC_MAX);
  ESC_LEFT.write(realPPM); //write the values
  ESC_RIGHT.write(realPPM);
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
