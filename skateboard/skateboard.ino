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

#include <Servo.h>
#include <Wire.h>
#include <FastLED.h>
#include <RF24.h>
//#include <VescUart.h> //Thank you to Gian Marcov for this awesome library: https://github.com/gianmarcov/arduino_vesc. Modified by me (Aaron Becker) to use SoftwareSerial instead of HardwareSerial
#include "printf.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>

//Debug stuff (incompatible with vesc)

//#define DEBUG


#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.println (x)
  #include "printf.h"
  const boolean debug = true;
#else
  #define DEBUG_PRINT(x)
  const boolean debug = false;
#endif


//ESC pins
Servo ESC; //Create FSESC "servo" output
#define ESC_PIN 5

#define ESC_MIN 800
#define ESC_NONBOOST_MAX 1700
#define ESC_MAX 2000
#define ESC_STOP (ESC_MIN+ESC_MAX)/2
#define ESC_DEADZONE 100

//in PPM pulses per second (Ppm range = 1200, so 100PPM = ~8.3% throttle, 5% thrott per sec is equal to 60 ppm/sec)
#define PPM_BRAKE_RATE 300
#define PPM_ACCEL_RATE_NONBOOST 85
#define PPM_ACCEL_RATE_BOOST 250
#define PPM_JUMP_VALUE 90
#define PPM_WITHIN_JUMP_RATE 500 //Pass through "dead zone" of throttle really quickly

#define HALL_MIN 0
#define HALL_MAX 255 //real is 220, TODO fix this
#define HALL_STOP (HALL_MAX+HALL_MIN)/2
int realPPM = ESC_STOP;
int realRAW = 0;
unsigned long prevLoopMillis = 0;
boolean throttleEnabled = false;
boolean boostEnabled = false;

boolean turnLightsEnabled = false;
boolean oldTurnLightsEnabled = false; //Keep track of prev state of turnLightsEnabled so that we can catch rising/falling events
#define TURNLIGHTS_TOGGLE_CLICK_COUNT 2

/*VescUart VUART;
float VRATIO_RPM_SPEED;
float VRATIO_TACHO_KM;
float VRATIO_KM_MI;
const boolean unitsInKM = false; //are units in kilometers?
float VBATT_MAX;
float VBATT_MIN;
unsigned long prevVUpdateMillis = 0;
#define VUpdateMillis 1000 //time between vesc updates
*/
#define initialVESCCheckDelay 5000
boolean initialVESCCheck = false;
/*
//Vesc data
struct VREALTIME {
  float speed;
  float distanceTravelled;
  float inputVoltage;
  float battPercent;
};
struct VREALTIME vesc_values_realtime;
*/
//Led pins/defs
#define LED_DATA_PIN    3
#define LED_TYPE    WS2811
#define COLOR_ORDER BRG //I'm using a BRG led strip which is kinda wacky
#define NUM_LEDS_BOARD    32
#define NUM_LEDS_BRAKE    2
CRGB leds[NUM_LEDS_BOARD+NUM_LEDS_BRAKE];
#define LED_BRIGHTNESS 128
#define LED_FPS 120
unsigned long prevLEDMillis = 0;
#define LEDdelayShort 10
#define LEDdelayLong 100
int LEDdelay = LEDdelayLong;
int ledPosition = 0; //current position in strip for pattern

typedef enum {
  LEDSTATE_OFF = 0,
  LEDSTATE_INITCHASE = 1,
  LEDSTATE_RAINBOW = 2,
  LEDSTATE_CHGTHROTT = 3
} LEDLIGHT_STATES;
int ledState = LEDSTATE_INITCHASE;
/* LED STATES MASTER
0 off
1 initial or disconnect leds (chasing blue)
2 rainbow
3 follow throttle
*/

typedef enum {
  BRAKELIGHT_INIT = 0,
  BRAKELIGHT_NOTBRAKING = 1,
  BRAKELIGHT_BRAKING = 2,
  BRAKELIGHT_TURNRIGHT = 3,
  BRAKELIGHT_TURNLEFT = 4
} BRAKELIGHT_STATES;
int brakeLightState = BRAKELIGHT_INIT;

boolean BRAKELIGHT_SETROLLOFFSET = false;
float BRAKELIGHT_ROLL_OFFSET = 0;
boolean BRAKELIGHT_TURNLIGHTON = false;
const float BRAKELIGHT_TURNING_THRESHOLD = 2.0; //Roll threshold for triggering turn signal, in degrees

//IMU pins/defs
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
const float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; //sea level pressure in HPA
const char ACCEL_AXIS = 'x'; //axis that board accelerates along; used for accel math

unsigned long prevSUpdateMillis = 0;
#define SUpdateMillis 200

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
const byte addresses [][6] = {"00001", "00002"}; //write at addr 00002, read at addr 00001
//Send 6 bytes (because each int is 2 bytes) per rx/tx
/*Data structure:
First int is command number
Second int is value 1
Third int is value 2 (so you can send up to four bytes of data if you want)
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

double dataRx[3]; //double takes up 8 bytes, each payload is 32 bytes, so this will use 24 of the 32 bytes (no dynamic payload)
double dataTx[3];
unsigned long lastHBTime = 0; //time when heartbeat signal was last recieved
#define HBTimeoutMax 750 //max time between signals before board cuts the motors in ms
boolean radioListening = false;

//General pins/defs
int MASTER_STATE = 0;

//The board should still be able to run if either the VESC or IMU is not present, since they are not essential to operation. These variables keep track of whether they're connected correctly
boolean VESCOK = false; //needs to be false because it's set true if its ok later (5s delay to give vesc time to start up)
boolean SENSOK = true; //needs to be true because it's immediately set false if it's bad

void setup() {
  while (!Serial){;}

  Serial.begin(115200);
  Serial.println(F("ESKATEINIT_setup begin. By Aaron Becker"));

  //Setup ESC
  ESC.attach(ESC_PIN);
  ESC.write(ESC_STOP);
  DEBUG_PRINT(F("Setup esc: ok"));

  //Setup radio
  radio.begin();
  radio.setPALevel(RF24_PA_MAX); //max because we don't want to lose connection
  radio.setRetries(3,3); // delay, count
  //SKATEBOARD Writes to addr 2, reads from addr 1
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]); //set address to recieve data
  radioRecieveMode();
  DEBUG_PRINT(F("Radio details:"));
  if (debug) {
    printf_begin();
    radio.printDetails();
  }
  DEBUG_PRINT(F("Setup radio: ok"));

  //Setup LEDS
  FastLED.addLeds<LED_TYPE,LED_DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS_BOARD+NUM_LEDS_BRAKE).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear();
  
  //Make sure we're displaying nothing
  writeBoardLEDSSolid(CRGB::Black);
  writeBrakeLEDSSolid(CRGB::Black);
  FastLED.show();
  DEBUG_PRINT(F("Setup leds: ok"));

  //Setup accelerometer
  if(!accel.begin())
  {
    // There was a problem detecting the ADXL345 ... check your connections
    Serial.println(F("Setup accel: fail. not detected :("));
    SENSOK = false;
  }
  if(!mag.begin())
  {
    // There was a problem detecting the LSM303 ... check your connections
    Serial.println(F("Setup mag: fail. not detected :("));
    SENSOK = false;
  }
  if(!bmp.begin())
  {
    // There was a problem detecting the BMP085 ... check your connections
    Serial.print(F("Setup bmp: fail. not detected :("));
    SENSOK = false;
  }
  DEBUG_PRINT(F("Setup accel/mag/bmp:"));
  DEBUG_PRINT((SENSOK)?"ok":"failed");

  //Setup VESC UART
  /*DEBUG_PRINT(F("bef vesc init"));
  VUART.setSerialPort(&Serial);
  DEBUG_PRINT(F("aft vesc init"));*/

  //calculateRatios(); //Calculate VESC distance/speed ratios

  //DEBUG_PRINT(F("Initial VESC Setup: ok")); //TODO add check if it's null or 0 so it can fail properly

  //Goto state 0; waiting for connection
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
          DEBUG_PRINT(F("Got first heartbeat signal from controller; we're go"));
          ledState = 0; //Make sure LEDs are off

          radioTransmitMode();
          resetDataTx();
          dataTx[0] = HEARTBEAT;
          radio.write(&dataTx, sizeof(dataTx)); //send one back

          delay(50);
          dataTx[0] = TONE;
          dataTx[1] = 2550; //tone (g)
          dataTx[2] = 200; //time
          radio.write(&dataTx, sizeof(dataTx)); //send pitch command

          lastHBTime = millis();
          transitionState(1);
        }
      }
      break;
    case 1: //standard operation
      radioRecieveMode();
      if (radio.available()) {
        resetDataRx();
        radio.read(&dataRx, sizeof(dataRx));

        if (dataRx[0] != HEARTBEAT && debug) {
          Serial.print(F("Got comm event #: "));
          Serial.println(dataRx[0]);
          Serial.print(F(", value1: "));
          Serial.println(dataRx[1]);
          Serial.print(F(", value2: "));
          Serial.println(dataRx[2]);
        }
        switch ((int)dataRx[0]) {
          case THROTTLE_VAL: //1 is throttle update
            realRAW = dataRx[1];
            break;
          case THROTTLE_SW:
            throttleEnabled = dataRx[1];
            break;
          case CLICK:
            if (dataRx[1] == TURNLIGHTS_TOGGLE_CLICK_COUNT) {
              turnLightsEnabled = !turnLightsEnabled; //toggle turnLightsEnabled
              //Small bit of logic to reupdate offset value on rising edge
              if (turnLightsEnabled != oldTurnLightsEnabled && turnLightsEnabled) {
                BRAKELIGHT_ROLL_OFFSET = sensor_values_realtime.roll;
              }
              oldTurnLightsEnabled = turnLightsEnabled;
            }
            break;
          case BOOSTMODE:
            boostEnabled = dataRx[1];
            break;
          case LEDMODE: //2 is led mode update
            if (dataRx[1] <= 3) { //sanity check for max LED state
              if (dataRx[1] == 0) { //If it's zero; clear everything and write
                writeBoardLEDSSolid(CRGB::Black);
                FastLED.show();
              }
              ledState = dataRx[1];
            }
            break;
          case HEARTBEAT: //heartbeat. if we get one, we should send one
            // DEBUG_PRINT(F("controller hb recieved"));
            radioTransmitMode();
            resetDataTx();
            dataTx[0] = HEARTBEAT;
            radio.write(&dataTx, sizeof(dataTx));

            lastHBTime = millis();
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
          DEBUG_PRINT(F("Got heartbeat signal from controller after disconnection"));
          ledState = 0; //Make sure LEDs are off

          radioTransmitMode();
          resetDataTx();
          dataTx[0] = SENDALLDATA; //Set sendAllData flag (SAD flag) on controller to poll all values
          radio.write(&dataTx, sizeof(dataTx));
          
          lastHBTime = millis();
          transitionState(1); //go back to normal operation
        }
      }
      break;
    default:
      DEBUG_PRINT(F("Undefined state; resetting"));
      transitionState(0);
  }

  unsigned long currentMillis = millis();
  int mappedVal;
  
  if (currentMillis-prevLEDMillis>=LEDdelay) { //make sure the leds are enabled
    prevLEDMillis = currentMillis;
    if (ledState > LEDSTATE_OFF) {
      switch (ledState) {
        case LEDSTATE_INITCHASE: //blue chase
          LEDdelay = LEDdelayLong;
          for (int i=0; i<NUM_LEDS_BOARD; i++) {
            if ((i+ledPosition)%5 == 0) {
              leds[i] = CRGB::Blue;
            } else {
              leds[i] = CRGB::Black;
            }
          }
          ledPosition++;
          if (ledPosition > 4) {
            ledPosition = 0;
          }
          break;
        case LEDSTATE_RAINBOW: //rainbow
          LEDdelay = LEDdelayShort;
          //https://github.com/marmilicious/FastLED_examples/blob/master/rainbow_brightness_and_saturation.ino
          //Fill internal LED array w/rainbow
          fill_rainbow(leds, NUM_LEDS_BOARD, millis()/10, 7);
          
          if (realPPM > ESC_STOP) { //Put a tint on if we're not braking
            mappedVal = map(realPPM, ESC_STOP, ESC_MAX, 0, 150); //Fade lights by 0 to 150 out of 255 in all RGB channels (towards white) based on dthrott
            CRGB tintAmnt = CRGB(mappedVal, mappedVal, mappedVal);
            for (int i=0; i<NUM_LEDS_BOARD; i++) {
              leds[i] += tintAmnt;
            }
          }
          break;
        case LEDSTATE_CHGTHROTT: //color changes based on throttle (chaser again)
          LEDdelay = LEDdelayShort;
          mappedVal = map(realPPM, ESC_MIN, ESC_MAX, 255, 0);
          for (int i=0; i<NUM_LEDS_BOARD; i++) {
            leds[i] = CRGB(mappedVal, 255-mappedVal, 0); //Modify Red and Green channel to be inverses to go between red and green as you accel (and yellow in the middle)
          }
          break;
      }
    }

    switch (brakeLightState) { //Only for the cases that need fast updating, i.e. safety critical lights like brake
      case BRAKELIGHT_INIT:
        writeBrakeLEDSSolid(CRGB::Blue);
        break;
      case BRAKELIGHT_BRAKING:
        writeBrakeLEDSSolid(CRGB::Red);
        break;
      case BRAKELIGHT_NOTBRAKING:
        writeBrakeLEDSSolid(CRGB::Black);
        break;
    }

    FastLED.show(); //Actually tell library to send LED state to LEDs
  }

  if (currentMillis-lastHBTime>=HBTimeoutMax && MASTER_STATE == 1) { //have we lost connection with the controller while operating normally? welp then we should prolly cut motors
    transitionState(2);
  }

  /*if (currentMillis-prevVUpdateMillis >= VUpdateMillis && MASTER_STATE == 1 && VESCOK) { //Time for VESC update
    prevVUpdateMillis = currentMillis;
    updateVESCData();
    sendVESCData();    
  }*/

  if (currentMillis-prevSUpdateMillis >= SUpdateMillis && MASTER_STATE == 1 && SENSOK) { //Time for sensor update
    prevSUpdateMillis = currentMillis;
    updateSensorData();
    updateBrakelightTurnState();
    sendSensorData();

    switch (brakeLightState) { //These need to only update once every sensorcheck ms
      case BRAKELIGHT_TURNRIGHT:
        writeBrakeLEDSSolid(CRGB::Black); //Start by making all brake leds blank (internal state)
        leds[NUM_LEDS_BOARD+NUM_LEDS_BRAKE-1] = (BRAKELIGHT_TURNLIGHTON) ? CRGB::Orange : CRGB::Black; //Make last LED orange
        BRAKELIGHT_TURNLIGHTON = !BRAKELIGHT_TURNLIGHTON;
        break;
      case BRAKELIGHT_TURNLEFT:
        writeBrakeLEDSSolid(CRGB::Black); //Start by making all brake leds blank (internal state)
        leds[NUM_LEDS_BOARD] = (BRAKELIGHT_TURNLIGHTON) ? CRGB::Orange : CRGB::Black; //Make first LED orange
        BRAKELIGHT_TURNLIGHTON = !BRAKELIGHT_TURNLIGHTON;
        break;
    }
    FastLED.show(); //Send state to LEDs

    if (debug) {
      Serial.println("BrakeLightState:");
      Serial.println(brakeLightState);
    }
  }

  if (currentMillis>initialVESCCheckDelay && !initialVESCCheck) { //VESC status check
    initialVESCCheck = true;
    /*if (VUART.getVescValues()) {
      VESCOK = true;
      DEBUG_PRINT(F("VESC intialComm: ok"));
    } else {
      VESCOK = false;
      DEBUG_PRINT(F("VESC initialComm: err"));
    }*/
    
    //Now that everything's setup...
    //GIVE rider a visual indication as to whether board is functioning ok. First blink: are sensors initialized? Second blink: is VESC ok?
    writeBrakeLEDSSolid((SENSOK)?CRGB::Green:CRGB::Red); //This is a fun one-liner to write, if sensOK then write CRGB green otherwise red
    FastLED.show();
    delay(250);
    writeBrakeLEDSSolid(CRGB::Black);
    FastLED.show();
    delay(250);
    writeBrakeLEDSSolid((VESCOK)?CRGB::Green:CRGB::Red);
    FastLED.show();
    delay(250);
  }

  updateESC(); //Update ESC with throttle value

  radioRecieveMode(); //Set radio back to recieve mode at end of loop
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*
void calculateRatios() { //seperate function to save ram
  // DEBUG_PRINT(F("vuart MC begin"));
  // mc_configuration VCONFIG = VUART.getMotorConfiguration(); //takes all data directly from VESC
  // float gearRatio = VCONFIG.si_gear_ratio;
  // int wheelDiameter = VCONFIG.si_wheel_diameter;
  // int motorPoles = VCONFIG.si_motor_poles;
  // int batteryCells = VCONFIG.si_battery_cells;
  // DEBUG_PRINT(F("vuart MC done"));


  //Hardcoded lol because oof old vesc library
  int motorPulley = 14;
  int wheelPulley = 36;
  float gearRatio = (float)motorPulley / (float)wheelPulley;
  int wheelDiameter = 90; //in mm
  int motorPoles = 14;
  int batteryCells = 10;
  DEBUG_PRINT(F("Vratios: hard ok"));

  //The following code yoinked basically directly from https://github.com/SolidGeek/nRF24-Esk8-Remote/blob/master/transmitter/transmitter.ino. Thanks SolidGeek :)
  VRATIO_RPM_SPEED = (gearRatio * 60.0 * (float)wheelDiameter * 3.14156) / (((float)motorPoles / 2.0) * 1000000.0); // ERPM to Km/h
  VRATIO_TACHO_KM = (gearRatio * (float)wheelDiameter * 3.14156) / (((float)motorPoles * 3.0) * 1000000.0); // Pulses to km travelled
  VRATIO_KM_MI = 0.6214;
  VBATT_MAX = (float)batteryCells*4.2; //assuming max charge of 4.2V per cell
  VBATT_MIN = (float)batteryCells*3.2; //assuming min charge of 3.2V per cell
}

void updateVESCData() {
  if (VUART.getVescValues()) {
    DEBUG_PRINT(F("Units in "));
    DEBUG_PRINT((unitsInKM?"kilometers":"miles"));
    vesc_values_realtime.distanceTravelled = (float)VUART.data.tachometerAbs*VRATIO_TACHO_KM*(unitsInKM?VRATIO_KM_MI:1);
    DEBUG_PRINT(F("Distance travelled (from tachometer): "));
    DEBUG_PRINT(String(vesc_values_realtime.distanceTravelled));

    vesc_values_realtime.speed = (float)VUART.data.rpm*VRATIO_RPM_SPEED*(unitsInKM?VRATIO_KM_MI:1);
    DEBUG_PRINT(F("Current speed (from rpm): "));
    DEBUG_PRINT(String(vesc_values_realtime.speed));

    vesc_values_realtime.inputVoltage = (float)VUART.data.inpVoltage;
    DEBUG_PRINT(F("Current inpVoltage: "));
    DEBUG_PRINT(String(vesc_values_realtime.inputVoltage));

    float battPercent = mapFloat(vesc_values_realtime.inputVoltage, VBATT_MIN, VBATT_MAX, 0, 100);
    vesc_values_realtime.battPercent = (battPercent < VBATT_MIN) ? VBATT_MIN : (battPercent > VBATT_MAX) ? VBATT_MAX : battPercent;
    DEBUG_PRINT(F("Current battPercent: "));
    DEBUG_PRINT(String(vesc_values_realtime.battPercent));
  } else {
    DEBUG_PRINT(F("Vesc data get fail"));
    vesc_values_realtime.distanceTravelled = -1.0;
    vesc_values_realtime.speed = -1.0;
    vesc_values_realtime.inputVoltage = -1.0;
    vesc_values_realtime.battPercent = -1.0;
  }
}

void sendVESCData() {
  radioTransmitMode();
  resetDataTx();
  //ID 12: VESC data [id, value]. ID 0 is speed, ID 1 is distance travelled, ID 2 is input voltage, ID 3 is batt percent
  dataTx[0] = 12;
  dataTx[1] = 0;
  dataTx[2] = vesc_values_realtime.speed;
  radio.write(&dataTx, sizeof(dataTx));

  dataTx[1] = 1;
  dataTx[2] = vesc_values_realtime.distanceTravelled;
  radio.write(&dataTx, sizeof(dataTx)); 
  
  dataTx[1] = 2;
  dataTx[2] = vesc_values_realtime.inputVoltage;
  radio.write(&dataTx, sizeof(dataTx)); 

  dataTx[1] = 3;
  dataTx[2] = vesc_values_realtime.battPercent;
  radio.write(&dataTx, sizeof(dataTx)); 
}
*/
void updateSensorData() {
  //Define variables to store accel events
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  accel.getEvent(&accel_event);
  if (!dof.accelGetOrientation(&accel_event, &orientation)) {
    DEBUG_PRINT(F("Error getting accel orientation"));
    return;
  }
  mag.getEvent(&mag_event);
  if (!dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
    DEBUG_PRINT(F("Error getting mag orientation"));
    return;
  }

  switch (ACCEL_AXIS) {
    case 'x':
      sensor_values_realtime.acceleration = accel_event.acceleration.x;
      break;
    case 'y':
      sensor_values_realtime.acceleration = accel_event.acceleration.y;
      break;
    case 'z':
      sensor_values_realtime.acceleration = accel_event.acceleration.z;
      break;
  }
  DEBUG_PRINT(F("Acceleration along direction of travel (axis, value):"));
  DEBUG_PRINT(ACCEL_AXIS);
  DEBUG_PRINT(sensor_values_realtime.acceleration);

  sensor_values_realtime.roll = orientation.roll;
  DEBUG_PRINT(F("Roll value: "));
  DEBUG_PRINT(sensor_values_realtime.roll);
  sensor_values_realtime.pitch = orientation.pitch;
  DEBUG_PRINT(F("Pitch value: "));
  DEBUG_PRINT(sensor_values_realtime.pitch);
  sensor_values_realtime.heading = orientation.heading;
  DEBUG_PRINT(F("Heading value: "));
  DEBUG_PRINT(sensor_values_realtime.heading);

  bmp.getEvent(&bmp_event);
  if (!bmp_event.pressure) {
    DEBUG_PRINT(F("Error getting bmp pressure (not calculating altitude)"));
  } else {
    bmp.getTemperature(&sensor_values_realtime.temperature);
    sensor_values_realtime.altitude = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, sensor_values_realtime.temperature);
    DEBUG_PRINT(F("Temperature value: "));
    DEBUG_PRINT(sensor_values_realtime.temperature);
    DEBUG_PRINT(F("Altitude value: "));
    DEBUG_PRINT(sensor_values_realtime.altitude);
  }
}

void updateBrakelightTurnState() {
  if (!BRAKELIGHT_SETROLLOFFSET) {
    BRAKELIGHT_SETROLLOFFSET = true;
    BRAKELIGHT_ROLL_OFFSET = sensor_values_realtime.roll;
  }
  if (brakeLightState != BRAKELIGHT_BRAKING) { //Ensure that we're not actually braking
    if (!turnLightsEnabled) { //Check if turn lights are enabled, if they aren't then set state accordingly
      brakeLightState = BRAKELIGHT_NOTBRAKING;
    } else {
      float adjRoll = sensor_values_realtime.roll-BRAKELIGHT_ROLL_OFFSET;
      if (debug) {
        Serial.println("BLS Roll adjusted:");
        Serial.println(String(adjRoll).substring(0,6));
      }
      if (adjRoll > BRAKELIGHT_TURNING_THRESHOLD) { //Pos roll -> clockwise turn -> turn right
        brakeLightState = BRAKELIGHT_TURNRIGHT;
      } else if (adjRoll < -BRAKELIGHT_TURNING_THRESHOLD) {
        brakeLightState = BRAKELIGHT_TURNLEFT;
      } else {
        brakeLightState = BRAKELIGHT_NOTBRAKING;
    }
    }
  }
}

void sendSensorData() {
  radioTransmitMode();
  resetDataTx();
  //ID 13: Sensor data [id, value]. ID 0 is pitch, ID 1 is roll, ID 2 is heading, ID 3 is acceleration, ID 4 is temperature, ID 5 is altitude
  dataTx[0] = 13;
  dataTx[1] = 0;
  dataTx[2] = sensor_values_realtime.pitch;
  radio.write(&dataTx, sizeof(dataTx));

  dataTx[1] = 1;
  dataTx[2] = sensor_values_realtime.roll;
  radio.write(&dataTx, sizeof(dataTx));

  dataTx[1] = 2;
  dataTx[2] = sensor_values_realtime.heading;
  radio.write(&dataTx, sizeof(dataTx));

  dataTx[1] = 3;
  dataTx[2] = sensor_values_realtime.acceleration;
  radio.write(&dataTx, sizeof(dataTx));
  
  dataTx[1] = 4;
  dataTx[2] = sensor_values_realtime.temperature;
  radio.write(&dataTx, sizeof(dataTx)); 

  dataTx[1] = 5;
  dataTx[2] = sensor_values_realtime.altitude;
  radio.write(&dataTx, sizeof(dataTx));
}

void transitionState(int newState) {
  MASTER_STATE = newState;
  DEBUG_PRINT(F("New state: "));
  DEBUG_PRINT(newState);
  switch (newState) {
    case 1:
      if (ledState == LEDSTATE_OFF) { //enable/disable the leds based on what's going on
        FastLED.clear();
        FastLED.show();
      }
      break;
    case 2: //uhoh we are going into remote disconnect mode
      DEBUG_PRINT(F("Uhoh we've lost connection to the remote :("));
      realRAW = HALL_STOP; //set target to 0 speed to bring us back down to 0 speed
      //DO NOT set realPPM to ESC_STOP, because this would instantly drop power to 0, meaning I could get thrown off the board

      ledState = LEDSTATE_INITCHASE; //go back into disconnected mode
      brakeLightState = BRAKELIGHT_INIT;
      break;
  }  
}

/*
*
* SAFETY-CRITICAL CODE NO TOUCHIE
*
*/
void updateESC() {
  unsigned long currentMillis = millis();

  int targetPPM = ESC_STOP; //Initialize tPPM
  if (throttleEnabled) {
    realRAW = constrain(realRAW, HALL_MIN, HALL_MAX); //constrain raw value
    targetPPM = map(realRAW, HALL_MIN, HALL_MAX, ESC_MAX, ESC_MIN); //calculate ppm

    //Check: is where we want to be within the deadzone? If so, the target should be just to stop
    int tDistFromZero = targetPPM-ESC_STOP; //Because of the way abs() is implemented, make a temp local variable
    if (abs(tDistFromZero) < ESC_DEADZONE) {
      targetPPM = ESC_STOP;
    }

    //update brakeLights if throttle is pressed
    brakeLightState = (targetPPM < ESC_STOP) ? BRAKELIGHT_BRAKING : (brakeLightState != BRAKELIGHT_TURNLEFT && brakeLightState != BRAKELIGHT_TURNRIGHT) ? BRAKELIGHT_NOTBRAKING : brakeLightState; //set brake light state based on whether we're below ESC_STOP threshold
  } else { //If throttle is not pressed, set target to STOP to bring us slowly back down to a stop
    targetPPM = ESC_STOP;
    //When throttle is not pressed, we literally can't be braking, so just check other conditions
    brakeLightState = (brakeLightState != BRAKELIGHT_TURNLEFT && brakeLightState != BRAKELIGHT_TURNRIGHT) ? BRAKELIGHT_NOTBRAKING : brakeLightState;
  }

  unsigned long deltaTime = currentMillis-prevLoopMillis;

  int jumpThrottUpperLimit = (ESC_STOP+PPM_JUMP_VALUE);
  int jumpThrottLowerLimit = (ESC_STOP-PPM_JUMP_VALUE);
  if (realPPM > jumpThrottUpperLimit && targetPPM < jumpThrottLowerLimit) {
    realPPM = jumpThrottLowerLimit; //Set throttle to -10% to already start braking process
  }

  if (realPPM < jumpThrottLowerLimit && targetPPM > jumpThrottUpperLimit) {
    realPPM = jumpThrottUpperLimit; //Set throttle to 10% to already start acceleration process
  }

  float fRate;
  if (targetPPM < realPPM) { //Target is less, so brake
    fRate = -(float)((realPPM > jumpThrottLowerLimit) ? PPM_WITHIN_JUMP_RATE : PPM_BRAKE_RATE); //store rate as intermediate value
  } else if (targetPPM > realPPM) { //Target is greater, so accelerate
    fRate = (float)((realPPM < jumpThrottUpperLimit) ? PPM_WITHIN_JUMP_RATE : (boostEnabled) ? PPM_ACCEL_RATE_BOOST : PPM_ACCEL_RATE_NONBOOST); //intermediate result
  } //Don't change rate if targetPPM is exactly equal to realPPM, because we are at our target

  int deltaPPM = (int)(fRate*((float)deltaTime/1000.0));

  if (abs(deltaPPM) >= 1) {
    //Do da realPPM addition
    realPPM += deltaPPM;
  }
  realPPM = constrain(realPPM, ESC_MIN, ESC_MAX); //make sure we're within limits for safety even tho it should never be an issue
  prevLoopMillis = currentMillis; //Update prevLoopMillis

  //For Human Readability

  /*Serial.print(F("Target: "));
  Serial.print(targetPPM);
  Serial.print(F(" real: "));
  Serial.println(realPPM);*/

  //For Arduino Serial Plotter

  Serial.print(targetPPM);
  Serial.print(",");
  Serial.println(realPPM);

  //Write the actual value out
  ESC.write(realPPM);

  delay(10); //10ms delay
}

//Doesn't call FastLED.show, so will only update internal state not actually write to leds
void writeBrakeLEDSSolid(CRGB color) {
  for (int i=0; i<NUM_LEDS_BRAKE; i++) {
    leds[NUM_LEDS_BOARD+i] = color;
  }
}

//Doesn't call FastLED.show, so will only update internal state not actually write to leds
void writeBoardLEDSSolid(CRGB color) {
  for (int i=0; i<NUM_LEDS_BOARD; i++) {
    leds[i] = color;
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