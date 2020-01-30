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
#include <VescUart.h> //Thank you to Gian Marcov for this awesome library: https://github.com/gianmarcov/arduino_vesc. Modified by me (Aaron Becker) to use SoftwareSerial instead of HardwareSerial
#include "printf.h"

//#include <Adafruit_Sensor.h>
//#include <Adafruit_LSM303_U.h>
//#include <Adafruit_BMP085_U.h>
//#include <Adafruit_10DOF.h>

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
Servo ESC_RIGHT; //Create FSESC "servo" output
#define ESC_R_PIN 5

#define ESC_MIN 800
#define ESC_NONBOOST_MAX 1700
#define ESC_MAX 2000
#define ESC_STOP (ESC_MIN+ESC_MAX)/2

#define HALL_MIN 0
#define HALL_MAX 255
#define HALL_STOP (HALL_MAX+HALL_MIN)/2
int realPPM = ESC_STOP;
int realRAW = 0;
unsigned long prevSpeedMillis = 0;
boolean throttleEnabled = false;
boolean boostEnabled = false;

VescUart VUART;
float VRATIO_RPM_SPEED;
float VRATIO_TACHO_KM;
float VRATIO_KM_MI;
const boolean unitsInKM = false; //are units in kilometers?
float VBATT_MAX;
float VBATT_MIN;
unsigned long prevVUpdateMillis = 0;
#define VUpdateMillis 500 //time between vesc updates

#define initialVESCCheckDelay 5000
boolean initialVESCCheck = false;

//Led pins/defs
#define LED_DATA_PIN    3
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    32
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

//IMU pins/defs
//Adafruit_10DOF                dof   = Adafruit_10DOF();
//Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
//Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
//Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
//const float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; //sea level pressure in HPA
const char ACCEL_AXIS = 'x'; //axis that board accelerates along; used for accel math

unsigned long prevSUpdateMillis = 0;
#define SUpdateMillis 500

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

  //Board -> Controller
  SENDALLDATA = 10,
  SCREENUPDATE = 11,
  VESCDATA = 12,
  SENSDATA = 13,
  TONE = 14
} RADIO_COMMANDS;

double dataRx[3]; //double takes up 8 bytes, each payload is 32 bytes, so this will use 24 of the 32 bytes
double dataTx[3];
unsigned long lastHBTime = 0; //time when heartbeat signal was last recieved
#define HBTimeoutMax 750 //max time between signals before board cuts the motors in ms
boolean radioListening = false;

//General pins/defs
int MASTER_STATE = 0;

void setup() {
  while (!Serial){;}

  Serial.begin(115200);
  Serial.println(F("ESKATEINIT_setup begin. By Aaron Becker"));

  //Setup ESC
  ESC_RIGHT.attach(ESC_R_PIN);
  ESC_RIGHT.write(ESC_STOP);
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
  FastLED.addLeds<LED_TYPE,LED_DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear();
  for (int i=0; i<NUM_LEDS; i++) {
    leds[i] = CRGB::White; //set all leds to be off
  }
  FastLED.show();
  DEBUG_PRINT(F("Setup leds: ok"));

  //Setup accelerometer
  /*if(!accel.begin())
  {
    // There was a problem detecting the ADXL345 ... check your connections
    Serial.println(F("Setup accel: fail. not detected :("));
    while(1);
  }
  if(!mag.begin())
  {
    // There was a problem detecting the LSM303 ... check your connections
    Serial.println("Setup mag: fail. not detected :(");
    while(1);
  }
  if(!bmp.begin())
  {
    // There was a problem detecting the BMP085 ... check your connections
    Serial.print("Setup bmp: fail. not detected :(");
    while(1);
  }*/
  DEBUG_PRINT("Setup accel/mag/bmp: ok");
  //Setup VESC UART
  DEBUG_PRINT(F("bef vesc init"));
  VUART.setSerialPort(&Serial);
  DEBUG_PRINT(F("aft vesc init"));

  calculateRatios();

  DEBUG_PRINT(F("Setup VESC: ok")); //TODO add check if it's null or 0 so it can fail properly

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

          delay(200);
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
        /*if (dataRx[0] != 200) {
          DEBUG_PRINT("RECV COMM ID: "+String(dataRx[0]));
        }*/
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
          case BOOSTMODE:
            boostEnabled = dataRx[1];
            break;
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
            // DEBUG_PRINT("controller hb recieved");
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
        int mappedVal = map(realPPM, ESC_MIN, ESC_MAX, 0, 255);
        for (int i=0; i<NUM_LEDS; i++) {
          leds[i] = CRGB(mappedVal, 0, 255-mappedVal);
        }
        FastLED.show();
        break;
    }
  }

  if (currentMillis-lastHBTime>=HBTimeoutMax && MASTER_STATE == 1) { //have we lost connection with the controller while operating normally? welp then we should prolly cut motors
    transitionState(2);
  }

  if (currentMillis-prevVUpdateMillis >= VUpdateMillis && MASTER_STATE == 1) { //Time for VESC update
    prevVUpdateMillis = currentMillis;
    sendVESCData();    
  }

  if (currentMillis-prevSUpdateMillis >= SUpdateMillis && MASTER_STATE == 1) { //Time for sensor update
    prevSUpdateMillis = currentMillis;
    sendSensorData();
  }

  if (currentMillis>initialVESCCheckDelay && !initialVESCCheck) { //VESC status check
    initialVESCCheck = true;
    if (VUART.getVescValues()) {
      for (int i=0; i<NUM_LEDS; i++) {
        leds[i] = CRGB::Blue; //set led to be green to indicate success vesc
      }
      FastLED.show();
      DEBUG_PRINT(F("VESC comm 1 ok"));
      delay(500);
    } else {
      for (int i=0; i<NUM_LEDS; i++) {
        leds[i] = CRGB::Red; //set led to be red to indicate failure vesc
      }
      FastLED.show();
      DEBUG_PRINT(F("VESC comm 1 err"));
      delay(500);
    }
  }

  updateESC(); //Update ESC with throttle value

  radioRecieveMode(); //Set radio back to recieve mode at end of loop
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calculateRatios() { //seperate function to save ram
  // DEBUG_PRINT("vuart MC begin");
  // mc_configuration VCONFIG = VUART.getMotorConfiguration(); //takes all data directly from VESC
  // float gearRatio = VCONFIG.si_gear_ratio;
  // int wheelDiameter = VCONFIG.si_wheel_diameter;
  // int motorPoles = VCONFIG.si_motor_poles;
  // int batteryCells = VCONFIG.si_battery_cells;
  // DEBUG_PRINT("vuart MC done");


  //Hardcoded lol because oof old vesc library
  int motorPulley = 14;
  int wheelPulley = 36;
  float gearRatio = (float)motorPulley / (float)wheelPulley;
  int wheelDiameter = 90; //in mm
  int motorPoles = 14;
  int batteryCells = 10;
  DEBUG_PRINT(F("Vrat: hard ok"));

  //The following code yoinked basically directly from https://github.com/SolidGeek/nRF24-Esk8-Remote/blob/master/transmitter/transmitter.ino. Thanks SolidGeek :)
  VRATIO_RPM_SPEED = (gearRatio * 60.0 * (float)wheelDiameter * 3.14156) / (((float)motorPoles / 2.0) * 1000000.0); // ERPM to Km/h
  VRATIO_TACHO_KM = (gearRatio * (float)wheelDiameter * 3.14156) / (((float)motorPoles * 3.0) * 1000000.0); // Pulses to km travelled
  VRATIO_KM_MI = 0.6214;
  VBATT_MAX = (float)batteryCells*4.2; //assuming max charge of 4.2V per cell
  VBATT_MIN = (float)batteryCells*3.2; //assuming min charge of 3.2V per cell
}

void sendVESCData() {
  radioTransmitMode();
  resetDataTx();
  //ID 12: VESC data [id, value]. ID 0 is speed, ID 1 is distance travelled, ID 2 is input voltage, ID 3 is batt percent
  float distance, speed, inpVoltage, battPercent;

  if (VUART.getVescValues()) {
    DEBUG_PRINT(F("Units in "));
    DEBUG_PRINT((unitsInKM?"kilometers":"miles"));
    distance = (float)VUART.data.tachometerAbs*VRATIO_TACHO_KM*(unitsInKM?VRATIO_KM_MI:1);
    DEBUG_PRINT(F("Distance travelled (from tachometer): "));
    DEBUG_PRINT(String(distance));

    speed = (float)VUART.data.rpm*VRATIO_RPM_SPEED*(unitsInKM?VRATIO_KM_MI:1);
    DEBUG_PRINT(F("Current speed (from rpm): "));
    DEBUG_PRINT(String(speed));

    inpVoltage = (float)VUART.data.inpVoltage;
    DEBUG_PRINT(F("Current inpVoltage: "));
    DEBUG_PRINT(String(inpVoltage));

    battPercent = mapFloat(inpVoltage, VBATT_MIN, VBATT_MAX, 0, 100);
    battPercent = (battPercent < VBATT_MIN) ? VBATT_MIN : (battPercent > VBATT_MAX) ? VBATT_MAX : battPercent;
    DEBUG_PRINT(F("Current battPercent: "));
    DEBUG_PRINT(String(battPercent));
  } else {
    DEBUG_PRINT(F("Vesc data get fail"));
    distance = -1.0;
    speed = -1.0;
    inpVoltage = -1.0;
    battPercent = -1.0;
  }

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
  dataTx[2] = battPercent;
  radio.write(&dataTx, sizeof(dataTx)); 
}

void sendSensorData() {
  return;
  radioTransmitMode();
  resetDataTx();

  //Define variables to store accel events
  /*sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;*/
  //Floats to store data
  float pitch, roll, heading, acceleration, temperature, altitude = -1;

  /*accel.getEvent(&accel_event);
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
      acceleration = accel_event.acceleration.x;
      break;
    case 'y':
      acceleration = accel_event.acceleration.y;
      break;
    case 'z':
      acceleration = accel_event.acceleration.z;
      break;
  }
  DEBUG_PRINT(F("Acceleration along direction of travel (axis, value):"));
  DEBUG_PRINT(ACCEL_AXIS);
  DEBUG_PRINT(acceleration);

  roll = orientation.roll;
  DEBUG_PRINT(F("Roll value: "));
  DEBUG_PRINT(roll);
  pitch = orientation.pitch;
  DEBUG_PRINT(F("Pitch value: "));
  DEBUG_PRINT(pitch);
  heading = orientation.heading;
  DEBUG_PRINT(F("Heading value: "));
  DEBUG_PRINT(heading);

  bmp.getEvent(&bmp_event);
  if (!bmp_event.pressure) {
    DEBUG_PRINT(F("Error getting bmp pressure"));
    return;
  }
  bmp.getTemperature(&temperature);
  altitude = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
  DEBUG_PRINT(F("Temperature value: "));
  DEBUG_PRINT(temperature);
  DEBUG_PRINT(F("Altitude value: "));
  DEBUG_PRINT(altitude);*/

  //ID 13: Sensor data [id, value]. ID 0 is pitch, ID 1 is roll, ID 2 is heading, ID 3 is acceleration, ID 4 is temperature, ID 5 is altitude
  dataTx[0] = 13;
  dataTx[1] = 0;
  dataTx[2] = pitch;
  radio.write(&dataTx, sizeof(dataTx));

  dataTx[1] = 1;
  dataTx[2] = roll;
  radio.write(&dataTx, sizeof(dataTx));

  dataTx[1] = 2;
  dataTx[2] = heading;
  radio.write(&dataTx, sizeof(dataTx));

  dataTx[1] = 3;
  dataTx[2] = acceleration;
  radio.write(&dataTx, sizeof(dataTx));
  
  dataTx[1] = 4;
  dataTx[2] = temperature;
  radio.write(&dataTx, sizeof(dataTx)); 

  dataTx[1] = 5;
  dataTx[2] = altitude;
  radio.write(&dataTx, sizeof(dataTx));
}

void transitionState(int newState) {
  MASTER_STATE = newState;
  DEBUG_PRINT(F("New state: "));
  DEBUG_PRINT(newState);
  switch (newState) {
    case 1:
      if (ledState == 0) { //enable/disable the leds based on what's going on
        FastLED.clear();
        FastLED.show();
      }
      break;
    case 2: //uhoh we are going into remote disconnect mode
      DEBUG_PRINT(F("Uhoh we've lost connection to the remote :("));
      realRAW = HALL_STOP; //set target to 0 speed to bring us back down to 0 speed
      ledState = 1; //go back into disconnected mode
      realPPM = ESC_STOP; //set target to 0 speed to bring us back down to 0 speed
      break;
  }  
}

void updateESC() {
  if (abs(realPPM-ESC_STOP) < 50) {
    realPPM = ESC_STOP;
  }

  if (throttleEnabled) {
    realRAW = constrain(realRAW, HALL_MIN, HALL_MAX); //constrain raw value
    realPPM = map(realRAW, HALL_MIN, HALL_MAX, ESC_MIN, (boostEnabled) ? ESC_MAX : ESC_NONBOOST_MAX); //calculate ppm
    realPPM = constrain(realPPM, ESC_MIN, ESC_MAX); //make sure we're within limits for safety even tho it should never be an issue
  } else {
    realPPM = ESC_STOP;
  }

  Serial.println(realPPM);
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
