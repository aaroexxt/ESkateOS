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

//Radio pins/defs
RF24 radio(7, 8);
const byte addresses [][6] = {"00001", "00002"}; //write at addr 00002, read at addr 00001

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
  ESC_LEFT.write((ESC_MIN+ESC_MAX)/2); //set them to be basically off (middle position)
  ESC_RIGHT.write((ESC_MIN+ESC_MAX)/2);
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
  //SKATEBOARD Writes to addr 2, reads from addr 1
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]); //set address to recieve data
  radio.startListening(); //puts radio in listening mode (waiting for hb signal from controller)
  Serial.println("Setup radio: ok");



  pinMode(potentiometer, INPUT);
  ESC.attach(9);    
  Serial.begin(9600);  
  curval=0;
  ESC.setMinimumPulse(800);
  ESC.setMaximumPulse(2000);
}

void loop() {

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

