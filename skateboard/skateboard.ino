/***************
 ______     ______     __  __     ______     ______   ______        ______     ______     ______     ______     _____    
/\  ___\   /\  ___\   /\ \/ /    /\  __ \   /\__  _\ /\  ___\      /\  == \   /\  __ \   /\  __ \   /\  == \   /\  __-.  
\ \  __\   \ \___  \  \ \  _"-.  \ \  __ \  \/_/\ \/ \ \  __\      \ \  __<   \ \ \/\ \  \ \  __ \  \ \  __<   \ \ \/\ \ 
 \ \_____\  \/\_____\  \ \_\ \_\  \ \_\ \_\    \ \_\  \ \_____\     \ \_____\  \ \_____\  \ \_\ \_\  \ \_\ \_\  \ \____- 
  \/_____/   \/_____/   \/_/\/_/   \/_/\/_/     \/_/   \/_____/      \/_____/   \/_____/   \/_/\/_/   \/_/ /_/   \/____/ 
                                                                                         
****************

  By Aaron Becker & Ilan Rosenbaum
  V3.Ilan Eventually
*/

#include <Wire.h>
#include <imumaths.h>

#include "printf.h"

// LED Library
#include <FastLED.h>

// Radio
#include <RF24.h>

// vesc imports
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <ServoTimer2.h>                       // controlling vesc speed
#include <SparkFun_ADS1015_Arduino_Library.h>  // library adafuit for reading adc data
#include <VescUart.h>

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.println(x)
#include "printf.h"
const boolean debug = true;
#else
#define DEBUG_PRINT(x)
const boolean debug = false;
#endif

// ESC pins
ServoTimer2 ESC;  // Create FSESC "servo" output
#define ESC_PIN 5

#define ESC_MIN 800
#define ESC_NONBOOST_MAX 1700
#define ESC_MAX 2000
#define ESC_STOP (ESC_MIN + ESC_MAX) / 2
#define ESC_DEADZONE 100

// In PPM pulses per second (Ppm range = 1200, so 100PPM = ~8.3% throttle, 5% thrott per sec is equal to 60 ppm/sec)
#define PPM_BRAKE_RATE 300
#define PPM_ACCEL_RATE_NONBOOST 85
#define PPM_ACCEL_RATE_BOOST 250
#define PPM_JUMP_VALUE 90
#define PPM_WITHIN_JUMP_RATE 500  // Pass through "dead zone" of throttle really quickly

#define HALL_MIN 0
#define HALL_MAX 255  // Real is 220, TODO fix this
#define HALL_STOP (HALL_MAX + HALL_MIN) / 2
int realPPM = ESC_STOP;
int realRAW = 0;
int turnSignalCounter = 0;
unsigned long prevLoopMillis = 0;
boolean throttleEnabled = false;
boolean speakerCharging = false;
boolean turnLightsEnabled = false;
boolean oldTurnLightsEnabled = false;  // Keep track of prev state of turnLightsEnabled so that we can catch rising/falling events
#define TURNLIGHTS_TOGGLE_CLICK_COUNT 4

#define initialVESCCheckDelay 5000
boolean initialVESCCheck = false;

// Physical Constants
const int motorPoles = 7;
const int motorPulley = 14;
const int wheelPulley = 36;
const double gearRatio = (double)motorPulley / (double)wheelPulley;
const double wheelDiameter = 3.54331;  // Inches

// LEDS
#define LED_DATA_PIN 3
#define LED_TYPE WS2811
#define COLOR_ORDER BRG  // I'm using a BRG led strip which is kinda wacky
#define NUM_LEDS_BOARD 32
CRGB leds[NUM_LEDS_BOARD];
#define LED_BRIGHTNESS 128
#define LED_FPS 120
unsigned long prevLEDMillis = 0;
#define LEDdelayShort 10
#define LEDdelayLong 100
#define LEDdelaySuperLong 500
int LEDdelay = LEDdelayLong;
int ledPosition = 0;  // Current position in strip for pattern

typedef enum {
    LEDSTATE_OFF = 0,
    LEDSTATE_INITCHASE = 1,
    LEDSTATE_RAINBOW = 2,
    LEDSTATE_CHGTHROTT = 3,
    LEDSTATE_TURNRIGHT = 4,
    LEDSTATE_TURNLEFT = 5,
    LEDSTATE_ERROR = 6
} LEDLIGHT_STATES;
int ledState = LEDSTATE_INITCHASE;

// pins/defs
#define INDICATOR_PIN 13
#define FETONE_PIN 23
#define FETTWO_PIN 22
#define BUZZER_PIN 20
#define RADIO_ISR_PIN 2
// Check I2C device address and correct line below (by default address is 0x29 or 0x28) id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Radio pins/defs
RF24 radio(7, 8);
const byte addresses[][6] = {"00001", "00002"};  // Write at addr 00002, read at addr 00001
// Send 6 bytes (because each int is 2 bytes) per rx/tx

/*
Data structure:
First int is command number
Second int is value 1
Third int is value 2 (so you can send up to four bytes of data if you want)

RADIO COMMANDS MASTER
ID 200: heartbeat. Data: [0, 0]
ID 1: ThrottleVal update. Data: [raw value, 0]
ID 2: ThrottleSW update. Data: [sw, 0]
ID 3: LED mode update. Data: [ledMode (0, 1, 2), 0]
ID 4: Turn signal update. Data: [turnSignal (0, 1, 2), 0]
ID 5: Speaker charging on/off. Data: [speakerOnOff]
ID 10: Ask controller to send state of all peripherals
ID 11: Controller force screen update
ID 12: VESC data [id, value]. ID 0 is speed, ID 1 is distance travelled, ID 2 is input voltage, ID 3 is fet temp, ID 4 is batt percent
ID 13: Sensor data [id, value]. ID 0 is pitch, ID 1 is roll, ID 2 is heading, ID 3 is acceleration, ID 4 is temperature, ID 5 is altitude
ID 14: Play tone
*/

typedef enum {
    HEARTBEAT = 200,

    // Controller -> Board
    THROTTLE_VAL = 1,
    THROTTLE_SW = 2,
    LEDMODE = 3,
    TURNSIGNAL = 4,
    SPEAKER_ON_OFF = 5,

    // Board -> Controller
    SENDALLDATA = 10,
    SCREENUPDATE = 11, // Not used
    VESCDATA = 12, // Calc speed
    SENSDATA = 13, // Not used
    TONE = 14 // Used once, why?
} RADIO_COMMANDS;

double dataRx[3];  // Double takes up 8 bytes, each payload is 32 bytes, so this will use 24 of the 32 bytes (no dynamic payload)
double dataTx[3];
unsigned long lastHBTime = 0;  // Time when heartbeat signal was last recieved
#define HBTimeoutMax 750       // Max time between signals before board cuts the motors in ms
boolean radioListening = false;

// General pins/defs
int MASTER_STATE = 0;

// The board should still be able to run if either the VESC or IMU is not present, since they are not essential to operation. These variables keep track of whether they're connected correctly
boolean VESCOK = false;  // Needs to be false because it's set true if its ok later (5s delay to give vesc time to start up)
boolean SENSOK = true;   // Needs to be true because it's immediately set false if it's bad

void setup() {
    // Don't move on until serial
    while (!Serial) {
        ;
    }

    Serial.begin(115200);
    Serial.println(F("ESKATEINIT_setup begin."));

    // Setup ESC
    ESC.attach(ESC_PIN);
    ESC.write(ESC_STOP);
    DEBUG_PRINT(F("Setup esc: ok"));

    // Setup radio
    radio.begin();
    radio.setPALevel(RF24_PA_MAX);  //max because we don't want to lose connection
    radio.setRetries(3, 3);         // delay, count
    radio.maskIRQ(1, 1, 0);         // mask all IRQ triggers except for receive (1 is mask, 0 is no mask)

    // Skateboard writes to addr 2, reads from addr 1
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);  //set address to recieve data
    radioRecieveMode();
    DEBUG_PRINT(F("Radio details:"));
    if (debug) {
        printf_begin();
        radio.printDetails();
    }

    // Setup IRQ
    // Create interrupt: 0 for pin 2 or 1 for pin 3, the name of the interrupt function or ISR, and condition to trigger interrupt
    attachInterrupt(RADIO_ISR_PIN, radioInterupt, FALLING);

    DEBUG_PRINT(F("Setup radio: ok"));

    //Setup LEDS
    pinMode(FETONE_PIN, OUTPUT);     //Mosfet shit lmao
    digitalWrite(FETONE_PIN, HIGH);  //Set both mosfet channel to be on
    FastLED.addLeds<LED_TYPE, LED_DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS_BOARD).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(LED_BRIGHTNESS);
    FastLED.clear();

    //Setup Speaker
    pinMode(FETTWO_PIN, OUTPUT);
    digitalWrite(FETTWO_PIN, LOW);

    //Make sure we're displaying nothing
    writeBoardLEDSSolid(CRGB::Black);
    FastLED.show();
    DEBUG_PRINT(F("Setup leds: ok"));

    //Setup VESC UART
    /*
      DEBUG_PRINT(F("bef vesc init"));
      VUART.setSerialPort(&Serial);
      DEBUG_PRINT(F("aft vesc init"));
    */

    //Goto state 0; waiting for connection
    transitionState(0);
}

void loop() {
    switch (MASTER_STATE) {
        // Waiting for first hb signal from controller
        case 0:
            radioRecieveMode();  // Ensure we're recieving
            resetDataRx();
            if (radio.available()) {
                radio.read(&dataRx, sizeof(dataRx));
                if (dataRx[0] == HEARTBEAT) {  // 200 is "heartbeat" signal
                    DEBUG_PRINT(F("Got first heartbeat signal from controller; we're go"));
                    ledState = 0;  // Make sure LEDs are off

                    radioTransmitMode();
                    resetDataTx();
                    dataTx[0] = HEARTBEAT;
                    radio.write(&dataTx, sizeof(dataTx));  //send one back

                    delay(50);
                    dataTx[0] = TONE;
                    dataTx[1] = 2550;                      //tone (g)
                    dataTx[2] = 200;                       //time
                    radio.write(&dataTx, sizeof(dataTx));  //send pitch command

                    lastHBTime = millis();
                    transitionState(1);
                }
            }
            break; 
        case 1:  // Standard operation
            // Uses interupt method now
        case 2:                  // Lost connection case
            radioRecieveMode();  // Ensure we're recieving
            resetDataRx();
            if (radio.available()) {
                radio.read(&dataRx, sizeof(dataRx));

                if (dataRx[0] == HEARTBEAT) {  // 200 is "heartbeat" signal
                    DEBUG_PRINT(F("Got heartbeat signal from controller after disconnection"));
                    ledState = 0;  // Make sure LEDs are off

                    radioTransmitMode();
                    resetDataTx();
                    dataTx[0] = SENDALLDATA;  // Set sendAllData flag (SAD flag) on controller to poll all values
                    radio.write(&dataTx, sizeof(dataTx));

                    lastHBTime = millis();
                    transitionState(1);  // Go back to normal operation
                        }
                    }
                    break;
        default:
            DEBUG_PRINT(F("Undefined state; resetting"));
            transitionState(0);
            }

            unsigned long currentMillis = millis();
            int mappedVal;

            if (turnSignalCounter == 1) {
                ledState = LEDSTATE_TURNRIGHT;
            } else if (turnSignalCounter == 2) {
                ledState = LEDSTATE_TURNLEFT;
            }

            if (currentMillis - prevLEDMillis >= LEDdelay) {  //make sure the leds are enabled
                prevLEDMillis = currentMillis;
                if (ledState > LEDSTATE_OFF) {
                    switch (ledState) {
                        case LEDSTATE_INITCHASE:  //blue chase
                            LEDdelay = LEDdelayLong;
                            for (int i = 0; i < NUM_LEDS_BOARD; i++) {
                                if ((i + ledPosition) % 5 == 0) {
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
                        case LEDSTATE_RAINBOW:  //rainbow
                            LEDdelay = LEDdelayShort;
                            // https://github.com/marmilicious/FastLED_examples/blob/master/rainbow_brightness_and_saturation.ino
                            // Fill internal LED array w/rainbow
                            fill_rainbow(leds, NUM_LEDS_BOARD, millis() / 10, 7);

                            if (realPPM > ESC_STOP) {                                 // Put a tint on if we're not braking
                                mappedVal = map(realPPM, ESC_STOP, ESC_MAX, 0, 150);  // Fade lights by 0 to 150 out of 255 in all RGB channels (towards white) based on dthrott
                                CRGB tintAmnt = CRGB(mappedVal, mappedVal, mappedVal);
                                for (int i = 0; i < NUM_LEDS_BOARD; i++) {
                                    leds[i] += tintAmnt;
                                }
                            }
                            break;
                        case LEDSTATE_CHGTHROTT:  //color changes based on throttle (chaser again)
                            LEDdelay = LEDdelayShort;
                            mappedVal = map(realPPM, ESC_MIN, ESC_MAX, 255, 0);
                            for (int i = 0; i < NUM_LEDS_BOARD; i++) {
                                leds[i] = CRGB(mappedVal, 255 - mappedVal, 0);  //Modify Red and Green channel to be inverses to go between red and green as you accel (and yellow in the middle)
                            }
                            break;
                        case LEDSTATE_TURNRIGHT:
                            LEDdelay = LEDdelaySuperLong;
                            if (currentMillis - prevLEDMillis >= LEDdelay) {
                                for (int i = 0; i < NUM_LEDS_BOARD / 2; i++) {
                                    leds[i] = CRGB::Red;
                                }
                                for (int i = 15; i < NUM_LEDS_BOARD; i++) {
                                    leds[i] = CRGB::Black;
                                }
                            } else {
                                for (int i = 0; i < NUM_LEDS_BOARD; i++) {
                                    leds[i] = CRGB::Black;
                                }
                            }
                            break;
                        case LEDSTATE_TURNLEFT:
                            LEDdelay = LEDdelaySuperLong;
                            if (currentMillis - prevLEDMillis >= LEDdelay) {
                                for (int i = 0; i < NUM_LEDS_BOARD / 2; i++) {
                                    leds[i] = CRGB::Black;
                                }
                                for (int i = 15; i < NUM_LEDS_BOARD; i++) {
                                    leds[i] = CRGB::Red;
                                }
                            } else {
                                for (int i = 0; i < NUM_LEDS_BOARD; i++) {
                                    leds[i] = CRGB::Black;
                                }
                            }
                            break;
                    }
                }
                // Tell library to send LED state to LEDs
                FastLED.show();
            }

            // have we lost connection with the controller while operating normally? welp then we should prolly cut motors
            if (currentMillis - lastHBTime >= HBTimeoutMax && MASTER_STATE == 1) {
                transitionState(2);
            }

            if (speakerCharging == true) {
                digitalWrite(FETTWO_PIN, HIGH);
            }

            // Calculate speed
            // if (currentMillis - prevVUpdateMillis >= VUpdateMillis && MASTER_STATE == 1 && VESCOK) {  // Time for VESC update
            //     prevVUpdateMillis = currentMillis;
            //     updateVESCData();
            //     sendVESCData();
            // }

            updateESC();  //Update ESC with throttle value

            radioRecieveMode();  //Set radio back to recieve mode at end of loop
    }

    // TODO: WRITE CODE CALCULATE SPEED TIME VROOM
    // double calculateSpeed() {
    //     mc_configuration VCONFIG = VUART.getMotorConfiguration();  //takes all data directly from VESC
    //     double rpm;

    //     if (VUART.getVescValues()) {
    //         rpm = (double)VUART.data.tachometerAbs / motorPoles;
    //     }

    //     return (rpm * wheelDiameter) / (gearRatio * 336)
    // }

    // void updateVESCData() {
    //     if (VUART.getVescValues()) {
    //         DEBUG_PRINT(F("Units in "));
    //         DEBUG_PRINT((unitsInKM ? "kilometers" : "miles"));
    //         vesc_values_realtime.distanceTravelled = (float)VUART.data.tachometerAbs * VRATIO_TACHO_KM * (unitsInKM ? VRATIO_KM_MI : 1);
    //         DEBUG_PRINT(F("Distance travelled (from tachometer): "));
    //         DEBUG_PRINT(String(vesc_values_realtime.distanceTravelled));

    //         vesc_values_realtime.speed = (float)VUART.data.rpm * VRATIO_RPM_SPEED * (unitsInKM ? VRATIO_KM_MI : 1);
    //         DEBUG_PRINT(F("Current speed (from rpm): "));
    //         DEBUG_PRINT(String(vesc_values_realtime.speed));

    //         vesc_values_realtime.inputVoltage = (float)VUART.data.inpVoltage;
    //         DEBUG_PRINT(F("Current inpVoltage: "));
    //         DEBUG_PRINT(String(vesc_values_realtime.inputVoltage));

    //         float battPercent = mapFloat(vesc_values_realtime.inputVoltage, VBATT_MIN, VBATT_MAX, 0, 100);
    //         vesc_values_realtime.battPercent = (battPercent < VBATT_MIN) ? VBATT_MIN : (battPercent > VBATT_MAX) ? VBATT_MAX
    //                                                                                                              : battPercent;
    //         DEBUG_PRINT(F("Current battPercent: "));
    //         DEBUG_PRINT(String(vesc_values_realtime.battPercent));
    //     } else {
    //         DEBUG_PRINT(F("Vesc data get fail"));
    //         vesc_values_realtime.distanceTravelled = -1.0;
    //         vesc_values_realtime.speed = -1.0;
    //         vesc_values_realtime.inputVoltage = -1.0;
    //         vesc_values_realtime.battPercent = -1.0;
    //     }
    // }

    // void sendVESCData() {
    //     radioTransmitMode();
    //     resetDataTx();
    //     //ID 12: VESC data [id, value]. ID 0 is speed, ID 1 is distance travelled, ID 2 is input voltage, ID 3 is batt percent
    //     dataTx[0] = 12;
    //     dataTx[1] = 0;
    //     dataTx[2] = vesc_values_realtime.speed;
    //     radio.write(&dataTx, sizeof(dataTx));

    //     dataTx[1] = 1;
    //     dataTx[2] = vesc_values_realtime.distanceTravelled;
    //     radio.write(&dataTx, sizeof(dataTx));

    //     dataTx[1] = 2;
    //     dataTx[2] = vesc_values_realtime.inputVoltage;
    //     radio.write(&dataTx, sizeof(dataTx));

    //     dataTx[1] = 3;
    //     dataTx[2] = vesc_values_realtime.battPercent;
    //     radio.write(&dataTx, sizeof(dataTx));
    // }

    // float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    //     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // }

    void transitionState(int newState) {
        MASTER_STATE = newState;
        DEBUG_PRINT(F("New state: "));
        DEBUG_PRINT(newState);
        switch (newState) {
            case 0:
                ledState = LEDSTATE_INITCHASE;
                break;
            case 1:
                if (ledState == LEDSTATE_OFF) {  //enable/disable the leds based on what's going on
                    FastLED.clear();
                    FastLED.show();
                }
                break;
            case 2:  //uhoh we are going into remote disconnect mode
                DEBUG_PRINT(F("Uhoh we've lost connection to the remote :("));
                realRAW = HALL_STOP;  //set target to 0 speed to bring us back down to 0 speed
                //DO NOT set realPPM to ESC_STOP, because this would instantly drop power to 0, meaning I could get thrown off the board

                ledState = LEDSTATE_INITCHASE;  //go back into disconnected mode
                break;
        }
    }

/**
 * Throttle code
 * SAFETY-CRITICAL CODE
*/
    void updateESC() {
        unsigned long currentMillis = millis();

        int targetPPM = ESC_STOP;  // Initialize tPPM
        if (throttleEnabled) {
            realRAW = constrain(realRAW, HALL_MIN, HALL_MAX);                // Constrain raw value
            targetPPM = map(realRAW, HALL_MIN, HALL_MAX, ESC_MAX, ESC_MIN);  // Calculate ppm

            // Check: is where we want to be within the deadzone? If so, the target should be just to stop
            int tDistFromZero = targetPPM - ESC_STOP;  // Because of the way abs() is implemented, make a temp local variable
            if (abs(tDistFromZero) < ESC_DEADZONE) {
                targetPPM = ESC_STOP;
            }
        } else {  // If throttle is not pressed, set target to STOP to bring us slowly back down to a stop
            targetPPM = ESC_STOP;
        }

        unsigned long deltaTime = currentMillis - prevLoopMillis;

        int jumpThrottUpperLimit = (ESC_STOP + PPM_JUMP_VALUE);
        int jumpThrottLowerLimit = (ESC_STOP - PPM_JUMP_VALUE);
        if (realPPM > jumpThrottUpperLimit && targetPPM < jumpThrottLowerLimit) {
            realPPM = jumpThrottLowerLimit;  // Set throttle to -10% to already start braking process
        }

        if (realPPM < jumpThrottLowerLimit && targetPPM > jumpThrottUpperLimit) {
            realPPM = jumpThrottUpperLimit;  // Set throttle to 10% to already start acceleration process
        }

        float fRate;
        if (targetPPM < realPPM) {                                                                       // Target is less, so brake
            fRate = -(float)((realPPM > jumpThrottLowerLimit) ? PPM_WITHIN_JUMP_RATE : PPM_BRAKE_RATE);  // Store rate as intermediate value
        } else if (targetPPM > realPPM) {                                                                // Target is greater, so accelerate
            fRate = (float)((realPPM < jumpThrottUpperLimit) ? PPM_WITHIN_JUMP_RATE : PPM_ACCEL_RATE_NONBOOST);
        }  // Don't change rate if targetPPM is exactly equal to realPPM, because we are at our target

        int deltaPPM = (int)(fRate * ((float)deltaTime / 1000.0));

        if (abs(deltaPPM) >= 1) {
            // Do da realPPM addition
            realPPM += deltaPPM;
        }
        realPPM = constrain(realPPM, ESC_MIN, ESC_MAX);  // Make sure we're within limits for safety even tho it should never be an issue
        prevLoopMillis = currentMillis;                  // Update prevLoopMillis

        // For Human Readability

        
        // Serial.print(F("Target: "));
        // Serial.print(targetPPM);
        // Serial.print(F(" real: "));
        // Serial.println(realPPM);
     

        // For Arduino Serial Plotter

        Serial.print(targetPPM);
        Serial.print(",");
        Serial.println(realPPM);

        // Write the actual value out
        ESC.write(realPPM);

        delay(10);  // 10ms delay
    }

    // Doesn't call FastLED.show, so will only update internal state not actually write to leds
    void writeBoardLEDSSolid(CRGB color) {
        for (int i = 0; i < NUM_LEDS_BOARD; i++) {
            leds[i] = color;
        }
    }

    void radioRecieveMode() {
        if (!radioListening) {  // If we're not listening
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

    void radioInterupt() {
        switch ((int)dataRx[0]) {
            case THROTTLE_VAL:  // 1 is throttle update
                realRAW = dataRx[1];
                break;
            case THROTTLE_SW:
                throttleEnabled = dataRx[1];
                break;
            case TURNSIGNAL:
                if (turnSignalCounter <= 2) {
                    turnSignalCounter++;
                } else {
                    turnSignalCounter -= 2;
                }
                break;
            case SPEAKER_ON_OFF:
                if (speakerCharging = false) {
                    speakerCharging = true;
                } else {
                    speakerCharging = false;
                }
                break;
            case LEDMODE:                  // 2 is led mode update
                if (dataRx[1] <= 3) {      // Sanity check for max LED state
                    if (dataRx[1] == 0) {  // If it's zero; clear everything and write
                        writeBoardLEDSSolid(CRGB::Black);
                        FastLED.show();
                    }
                    ledState = dataRx[1];
                }
                break;
            case HEARTBEAT:  // Heartbeat. if we get one, we should send one
                radioTransmitMode();
                resetDataTx();
                dataTx[0] = HEARTBEAT;
                radio.write(&dataTx, sizeof(dataTx));

                lastHBTime = millis();
                break;
        }
    }

    void ledErrorCode(int e1, int e2, int e3) {
        if (e1 + e2 + e3 + 2 > NUM_LEDS_BOARD) {
            leds[0] = CRGB::Red;
            leds[1] = CRGB::Red;
            leds[2] = CRGB::Red;
            leds[3] = CRGB::Red;
            leds[5] = CRGB::Black;
            leds[6] = CRGB::Black;
            leds[7] = CRGB::Black;
            leds[8] = CRGB::Red;
            leds[9] = CRGB::Red;
            leds[10] = CRGB::Red;
            leds[11] = CRGB::Red;
        }
        if (e1 < NUM_LEDS_BOARD) {
            for (int i = 0; i < e1; i++) {
                leds[i] = CRGB::Red;
            }
        }
        if (e1 + e2 + 1 < NUM_LEDS_BOARD) {
            for (int i = e1 + 1; i < e1 + e2 + 1; i++) {
                leds[i] = CRGB::Red;
            }
        }
        if (e1 + e2 + e3 + 2 < NUM_LEDS_BOARD) {
            for (int i = e2 + 2; i < e1 + e2 + e3 + 2; i++) {
                leds[i] = CRGB::Red;
            }
        }

        for (int i = e1 + e2 + e3 + 2; i < NUM_LEDS_BOARD; i++) {
            leds[i] = CRGB::Black;
        }

        FastLED.show();
        delay(5000);
    }