/***************
 ______     ______     __  __     ______     ______   ______        ______     ______     ______     ______     _____    
/\  ___\   /\  ___\   /\ \/ /    /\  __ \   /\__  _\ /\  ___\      /\  == \   /\  __ \   /\  __ \   /\  == \   /\  __-.  
\ \  __\   \ \___  \  \ \  _"-.  \ \  __ \  \/_/\ \/ \ \  __\      \ \  __<   \ \ \/\ \  \ \  __ \  \ \  __<   \ \ \/\ \ 
 \ \_____\  \/\_____\  \ \_\ \_\  \ \_\ \_\    \ \_\  \ \_____\     \ \_____\  \ \_____\  \ \_\ \_\  \ \_\ \_\  \ \____- 
  \/_____/   \/_____/   \/_/\/_/   \/_/\/_/     \/_/   \/_____/      \/_____/   \/_____/   \/_/\/_/   \/_/ /_/   \/____/ 
                                                                                         
****************

By Aaron Becker & Ilan Rosenbaum
V3.0
*/

/**
 * Error codes
 * 100: Vesc setup error
 * 200: Radio setup error
 * 300: ADC setup error
*/

#include <Wire.h>

#include "printf.h"

// LED Library
#include <FastLED.h>

// Radio
#include <RF24.h>

// vesc imports
#include <ServoTimer2.h>                       // controlling vesc speed
#include <SparkFun_ADS1015_Arduino_Library.h>  // library adafuit for reading adc data
#include <VescUart.h>

// Definitions

//Uncomment below to enable debug prints
//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.println(x)
#include "printf.h"
const boolean debug = true;
#else
#define DEBUG_PRINT(x)
const boolean debug = false;
#endif

// ESC pin defs
#define ESC_PIN 15
#define ESC_MIN 800
#define ESC_MAX 2000
#define ESC_STOP (ESC_MIN + ESC_MAX) / 2
#define ESC_DEADZONE 100

// In PPM pulses per second (Ppm range = 1200, so 100PPM = ~8.3% throttle, 5% thrott per sec is equal to 60 ppm/sec)
#define PPM_BRAKE_RATE 300
#define PPM_ACCEL_RATE 85
#define PPM_JUMP_VALUE 90
#define PPM_WITHIN_JUMP_RATE 500  // Pass through "dead zone" of throttle really quickly

#define HALL_MIN 0
#define HALL_MAX 255  // TODO: fix this, Real is 220
#define HALL_STOP (HALL_MAX + HALL_MIN) / 2
#define initialVESCDelay 5000

#define HBTimeoutMax 750  // Max time between signals before board cuts the motors in ms

// LEDs defs
#define LED_DATA_PIN 14
#define LED_TYPE WS2811
#define COLOR_ORDER BRG  // I'm using a BRG led strip which is kinda wacky
#define NUM_LEDS_BOARD 26
#define LED_BRIGHTNESS 128
#define LED_FPS 120  // Not used
#define LEDdelayShort 10
#define LEDdelayLong 100
#define LEDdelayTurnSignal 500

// Pins defs
#define INDICATOR_PIN 13  // Not used
#define FETONE_PIN 23
#define FETTWO_PIN 22
#define BUZZER_PIN 20  // Not used
#define RADIO_ISR_PIN 2

// Variable declarations

// General Hardware
ServoTimer2 ESC;  // Create FSESC "servo" output
VescUart VUART;

// Radio
RF24 radio(0, 1);
const byte addresses[][6] = {"00003", "00004"};  // Read at addr 00001, write at addr 00002
float dataRx[3];                                 // Double takes up 8 bytes, each payload is 32 bytes, so this will use 24 of the 32 bytes (no dynamic payload)
float dataTx[3];
unsigned long lastHBTime = 0;  // Time when heartbeat signal was last recieved
boolean radioListening = false;

// LEDs
CRGB leds[NUM_LEDS_BOARD];
unsigned long prevLEDMillis = 0;
int LEDdelay = LEDdelayLong;
int ledPosition = 0;  // Current position in strip for pattern
int mappedVal;

// VESC
unsigned long prevVescMillis = 0;
int vescDelay = 100;

boolean error = false;
boolean throttleEnabled = false;

int realPPM = ESC_STOP;
int realRAW = 0;
int MASTER_STATE = 0;

unsigned long prevLoopMillis = 0;

// Physical Constants
// TODO: set these variables, should be imperial units (we want MPH) delete this comment when complete
const int motorPoles = 7;
const int motorPulley = 14;  // Inches
const int wheelPulley = 36;  // Inches
const double gearRatio = (double)motorPulley / (double)wheelPulley;
const double wheelDiameter = 3.54331;  // Inches
const double VBATT_MIN = 3.2;          // Voltage
const double VBATT_MAX = 4.4;          // Voltage

// Enums
typedef enum {
    LEDSTATE_OFF = 0,
    LEDSTATE_INITCHASE = 1,
    LEDSTATE_RAINBOW = 2,
    LEDSTATE_CHGTHROTT = 3
} LEDLIGHT_STATES;
int ledState = LEDSTATE_INITCHASE;

typedef enum {
    NOTTURNING = 0,
    LEDSTATE_TURNRIGHT = 1,
    LEDSTATE_TURNLEFT = 2
} TURNSIGNAL_STATES;
int turnSignalStates = NOTTURNING;

// Data structure:
// First int is command number (enum RADIO_COMMANDS)
// Second int is value 1
// Third int is value 2 (so you can send up to four bytes of data if you want)

typedef enum {
    HEARTBEAT = 200,

    // Controller -> Board
    THROTTLE_VAL = 1,
    THROTTLE_SW = 2,
    LEDMODE = 3,
    TURNSIGNAL = 4,

    // Board -> Controller
    SENDALLDATA = 10,
    VESCDATA = 11,  // Calc speed
    TONE = 12
} RADIO_COMMANDS;

// Functions

void setup() {
    // TODO: Check if hardware getting voltage

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
    radio.setPALevel(RF24_PA_MAX);  // Max because we don't want to lose connection
    radio.setRetries(3, 3);

    // Skateboard writes to addr 2, reads from addr 1
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);  // Set address to recieve data
    radioRecieveMode();

    DEBUG_PRINT(F("Radio details:"));
    if (debug) {
        printf_begin();
        radio.printDetails();
    }

    if (radio.getPALevel() != RF24_PA_MAX) displayErrorCode(true, 2, 0, 0);

    DEBUG_PRINT(F("Setup radio: ok"));

    // Setup LEDs
    pinMode(FETONE_PIN, OUTPUT);     // Mosfet shit lmao
    digitalWrite(FETONE_PIN, HIGH);  // Set both mosfet channel to be on
    FastLED.addLeds<LED_TYPE, LED_DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS_BOARD).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(LED_BRIGHTNESS);
    FastLED.clear();

    // Setup speaker
    pinMode(FETTWO_PIN, OUTPUT);
    digitalWrite(FETTWO_PIN, LOW);

    // Make sure we're displaying nothing
    writeBoardLEDSSolid(CRGB::Black);
    FastLED.show();

    DEBUG_PRINT(F("Setup leds: ok"));

    //    // Setup VESC UART
    //    delay(initialVESCDelay);
    //    DEBUG_PRINT(F("bef vesc init"));
    //    VUART.setSerialPort(&Serial);
    //    DEBUG_PRINT(F("aft vesc init"));

    //    if (VUART.getVescValues()) {
    //        DEBUG_PRINT(F("VESC intialComm: ok"));
    //    } else {
    //        displayErrorCode(true, 1, 0, 0);
    //        DEBUG_PRINT(F("VESC initialComm: err"));
    //    }

    // Go to state 0; waiting for connection
    transitionState(0);
}

void loop() {
    //    if (currentMillis - prevVescMillis >= vescDelay && !error) {
    //        prevVescMillis = currentMillis;
    //        sendVESCData();
    //    }
    radioRecieveMode();
    if (radio.available()) {
        Serial.println("RAD RECV");
        resetDataRx();
        radio.read(&dataRx, sizeof(dataRx));
        Serial.println(dataRx[0]);

        switch (MASTER_STATE) {
            // Waiting for first hb signal from controller
            case 0:
                if (dataRx[0] == HEARTBEAT) {  // 200 is "heartbeat" signal
                    DEBUG_PRINT(F("Got first heartbeat signal from controller; we're go"));
                    ledState = 0;  // Make sure LEDs are off

                    radioTransmitMode();
                    resetDataTx();
                    dataTx[0] = HEARTBEAT;
                    radio.write(&dataTx, sizeof(dataTx));  // Send one back

                    delay(50);
                    dataTx[0] = TONE;
                    dataTx[1] = 2550;                      // Tone (g)
                    dataTx[2] = 200;                       // Time
                    radio.write(&dataTx, sizeof(dataTx));  // Send pitch command

                    lastHBTime = millis();
                    transitionState(1);
                }
                break;
            case 1:  // Standard operation
                switch ((int)dataRx[0]) {
                    case THROTTLE_VAL:
                        realRAW = dataRx[1];
                        break;
                    case THROTTLE_SW:
                        throttleEnabled = dataRx[1];
                        break;
                    case LEDMODE:
                        ledState = dataRx[1];
                        break;
                    case TURNSIGNAL:
                        turnSignalStates = dataRx[1];
                        break;
                    case HEARTBEAT:
                        lastHBTime = millis();
                        radioTransmitMode();
                        resetDataTx();
                        dataTx[0] = HEARTBEAT;
                        radio.write(&dataTx, sizeof(dataTx));
                        break;
                }
            case 2:  // Lost connection case
                if (dataRx[0] == HEARTBEAT) {
                    DEBUG_PRINT(F("Got heartbeat signal from controller after disconnection"));
                    // ledState = 0;

                    radioTransmitMode();
                    resetDataTx();
                    dataTx[0] = SENDALLDATA;  // Set sendAllData flag (SAD flag) on controller to poll all values
                    radio.write(&dataTx, sizeof(dataTx));

                    lastHBTime = millis();
                    transitionState(1);  // Go back to normal operation
                }
                break;
            default:
                DEBUG_PRINT(F("Undefined state; resetting"));
                transitionState(0);
        }
    }

    // Have we lost connection with the controller while operating normally? welp then we should prolly cut motors
    if (millis() - lastHBTime >= HBTimeoutMax && MASTER_STATE == 1) {
        transitionState(2);
    }

    if (millis() - prevLEDMillis >= LEDdelay) {
        prevLEDMillis = millis();
        if (turnSignalStates == NOTTURNING) {
            switch (ledState) {
                case LEDSTATE_INITCHASE:
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
                case LEDSTATE_RAINBOW:
                    LEDdelay = LEDdelayShort;
                    fill_rainbow(leds, NUM_LEDS_BOARD, millis() / 10, 7);

                    if (realPPM > ESC_STOP) {                                 // Put a tint on if we're not braking
                        mappedVal = map(realPPM, ESC_STOP, ESC_MAX, 0, 150);  // Fade lights by 0 to 150 out of 255 in all RGB channels (towards white) based on dthrott
                        CRGB tintAmnt = CRGB(mappedVal, mappedVal, mappedVal);
                        for (int i = 0; i < NUM_LEDS_BOARD; i++) {
                            leds[i] += tintAmnt;
                        }
                    }
                    break;
                case LEDSTATE_CHGTHROTT:  // Color changes based on throttle (chaser again)
                    LEDdelay = LEDdelayShort;
                    mappedVal = map(realPPM, ESC_MIN, ESC_MAX, 255, 0);
                    for (int i = 0; i < NUM_LEDS_BOARD; i++) {
                        leds[i] = CRGB(mappedVal, 255 - mappedVal, 0);  // Modify Red and Green channel to be inverses to go between red and green as you accel (and yellow in the middle)
                    }
                    break;
            }
        } else {
            switch (turnSignalStates) {
                case LEDSTATE_TURNRIGHT:
                    LEDdelay = LEDdelayTurnSignal;
                    for (int i = 0; i < 8; i++) {
                        leds[i] = CRGB::Red;
                    }
                    for (int i = NUM_LEDS_BOARD - 5; i < NUM_LEDS_BOARD; i++) {
                        leds[i] = CRGB::Red;
                    }
                    for (int i = 8; i < (NUM_LEDS_BOARD - 5); i++) {
                        leds[i] = CRGB::Black;
                    }
                    break;
                case LEDSTATE_TURNLEFT:
                    LEDdelay = LEDdelayTurnSignal;

                    for (int i = 0; i < 8; i++) {
                        leds[i] = CRGB::Black;
                    }
                    for (int i = NUM_LEDS_BOARD - 5; i < NUM_LEDS_BOARD; i++) {
                        leds[i] = CRGB::Black;
                    }
                    for (int i = 8; i < (NUM_LEDS_BOARD - 5); i++) {
                        leds[i] = CRGB::Red;
                    }
                    break;
            }
        }
    }

    FastLED.show();

    // # TODO: Add calc speed code to main loop

    updateESC();  // Update ESC with throttle value

    radioRecieveMode();  // Set radio back to recieve mode at end of loop
}

void transitionState(int newState) {
    MASTER_STATE = newState;
    DEBUG_PRINT(F("New state: "));
    DEBUG_PRINT(newState);
    switch (newState) {
        case 0:
            ledState = LEDSTATE_INITCHASE;
            break;
        case 1:
            if (ledState == LEDSTATE_OFF && !error) {  // Enable/disable the leds based on what's going on
                FastLED.clear();
                FastLED.show();
            }
            break;
        case 2:  // We are going into remote disconnect mode
            DEBUG_PRINT(F("We've lost connection to the remote"));
            writeBoardLEDSSolid(CRGB::Red);
            FastLED.show();

            realRAW = HALL_STOP;  // Set target to 0 speed to bring us back down to 0 speed
            // DO NOT set realPPM to ESC_STOP, because this would instantly drop power to 0, meaning I could get thrown off the board

            ledState = LEDSTATE_INITCHASE;  // Go back into disconnected mode
            break;
    }
}

// TODO: Optomize later
// SAFETY-CRITICAL CODE

void updateESC() {

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

    unsigned long deltaTime = millis() - prevLoopMillis;

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
        fRate = (float)((realPPM < jumpThrottUpperLimit) ? PPM_WITHIN_JUMP_RATE : PPM_ACCEL_RATE);   // Intermediate result
    }                                                                                                // Don't change rate if targetPPM is exactly equal to realPPM, because we are at our target

    int deltaPPM = (int)(fRate * ((float)deltaTime / 1000.0));

    if (abs(deltaPPM) >= 1) {
        realPPM += deltaPPM;
    }
    realPPM = constrain(realPPM, ESC_MIN, ESC_MAX);  // Make sure we're within limits for safety
    prevLoopMillis = millis();

    // For Human Readability
    DEBUG_PRINT(F("Target: "));
    DEBUG_PRINT(targetPPM);
    DEBUG_PRINT(F(" real: "));
    DEBUG_PRINT(realPPM);
    DEBUG_PRINT(F("\n"));

    // Write the actual value out
    ESC.write(realPPM);

    delay(10);  // 10ms delay
}

void displayErrorCode(boolean hang, int e1, int e2, int e3) {
    error = true;

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
    while (hang) {
        realRAW = HALL_STOP;
    }
}

// Updates internal state but does not actually write to leds
void writeBoardLEDSSolid(CRGB color) {
    for (int i = 0; i < NUM_LEDS_BOARD; i++) {
        leds[i] = color;
    }
}

void radioRecieveMode() {
    if (!radioListening) {
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

double getSpeed() {
    if (VUART.getVescValues()) {
        return (((double)VUART.data.tachometerAbs / motorPoles) * wheelDiameter) / (gearRatio * 336);
    } else {
        DEBUG_PRINT(F("Vesc data get fail"));
        return -1.0;
    }
}

double getBattPercent() {
    if (VUART.getVescValues()) {
        float inpVoltage = (float)VUART.data.inpVoltage;
        float voltageRounded = ((inpVoltage < VBATT_MIN) ? VBATT_MIN : (inpVoltage > VBATT_MAX) ? VBATT_MAX
                                                                                                : inpVoltage);
        return mapFloat(voltageRounded, VBATT_MIN, VBATT_MAX, 0, 100);
    } else {
        DEBUG_PRINT(F("Vesc data get fail"));
        return -1.0;
    }
}

void sendVESCData() {
    radioTransmitMode();
    resetDataTx();
    // ID 12: VESC data [id, value]. ID 0 is speed, ID 1 is battery percent
    dataTx[0] = VESCDATA;
    dataTx[1] = 0;
    dataTx[2] = getSpeed();
    radio.write(&dataTx, sizeof(dataTx));

    resetDataTx();
    dataTx[0] = VESCDATA;
    dataTx[1] = 1;
    dataTx[2] = getBattPercent();
    radio.write(&dataTx, sizeof(dataTx));
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
