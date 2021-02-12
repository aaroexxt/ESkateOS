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

/**
 * Error code key
 * 100: Vesc setup error
 * 200: Radio setup error
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

// Definitions

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.println(x)
    #include "printf.h"
    const boolean debug = true;
#else
    #define DEBUG_PRINT(x)
    const boolean debug = false;
#endif

// ESC pin defs
#define ESC_PIN 5
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
#define LED_DATA_PIN 3
#define LED_TYPE WS2811
#define COLOR_ORDER BRG  // I'm using a BRG led strip which is kinda wacky
#define NUM_LEDS_BOARD 32
#define LED_BRIGHTNESS 128
#define LED_FPS 120  // Not used
#define LEDdelayShort 10
#define LEDdelayLong 100
#define LEDdelaySuperLong 500

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
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Radio
RF24 radio(7, 8);
const byte addresses[][6] = {"00001", "00002"};  // Read at addr 00001, write at addr 00002
double dataRx[3];                                // Double takes up 8 bytes, each payload is 32 bytes, so this will use 24 of the 32 bytes (no dynamic payload)
double dataTx[3];
unsigned long lastHBTime = 0;  // Time when heartbeat signal was last recieved
boolean radioListening = false;

// LEDs
CRGB leds[NUM_LEDS_BOARD];
unsigned long prevLEDMillis = 0;
int LEDdelay = LEDdelayLong;
int ledPosition = 0;  // Current position in strip for pattern

boolean error = false;
boolean throttleEnabled = false;
boolean speakerCharging = false;

int realPPM = ESC_STOP;
int realRAW = 0;
int MASTER_STATE = 0;

unsigned long prevLoopMillis = 0;


// Physical Constants
const int motorPoles = 7;
const int motorPulley = 14;
const int wheelPulley = 36;
const double gearRatio = (double)motorPulley / (double)wheelPulley;
const double wheelDiameter = 3.54331;  // Inches

// Enums
typedef enum {
    LEDSTATE_OFF = 0,
    LEDSTATE_INITCHASE = 1,
    LEDSTATE_RAINBOW = 2,
    LEDSTATE_CHGTHROTT = 3,
    LEDSTATE_ERROR = 6
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
    SPEAKER_ON_OFF = 5,

    // Board -> Controller
    SENDALLDATA = 10,
    SCREENUPDATE = 11,  // Not used
    VESCDATA = 12,      // Calc speed
    SENSDATA = 13,      // Not used
    TONE = 14           // Used once, why?
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
    radio.setRetries(3, 3);         // Delay, count
    radio.maskIRQ(1, 1, 0);         // Mask all IRQ triggers except for receive (1 is mask, 0 is no mask)

    // Skateboard writes to addr 2, reads from addr 1
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);  // Set address to recieve data
    radioRecieveMode();

    DEBUG_PRINT(F("Radio details:"));
    if (debug) {
        printf_begin();
        radio.printDetails();
    }

    if (radio.getPALevel() != RF24_PA_MAX) ledErrorCode(true, 2, 0, 0);

    // Setup IRQ
    attachInterrupt(RADIO_ISR_PIN, radioInterupt, FALLING);

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

    // Setup VESC UART
    delay(initialVESCDelay);
    DEBUG_PRINT(F("bef vesc init"));
    VUART.setSerialPort(&Serial);
    DEBUG_PRINT(F("aft vesc init"));

    if (VUART.getVescValues()) {
        DEBUG_PRINT(F("VESC intialComm: ok"));
    } else {
        ledErrorCode(true, 1, 0, 0);
        DEBUG_PRINT(F("VESC initialComm: err"));
    }
    
    // Go to state 0; waiting for connection
    transitionState(0);
}

void loop() {
    unsigned long currentMillis = millis();
    int mappedVal;
    if (currentMillis - prevLEDMillis >= LEDdelay && !error) {  // Make sure the leds are enabled
        prevLEDMillis = currentMillis;
        if (turnSignalStates == NOTTURNING) {
            if (ledState > LEDSTATE_OFF) {
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
            }
        } else {
            LEDdelay = LEDdelaySuperLong;

            if (turnSignalStates == LEDSTATE_TURNRIGHT) {
                for (int i = 0; i < NUM_LEDS_BOARD / 2; i++) {
                    leds[i] = CRGB::Red;
                }
                for (int i = NUM_LEDS_BOARD / 2; i < NUM_LEDS_BOARD; i++) {
                    leds[i] = CRGB::Black;
                }
            } else if (turnSignalStates == LEDSTATE_TURNLEFT) {
                for (int i = 0; i < NUM_LEDS_BOARD / 2; i++) {
                    leds[i] = CRGB::Black;
                }
                for (int i = NUM_LEDS_BOARD / 2; i < NUM_LEDS_BOARD; i++) {
                    leds[i] = CRGB::Red;
                }
            } else {
                writeBoardLEDSSolid(CRGB::Black);
            }
        }
    }

    // Tell library to send LED state to LEDs
    FastLED.show();

    // Have we lost connection with the controller while operating normally? welp then we should prolly cut motors
    if (currentMillis - lastHBTime >= HBTimeoutMax && MASTER_STATE == 1) {
        transitionState(2);
    }

    // # TODO: Add calc speed code to main loop

    updateESC();  // Update ESC with throttle value

    radioRecieveMode();  // Set radio back to recieve mode at end of loop
}

void radioInterupt() {
    resetDataRx();
    radio.read(&dataRx, sizeof(dataRx));

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
                case THROTTLE_VAL:  // 1 is throttle update
                    realRAW = dataRx[1];
                    break;
                case THROTTLE_SW:
                    throttleEnabled = dataRx[1];
                    break;
                case LEDMODE:                            // 2 is led mode update
                    if (dataRx[1] <= 3) {                // Sanity check for max LED state
                        if (dataRx[1] == 0 && !error) {  // If it's zero; clear everything and write
                            writeBoardLEDSSolid(CRGB::Black);
                            FastLED.show();
                        }
                        ledState = dataRx[1];
                    }
                    break;
                case TURNSIGNAL:
                    if (turnSignalStates <= 2) {
                        turnSignalStates++;
                    } else {
                        turnSignalStates -= 2;
                    }
                    break;
                case SPEAKER_ON_OFF:
                    if (!speakerCharging) {
                        speakerCharging = true;
                        digitalWrite(FETTWO_PIN, HIGH);
                    } else {
                        speakerCharging = false;
                        digitalWrite(FETTWO_PIN, LOW);
                    }
                    break;
                case HEARTBEAT:
                    radioTransmitMode();
                    resetDataTx();
                    dataTx[0] = HEARTBEAT;
                    radio.write(&dataTx, sizeof(dataTx));

                    lastHBTime = millis();
                    break;
            }
        case 2:                            // Lost connection case
            if (dataRx[0] == HEARTBEAT) {  
                DEBUG_PRINT(F("Got heartbeat signal from controller after disconnection"));
                ledState = 0;

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
            realRAW = HALL_STOP;  // Set target to 0 speed to bring us back down to 0 speed
            // DO NOT set realPPM to ESC_STOP, because this would instantly drop power to 0, meaning I could get thrown off the board

            ledState = LEDSTATE_INITCHASE;  // Go back into disconnected mode
            break;
    }
}

// TODO: Optomize later
// SAFETY-CRITICAL CODE

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
        fRate = (float)((realPPM < jumpThrottUpperLimit) ? PPM_WITHIN_JUMP_RATE : PPM_ACCEL_RATE);   // Intermediate result
    }                                                                                                // Don't change rate if targetPPM is exactly equal to realPPM, because we are at our target

    int deltaPPM = (int)(fRate * ((float)deltaTime / 1000.0));

    if (abs(deltaPPM) >= 1) {
        realPPM += deltaPPM;
    }
    realPPM = constrain(realPPM, ESC_MIN, ESC_MAX);  // Make sure we're within limits for safety
    prevLoopMillis = currentMillis;          

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

void ledErrorCode(boolean hang, int e1, int e2, int e3) {
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








// TODO: Rewrite calc speed

// double calculateSpeed() {
//     mc_configuration VCONFIG = VUART.getMotorConfiguration();  // Takes all data directly from VESC
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
//     // ID 12: VESC data [id, value]. ID 0 is speed, ID 1 is distance travelled, ID 2 is input voltage, ID 3 is batt percent
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