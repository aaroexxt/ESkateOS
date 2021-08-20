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

#include <RF24.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <nRF24L01.h>

#include "printf.h"

// Definitions

#ifdef DEBUG
#define DEBUG_PRINT(x) DEBUG_PRINT(x)
#include "printf.h"
const boolean debug = true;
#else
#define DEBUG_PRINT(x)
const boolean debug = false;
#endif

// Digital pins
#define TURN_SIGNAL_BUTTON 2
#define LED_BUTTON 3
#define AUX_PIN 4
#define RADIO_CE 5
#define RADIO_CSN 6
#define BUZZER_PIN 9
#define RADIO_ISR_PIN 10

// Analog pins
#define VBATT_PIN A0
#define HALLEFFECT_PIN A1
#define THROTT_ENABLE_BUTTON A2

// Throttle stuff
#define throttleDeadzone 4  // About 1.5% intrinsic deadzone, can be bigger on skateboard but want to minimize deadzone from controller side because it's harder to modify
#define THROTTLE_MIN 0
#define THROTTLE_MAX 255
#define THROTTLE_STOP (THROTTLE_MIN + THROTTLE_MAX) / 2
#define HALL_MIN 266
#define HALL_MAX 793
#define HALL_CENTER (HALL_MIN + HALL_MAX) / 2

// Screen constants
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// Heartbeat
#define HBInterval 125           // Send a heartbeat every 125ms, 8x per second
#define radioResendInterval 250  // Send all radio commands every 250ms, ~4x per second
#define HBTimeoutMax 750         // Max time between signals before board cuts the motors in ms

// Misc
#define displayStateOneChangeTime 4000
#define debounceDelay 300

// Hardware declaration

RF24 radio(RADIO_CE, RADIO_CSN);
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);  // Defining the type of display used (128x64)

// Variable declarations

int MASTER_STATE = 0;
int ledMode = 0;
int turnSignalMode = 0;
int dispMinUpdate = 20;  // Minimum time between display updates in ms to make sure we don't update faster than what the screen can handle
int prevThrottle = THROTTLE_STOP;
int throttle = THROTTLE_STOP;

float dataRx[3];
float dataTx[3];

boolean throttleEnabled = false;
boolean oldThrottleEnabled = false;
boolean updateDisplayFlag = false;
boolean radioListening = false;
boolean connected = false;  // Check if connected
boolean oldConnected = false;
boolean toneActive = false;

const bool displayVESCData = false;

// Buzzer
unsigned long toneExpire = 0;
unsigned long prevDispUpdateMillis = 0;

unsigned long lastLEDDebounceTime = 0;
unsigned long lastTSDebounceTime = 0;
unsigned long lastDispStateOneTime = 0;

unsigned long prevHBMillis = 0;
unsigned long prevRadioResendMillis = 0;
unsigned long lastHBTime = 0;  // Time when heartbeat signal was last recieved

// Bitmaps, XBM format
const unsigned char logo_bits[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x3c, 0x01,
    0xe0, 0x00, 0x07, 0x70, 0x18, 0x0e, 0x30, 0x18, 0x0c, 0x98, 0x99, 0x19,
    0x80, 0xff, 0x01, 0x04, 0xc3, 0x20, 0x0c, 0x99, 0x30, 0xec, 0xa5, 0x37,
    0xec, 0xa5, 0x37, 0x0c, 0x99, 0x30, 0x04, 0xc3, 0x20, 0x80, 0xff, 0x01,
    0x98, 0x99, 0x19, 0x30, 0x18, 0x0c, 0x70, 0x18, 0x0e, 0xe0, 0x00, 0x07,
    0x80, 0x3c, 0x01, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const unsigned char signal_transmitting_bits[] = {
    0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x0f,
    0x33, 0x0f, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00};

const unsigned char signal_connected_bits[] = {
    0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x09,
    0x33, 0x09, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00};

const unsigned char signal_noconnection_bits[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x09,
    0x00, 0x09, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

char displayBuffer[20];

const byte addresses[][6] = {"00003", "00004"};  // Write at addr 00001, read at addr 00002

String displayString;
typedef enum {
    DISPU_FULL = 0,
    DISPU_CONN_WAIT = 1,
    DISPU_START = 2,
    DISPU_VERSION = 3
} DISPLAY_UPDATE_TYPES;

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

// Vesc data
struct VREALTIME {
    float speed;
    float distanceTravelled;
    float inputVoltage;
    float battPercent;
};
struct VREALTIME vesc_values_realtime;

// Functions

void setup() {
    Serial.begin(115200);
    DEBUG_PRINT(F("Eskate controller setup begin"));

    pinMode(HALLEFFECT_PIN, INPUT);
    pinMode(THROTT_ENABLE_BUTTON, INPUT);
    pinMode(VBATT_PIN, INPUT);
    pinMode(TURN_SIGNAL_BUTTON, INPUT);
    pinMode(LED_BUTTON, INPUT);
    pinMode(AUX_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    DEBUG_PRINT(F("Pin config: ok"));

    // Initialize display
    u8g2.begin();
    DEBUG_PRINT(F("OLED display: ok"));

    // Setup radio
    radio.begin();
    radio.setPALevel(RF24_PA_MAX);  // Max because we don't want to lose connection
    radio.setRetries(3, 3);         // Delay, count

    // CONTROLLER Writes to addr 1, reads from addr 2
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);  // Set address to recieve data
    radioTransmitMode();

    DEBUG_PRINT(F("Radio details:"));
    if (debug) {
        printf_begin();
        radio.printDetails();
    }

    if (radio.getPALevel() != RF24_PA_MAX) {
        DEBUG_PRINT(F("Radio: PA level not set to max"));
        hang();
    }

    radioRecieveMode();

    DEBUG_PRINT(F("Setup radio: ok"));

    updateDisplay(DISPU_START);
    updateDisplay(DISPU_VERSION);
    DEBUG_PRINT(F("Start screen going up"));
    tone(BUZZER_PIN, 3830);  // Play a c note
    delay(200);
    tone(BUZZER_PIN, 3400);  // Play a d note
    delay(200);
    tone(BUZZER_PIN, 3038);  // Play a e note
    delay(200);
    noTone(BUZZER_PIN);
    delay(400);
    transitionState(0);  // Make sure to call transitionState to update screen
}

void loop() {
    radioRecieveMode();
    if (radio.available()) {
        resetDataRx();
        radio.read(&dataRx, sizeof(dataRx));

        if (MASTER_STATE == 0 && dataRx[0] == 200) {
            DEBUG_PRINT(F("Got first heartbeat signal from board"));
            connected = true;  // Set connected flag
            transitionState(1);
        }

        boolean vvNew = false;  // Keep track of new data from vesc; did it actually arrive

        switch ((int)dataRx[0]) {
            case HEARTBEAT:  // Board -> remote heartbeats
                lastHBTime = millis();
                break;
            case SENDALLDATA:
                sendAllRadioCommands();
                break;
            case VESCDATA:  // ID 0 is speed, ID 1 is distance travelled, ID 2 is input voltage, ID 3 is batt percent
                switch ((int)dataRx[1]) {
                    case 0:
                        vesc_values_realtime.speed = dataRx[2];
                        vvNew = true;
                        break;
                    case 1:
                        vesc_values_realtime.battPercent = dataRx[2];
                        vvNew = true;
                        break;
                }
                break;
            case TONE:
                asynchTone(dataRx[1], dataRx[2]);  // Tone, time
                break;
        }
        if (vvNew && displayVESCData) {
            DEBUG_PRINT(F("New VESC data recieved\nSpeed: "));
            DEBUG_PRINT(String(vesc_values_realtime.speed));
            DEBUG_PRINT(F("Distance travelled: "));
            DEBUG_PRINT(String(vesc_values_realtime.distanceTravelled));
            DEBUG_PRINT(F("Input voltage: "));
            DEBUG_PRINT(String(vesc_values_realtime.inputVoltage));
            DEBUG_PRINT(F("Batt percent: "));
            DEBUG_PRINT(String(vesc_values_realtime.battPercent));
        }
    }
    // Check/update throttle state, since that should happen no matter what because of safety

    switch (MASTER_STATE) {
        case 0:  // 0 is waiting for board response because hb signals are being sent constantly
            radioRecieveMode();
            if (digitalRead(THROTT_ENABLE_BUTTON)) {
                transitionState(1);
            }
            break;

        case 1:                                                   // Normal operation
            throttleEnabled = digitalRead(THROTT_ENABLE_BUTTON);  // Because of input pullup, invert inputs (since it'll be pulled to ground if high)

            if (throttleEnabled != oldThrottleEnabled) {
                updateDisplayFlag = true;  // Set display update flag for next timer cycle
                DEBUG_PRINT(F("ThrottleEnChgState:"));
                DEBUG_PRINT(throttleEnabled);

                // Now send the data since there's been an update
                radioTransmitMode();
                resetDataTx();
                dataTx[0] = THROTTLE_SW;  // Sw update
                dataTx[1] = (throttleEnabled) ? 1 : 0;
                radio.write(&dataTx, sizeof(dataTx));
                oldThrottleEnabled = throttleEnabled;
            }

            // Update hall effect sensor
            int measurement = 0;
            for (int i = 0; i < 10; i++) {  // Take average reading over 10 samples to reduce noise
                measurement += analogRead(HALLEFFECT_PIN);
            }
            measurement /= 10;

            if (measurement >= HALL_CENTER) {                                                            // If true, we're going forward = >127 value
                int forwardVal = map(measurement, HALL_CENTER, HALL_MAX, THROTTLE_STOP, THROTTLE_MAX);   // Map from middle to max (127-255)
                throttle = constrain(forwardVal, THROTTLE_STOP, THROTTLE_MAX);                           // Make sure it's in range
            } else {                                                                                     // If false, we're going backward
                int backwardVal = map(measurement, HALL_MIN, HALL_CENTER, THROTTLE_MIN, THROTTLE_STOP);  // Map from min to middle (0, 127)
                throttle = constrain(backwardVal, THROTTLE_MIN, THROTTLE_STOP);                          // Make sure it's in range
            }

            if (abs(throttle - THROTTLE_STOP) < throttleDeadzone) {  // Use deadzone of throttleDeadzone%
                throttle = THROTTLE_STOP;                            // Just set it to 0 if it's not enabled or within deadzone
            }
            if (throttle != prevThrottle) {
                // Update display
                if (abs(throttle - prevThrottle) > 2) {  // Because display updates are kinda annoying, try to prevent as many as we can. make sure difference is at least 5
                    updateDisplayFlag = true;            // Set display update flag for next timer cycle
                    DEBUG_PRINT(F("HallChgState:"));
                    DEBUG_PRINT(throttle);
                }

                // Send position to board
                radioTransmitMode();
                resetDataTx();
                dataTx[0] = THROTTLE_VAL;  // Throttle update
                dataTx[1] = throttle;
                radio.write(&dataTx, sizeof(dataTx));

                prevThrottle = throttle;
            }

            // LEDs
            int LEDReading = digitalRead(LED_BUTTON);  // Input pullup invert inputs (since it'll be pulled to ground if high)
            if (LEDReading && (millis() - lastLEDDebounceTime) > debounceDelay) {
                lastLEDDebounceTime = millis();
                ledMode++;
                // There are 4 LED states 0-3 if ledMode goes above 3 it should loop back to 0
                if (ledMode == 4) {
                    ledMode = 0;
                }
                // Update display
                updateDisplayFlag = true;  // Set display update flag for next timer cycle
                DEBUG_PRINT(F("LEDChgState:"));
                DEBUG_PRINT(ledMode);

                // Now send the data since there's been an update
                radioTransmitMode();
                resetDataTx();
                dataTx[0] = LEDMODE;  //led update
                dataTx[1] = ledMode;
                radio.write(&dataTx, sizeof(dataTx));
            }

            // Turn Signals
            int turnSignalReading = digitalRead(TURN_SIGNAL_BUTTON);  // Input pullup invert inputs (since it'll be pulled to ground if high)
            if (turnSignalReading && (millis() - lastTSDebounceTime) > debounceDelay) {
                lastTSDebounceTime = millis();
                turnSignalMode++;

                if (turnSignalMode == 3) {
                    turnSignalMode = 0;
                }
                // Update display
                updateDisplayFlag = true;  // Set display update flag for next timer cycle
                DEBUG_PRINT(F("LEDChgState to turn signal:"));
                DEBUG_PRINT(turnSignalMode);

                // Now send the data since there's been an update
                radioTransmitMode();
                resetDataTx();
                dataTx[0] = TURNSIGNAL;
                dataTx[1] = turnSignalMode;
                radio.write(&dataTx, sizeof(dataTx));
            }

            if (millis() - prevRadioResendMillis >= radioResendInterval) {  // Check if we should send all radio commands
                sendAllRadioCommands();                                     // Don't need to send second HB signal because it was already send in sendAllRadioCommands
                prevHBMillis = millis();
                prevRadioResendMillis = millis();
            }
            break;
    }

    if (millis() - prevHBMillis >= HBInterval) {  // Every HBInterval ms send a new heartbeat to board (if it isn't already sent
        radioTransmitMode();
        resetDataTx();
        dataTx[0] = HEARTBEAT;
        radio.write(&dataTx, sizeof(dataTx));
        prevHBMillis = millis();
    }

    if (toneActive && millis() > toneExpire) {
        noTone(BUZZER_PIN);
        toneActive = false;
    }

    // Check if controller is still connected (hearbeat signal present within time?)
    connected = (millis() - lastHBTime >= HBTimeoutMax) ? false : true;
    if (oldConnected != connected) {  // Only update on change
        updateDisplayFlag = true;
    }
    oldConnected = connected;

    // Perform display update if enough time has elapsed
    if (millis() - prevDispUpdateMillis >= dispMinUpdate && (updateDisplayFlag || millis() - lastDispStateOneTime > displayStateOneChangeTime) && MASTER_STATE != 0) {
        updateDisplayFlag = false;
        prevDispUpdateMillis = millis();
        updateDisplay(DISPU_FULL);
    }
    radioRecieveMode();  // Default to recieve mode so as to not drop transmissions
}

void transitionState(int newState) {
    MASTER_STATE = newState;
    DEBUG_PRINT(F("New state: "));
    DEBUG_PRINT(newState);
    switch (newState) {
        case 0:
            updateDisplay(DISPU_CONN_WAIT);
            break;
        case 1:
            updateDisplay(DISPU_FULL);
            break;
    }
}

void sendAllRadioCommands() {  // Sends all commands to board
    radioTransmitMode();

    resetDataTx();
    dataTx[0] = HEARTBEAT;
    radio.write(&dataTx, sizeof(dataTx));

    resetDataTx();
    dataTx[0] = THROTTLE_SW;  // Throttle switch update
    dataTx[1] = (throttleEnabled) ? 1 : 0;
    radio.write(&dataTx, sizeof(dataTx));

    resetDataTx();
    dataTx[0] = THROTTLE_VAL;  // Throttle update
    dataTx[1] = prevThrottle;
    radio.write(&dataTx, sizeof(dataTx));
}

void asynchTone(int pitch, int time) {  // Time in ms
    tone(BUZZER_PIN, pitch);            // A5 note
    toneExpire = millis() + time;
    toneActive = true;
}

void updateDisplay(DISPLAY_UPDATE_TYPES d) {  // A lot of help for this: https://github.com/olikraus/u8glib/wiki/tpictureloop and this http://henrysbench.capnfatz.com/henrys-bench/u8glib-graphics-library-user-guide/u8glib-arduino-oled-tutorial-1-hello-world-on-steroids/
    int x = 0;
    int y = 0;
    // Draw throttle, page, batt, signal
    u8g2.firstPage();
    do {
        switch (d) {
            case DISPU_CONN_WAIT:
                u8g2.setFont(u8g2_font_helvB12_tr);
                u8g2.drawStr(0, 13, "Waiting for");
                u8g2.drawStr(0, 28, "connection...");
                u8g2.setFont(u8g2_font_helvR10_tr);
                u8g2.drawStr(0, 48, "Autoconnect or");
                u8g2.drawStr(0, 64, "click to unlock");

                break;
            case DISPU_START:
                u8g2.setFont(u8g2_font_helvR10_tr);
                u8g2.drawXBM(4, 4, 24, 24, logo_bits);
                u8g2.drawStr(34, 22, "EskateOS V2");
                u8g2.drawStr(5, 50, "By Aaron Becker & Ilan Rosenbaum");
                break;
            case DISPU_VERSION:
                u8g2.setFont(u8g2_font_logisoso22_tn);
                u8g2.drawStr(5, 50, "V2.0");
                break;
            case DISPU_FULL:
                // TODO: Should display battery level % controller AND board, Speed in MPH, Signal in 5 bars, and battery voltage
                // x and y are positions on OLED in pixels

                
                x = 88;
                y = 9;

                String suffix = F("%");
                String prefix = F("C -");
                float value = 90;
                
                // Display perfix
                displayString = prefix;
                displayString.toCharArray(displayBuffer, 4);
                u8g2.setFont(u8g2_font_profont12_tr);
                u8g2.drawStr(x - 22, y, displayBuffer);         

                // Display suffix
                displayString = suffix;
                displayString.toCharArray(displayBuffer, 3);
                u8g2.setFont(u8g2_font_profont12_tr);
                u8g2.drawStr(x + 12, y, displayBuffer);

                // Display numbers
                displayString = value;
                displayString.toCharArray(displayBuffer, 3);
                u8g2.setFont(u8g2_font_profont12_tr);
                u8g2.drawStr(x, y, displayBuffer);
                

                // Controller Battery Box

                x = 108;
                y = 0;

                u8g2.drawFrame(x + 2, y, 18, 9);
                u8g2.drawBox(x, y + 2, 2, 5);

                
                for (int i = 0; i < 5; i++) {
                    int p = round((100 / 5) * i);
                    if (p <= vesc_values_realtime.battPercent) {
                        u8g2.drawBox(x + 4 + (3 * i), y + 2, 2, 5);
                    }
                }

                x = 88;
                y = 23;

                suffix = F("%");
                prefix = F("B -");
                value = 90;
                
                // Display perfix
                displayString = prefix;
                displayString.toCharArray(displayBuffer, 5);
                u8g2.setFont(u8g2_font_profont12_tr);
                u8g2.drawStr(x - 22, y, displayBuffer);         

                // Display suffix
                displayString = suffix;
                displayString.toCharArray(displayBuffer, 3);
                u8g2.setFont(u8g2_font_profont12_tr);
                u8g2.drawStr(x + 12, y, displayBuffer);    



                // Display numbers
                displayString = value;
                displayString.toCharArray(displayBuffer, 3);
                u8g2.setFont(u8g2_font_profont12_tr);
                u8g2.drawStr(x, y, displayBuffer);

                // Board Battery Box
                x = 108;
                y = 14;

                u8g2.drawFrame(x + 2, y, 18, 9);
                u8g2.drawBox(x, y + 2, 2, 5);

                
                for (int i = 0; i < 5; i++) {
                    int p = round((100 / 5) * i);
                    if (p <= vesc_values_realtime.battPercent) {
                        u8g2.drawBox(x + 4 + (3 * i), y + 2, 2, 5);
                    }
                }

                // SIGNAL INDICATOR

                x = 114;
                y = 55;

                if (connected == true) {
                    if (throttleEnabled) {
                        u8g2.drawXBM(x, y, 12, 12, signal_transmitting_bits);
                    } else {
                        u8g2.drawXBM(x, y, 12, 12, signal_connected_bits);
                    }
                } else {
                    u8g2.drawXBM(x, y, 12, 12, signal_noconnection_bits);
                }

                // THROTTLE INDICATOR

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

        }
    } while (u8g2.nextPage());
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

// Purposefully stop code from continuing to run
void hang() {
    while (true) {
    };
}
