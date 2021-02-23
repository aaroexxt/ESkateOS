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

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define BUZZER_PIN 9

// Throttle pins/setup
#define HALLEFFECT A1
#define THROTT_ENABLE_SW 6
#define throttleDeadzone 4  // About 1.5% intrinsic deadzone, can be bigger on skateboard but want to minimize deadzone from controller side because it's harder to modify
#define THROTTLE_MIN 0
#define THROTTLE_MAX 255
#define THROTTLE_STOP (THROTTLE_MIN + THROTTLE_MAX) / 2

// Hall real low 481, hall middle 633
#define HALL_MIN 500
#define HALL_MAX 1100
#define HALL_CENTER 700  // (HALL_MIN+HALL_MAX)/2

#define ENClickTimeout 1000  // Max time to register clicks

// Battery pins
#define VBATT_PIN A0

#define RADIO_ISR_PIN 2

#define displayStateOneChangeTime 4000

#define BOOST_SW 3
#define LED_1_SW 5
#define LED_2_SW 4
#define debounceDelay 50

#define HBInterval 125           // Send a heartbeat every 125ms, 8x per second
#define radioResendInterval 250  // Send all radio commands every 250ms, ~4x per secodn
#define HBTimeoutMax 750         // Max time between signals before board cuts the motors in ms

// Hardware declaration

RF24 radio(7, 8);
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);  // Defining the type of display used (128x64)

// Variable declarations

int MASTER_STATE = 0;
int ledMode = -1;
int oldLedMode = -1;
int dispMinUpdate = 20;  // Minimum time between display updates in ms to make sure we don't update faster than what the screen can handle
int prevThrottle = THROTTLE_STOP;
int throttle = THROTTLE_STOP;
int ENclicks = 0;

double dataRx[3];
double dataTx[3];

boolean throttleEnabled = false;
boolean oldThrottleEnabled = false;
boolean boostEnabled = false;
boolean oldBoostEnabled = false;
boolean updateDisplayFlag = false;
boolean toneActive = false;
boolean displayStateOne = false;  // Boolean controls what data is displayed during normal display
boolean displaySensorData = false;
boolean radioListening = false;
boolean connected = false;  // Check if connected
boolean oldConnected = false;

// Pins
const bool displayVESCData = false;
const bool displaySENSData = false;

// Buzzer
unsigned long toneExpire = 0;
unsigned long prevDispUpdateMillis = 0;

unsigned long lastLEDDebounceTime = 0;
unsigned long lastDispStateOneTime = 0;

// Click tracking of enable switch
unsigned long ENLastClickTime = 0;

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

const byte addresses[][6] = {"00001", "00002"};  // Write at addr 00001, read at addr 00002

String displayString;

// Enums

typedef enum {
    DISPU_FULL = 0,
    DISPU_CONN_WAIT = 1,
    DISPU_START = 2,
    DISPU_SENSDATA = 3,
    CLEAR = 4,
    DISPU_VERSION = 5
} DISPLAY_UPDATE_TYPES;

// Data structure:
// First int is command number (enum RADIO_COMMANDS)
// Second int is value 1
// Third int is value 2 (so you can send up to four bytes of data if you want)

typedef enum {
  HEARTBEAT = 200,

  //Controller -> Board
  THROTTLE_VAL = 1,
  THROTTLE_SW = 2,
  LEDMODE = 3,
  BOOSTMODE = 4, // Needs to become TURNSIGNAL
  CLICK = 5, // Needs to become SPEAKER_ON_OFF

  //Board -> Controller
  SENDALLDATA = 10,
  SCREENUPDATE = 11,
  VESCDATA = 12,
  SENSDATA = 13,
  TONE = 14
} RADIO_COMMANDS;

// Structs

// Vesc data
struct VREALTIME {
    float speed;
    float distanceTravelled;
    float inputVoltage;
    float battPercent;
};
struct VREALTIME vesc_values_realtime;

// Gyro/accel/temp data
struct SREALTIME {
    float pitch;
    float roll;
    float heading;
    float acceleration;
    float temperature;
    float altitude;
};
struct SREALTIME sensor_values_realtime;

// Functions

void setup() {
    Serial.begin(115200);
    DEBUG_PRINT(F("Eskate controller setup begin"));

    pinMode(HALLEFFECT, INPUT_PULLUP);
    pinMode(THROTT_ENABLE_SW, INPUT_PULLUP);
    pinMode(VBATT_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BOOST_SW, INPUT_PULLUP);
    pinMode(LED_1_SW, INPUT_PULLUP);
    pinMode(LED_2_SW, INPUT_PULLUP);

    DEBUG_PRINT(F("Pin config: ok"));

    // Initialize display
    u8g2.begin();
    DEBUG_PRINT(F("OLED display: ok"));

    // Setup radio
    if (!radio.begin()) {
        DEBUG_PRINT(F("Radio: failed"));
        hang();
    }

    radio.setPALevel(RF24_PA_MAX);  // Max because we don't want to lose connection
    radio.setRetries(3, 3);         // Delay, count
    radio.maskIRQ(1, 1, 0);         // Mask all IRQ triggers except for receive (1 is mask, 0 is no mask)

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

    // Setup IRQ
    attachInterrupt(RADIO_ISR_PIN, radioInterupt, FALLING);

    
    DEBUG_PRINT(F("Setup radio: ok"));

    updateDisplay(DISPU_START);
    DEBUG_PRINT(F("Start screen going up"));
    asynchTone(3830, 100);  // Play a c note
    asynchTone(3400, 100);  // Play a d note
    asynchTone(3038, 100);  // Play a e note
    updateDisplay(DISPU_VERSION);
    delay(1000);
    transitionState(0);  // Make sure to call transitionState to update screen
}

void loop() {
    unsigned long currentMillis = millis();  // Store millis value (current value) for reference

    // Check/update throttle state, since that should happen no matter what because of safety
    throttleEnabled = !digitalRead(THROTT_ENABLE_SW);  // Because of input pullup, invert inputs (since it'll be pulled to ground if high)
    if (throttleEnabled != oldThrottleEnabled) {
        updateDisplayFlag = true;  // Set display update flag for next timer cycle
        DEBUG_PRINT(F("ThrottleEnChgState:"));
        DEBUG_PRINT(throttleEnabled);

        if (throttleEnabled == HIGH) {  // On rising state change
            if (ENclicks == 0) {        // First click just set the last click time to currentMillis
                ENLastClickTime = currentMillis;
                ENclicks++;
            } else if (ENclicks > 0 && currentMillis < (ENLastClickTime + ENClickTimeout)) {  // Clicked within time interval
                ENclicks++;
                DEBUG_PRINT(F("Clicked "));
                DEBUG_PRINT(ENclicks);
                DEBUG_PRINT(F(" times"));

                // Now send the data since there's been an update
                radioTransmitMode();
                resetDataTx();
                dataTx[0] = CLICK;  // Click event
                dataTx[1] = ENclicks;
                radio.write(&dataTx, sizeof(dataTx));

                if (ENclicks == 3) {          // Clicked 3 times
                    if (MASTER_STATE == 0) {  // If we're in state 0, triple click should transition to normal display (for testing reasons for example)
                        transitionState(1);
                    } else {
                        updateDisplayFlag = true;
                        displaySensorData = !displaySensorData;  // Toggle display sensor data state
                    }
                }
            }
        }

        // Now send the data since there's been an update
        radioTransmitMode();
        resetDataTx();
        dataTx[0] = THROTTLE_SW;  // Sw update
        dataTx[1] = (throttleEnabled) ? 1 : 0;
        radio.write(&dataTx, sizeof(dataTx));

        oldThrottleEnabled = throttleEnabled;

        switch (MASTER_STATE) {
            case 0:  // 0 is waiting for board response because hb signals are being sent constantly
                radioRecieveMode();
                break;

            case 1:  // Normal operation
                // Update hall effect sensor
                int measurement = 0;
                for (int i = 0; i < 10; i++) {  // Take average reading over 10 samples to reduce noise
                    measurement += analogRead(HALLEFFECT);
                }
                measurement /= 10;

                // DEBUG_PRINT(measurement);

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

                boostEnabled = !digitalRead(BOOST_SW);  // Because of input pullup, invert inputs (since it'll be pulled to ground if high)
                if (boostEnabled != oldBoostEnabled) {
                    updateDisplayFlag = true;  // Set display update flag for next timer cycle
                    DEBUG_PRINT(F("BoostChgState:"));
                    DEBUG_PRINT(boostEnabled);

                    // Now send the data since there's been an update
                    radioTransmitMode();
                    resetDataTx();
                    dataTx[0] = BOOSTMODE;  // Boost update
                    dataTx[1] = (boostEnabled) ? 1 : 0;
                    radio.write(&dataTx, sizeof(dataTx));

                    oldBoostEnabled = boostEnabled;
                }

                int LEDReading1 = !digitalRead(LED_1_SW);  //because of input pullup, invert inputs (since it'll be pulled to ground if high)
                int LEDReading2 = !digitalRead(LED_2_SW);
                if ((ledMode == 2 && LEDReading2 && !LEDReading1) || (ledMode == 3 && LEDReading1 && !LEDReading2) || (ledMode == 0 && !LEDReading2 && !LEDReading1)) {
                    lastLEDDebounceTime = currentMillis;
                }
                if ((currentMillis - lastLEDDebounceTime) > debounceDelay) {  //it's been there longer than the 50ms, so just write it
                    ledMode = (LEDReading1) ? 2 : (LEDReading2) ? 3
                                                                : 0;  //switch states

                    if (oldLedMode != ledMode) {
                        //Update display
                        updateDisplayFlag = true;  //set display update flag for next timer cycle
                        DEBUG_PRINT(F("LEDChgState:"));
                        DEBUG_PRINT(ledMode);

                        //Now send the data since there's been an update
                        radioTransmitMode();
                        resetDataTx();
                        dataTx[0] = LEDMODE;  //led update
                        dataTx[1] = ledMode;
                        radio.write(&dataTx, sizeof(dataTx));

                        oldLedMode = ledMode;
                    }
                }
                break;
        }
    }

    if (currentMillis >= (ENLastClickTime + ENClickTimeout) && ENLastClickTime != 0) {  // Reset click count; time interval expired
        ENclicks = 0;
        ENLastClickTime = 0;
    }

    if (currentMillis - prevRadioResendMillis >= radioResendInterval) {  // Check if we should send all radio commands
        sendAllRadioCommands();                                          // Don't need to send second HB signal because it was already send in sendAllRadioCommands
        prevHBMillis = currentMillis;
        prevRadioResendMillis = currentMillis;
    } else if (currentMillis - prevHBMillis >= HBInterval) {  // Every HBInterval ms send a new heartbeat to board (if it isn't already sent
        radioTransmitMode();
        resetDataTx();
        dataTx[0] = HEARTBEAT;
        radio.write(&dataTx, sizeof(dataTx));
        prevHBMillis = currentMillis;
    }

    // Check if controller is still connected (hearbeat signal present within time?)
    connected = (currentMillis - lastHBTime >= HBTimeoutMax) ? false : true;
    if (oldConnected != connected) {  // Only update on change
        updateDisplayFlag = true;
    }
    oldConnected = connected;

    // Perform display update if enough time has elapsed
    if (currentMillis - prevDispUpdateMillis >= dispMinUpdate && (updateDisplayFlag || currentMillis - lastDispStateOneTime > displayStateOneChangeTime) && MASTER_STATE != 0) {
        updateDisplayFlag = false;
        prevDispUpdateMillis = currentMillis;

        if (displaySensorData) {  // Update with proper display mode
            updateDisplay(DISPU_SENSDATA);
        } else {
            updateDisplay(DISPU_FULL);
        }
    }

    // End tone if it's expired
    if (currentMillis > toneExpire && toneActive) {
        noTone(BUZZER_PIN);
        toneActive = false;
    }

    radioRecieveMode();  // Default to recieve mode so as to not drop transmissions
}

void radioInterupt() {
    resetDataRx();
    radio.read(&dataRx, sizeof(dataRx));

    if (MASTER_STATE = 0) {
        if (radio.available()) {
            if (dataRx[0] == 200) {  // 200 is "heartbeat" signal
                DEBUG_PRINT(F("Got first heartbeat signal from board"));
                connected = true;  // Set connected flag
                transitionState(1);
            }
        }
    }

    boolean vvNew = false;  // Keep track of new data from vesc; did it actually arrive
    boolean ssNew = false;  // Keep track of new data from sensor; did it actually arrive

    switch ((int)dataRx[0]) {
        case HEARTBEAT:  // Board -> remote heartbeats
            lastHBTime = currentMillis;
            break;
        case SENDALLDATA:
            sendAllRadioCommands();
            break;
        case SCREENUPDATE:
            updateDisplayFlag = true;  // Set flag so as not to refresh too fast
            break;
        case VESCDATA:  // ID 0 is speed, ID 1 is distance travelled, ID 2 is input voltage, ID 3 is batt percent
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
        case SENSDATA:  // ID 13: Sensor data [id, value]. ID 0 is pitch, ID 1 is roll, ID 2 is heading, ID 3 is acceleration, ID 4 is temperature, ID 5 is altitude
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
    if (ssNew && displaySENSData) {
        DEBUG_PRINT(F("New sensor data recieved\n(Pitch, roll, heading): "));
        DEBUG_PRINT(String(sensor_values_realtime.pitch) + "," + String(sensor_values_realtime.roll) + "," + String(sensor_values_realtime.heading));
        DEBUG_PRINT(F("Acceleration: "));
        DEBUG_PRINT(String(sensor_values_realtime.acceleration));
        DEBUG_PRINT(F("Temperature: "));
        DEBUG_PRINT(String(sensor_values_realtime.temperature));
        DEBUG_PRINT(F("Altitude: "));
        DEBUG_PRINT(String(sensor_values_realtime.altitude));
    }
    break;
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
    dataTx[0] = LEDMODE;  // Led update
    dataTx[1] = ledMode;
    radio.write(&dataTx, sizeof(dataTx));
    resetDataTx();

    dataTx[0] = BOOSTMODE;  // Boost update
    dataTx[1] = (boostEnabled) ? 1 : 0;
    radio.write(&dataTx, sizeof(dataTx));

    resetDataTx();
    dataTx[0] = THROTTLE_SW;  // Sw update
    dataTx[1] = (throttleEnabled) ? 1 : 0;
    radio.write(&dataTx, sizeof(dataTx));

    resetDataTx();
    dataTx[0] = THROTTLE_VAL;  // Throttle update
    dataTx[1] = prevThrottle;
    radio.write(&dataTx, sizeof(dataTx));

    resetDataTx();
    dataTx[0] = CLICK;  // Click event
    dataTx[1] = ENclicks;
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
                u8g2.drawStr(x + 50, y + 12, "Sensor Data");
                u8g2.setFont(u8g2_font_profont12_tr);

                y += 12;  // Pitch
                displayString = "P:";
                displayString += String(sensor_values_realtime.pitch);
                displayString.toCharArray(displayBuffer, 8);
                u8g2.drawStr(x, y, displayBuffer);

                y += 12;  // Roll
                displayString = "R:";
                displayString += String(sensor_values_realtime.roll);
                displayString.toCharArray(displayBuffer, 8);
                u8g2.drawStr(x, y, displayBuffer);

                y += 12;  // Heading
                displayString = "H:";
                displayString += String(sensor_values_realtime.heading);
                displayString.toCharArray(displayBuffer, 8);
                u8g2.drawStr(x, y, displayBuffer);

                y += 12;  // Temperature
                displayString = "T:";
                displayString += String(sensor_values_realtime.temperature);
                displayString.toCharArray(displayBuffer, 8);
                u8g2.drawStr(x, y, displayBuffer);

                y += 12;  // Altitude
                displayString = "A:";
                displayString += String(sensor_values_realtime.altitude);
                displayString.toCharArray(displayBuffer, 8);
                u8g2.drawStr(x, y, displayBuffer);
                break;
            case DISPU_FULL:
                x = 0;
                y = 0;
                // BATTERY LEVEL
                // Position on OLED
                x = 108;
                y = 4;

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
                y = 17;

                if (connected == true) {
                    if (throttleEnabled) {
                        u8g2.drawXBM(x, y, 12, 12, signal_transmitting_bits);
                    } else {
                        u8g2.drawXBM(x, y, 12, 12, signal_connected_bits);
                    }
                } else {
                    u8g2.drawXBM(x, y, 12, 12, signal_noconnection_bits);
                }

                // ENABLED INDICATOR

                x = 110;
                y = 40;

                u8g2.setFont(u8g2_font_profont12_tr);
                if (throttleEnabled) {
                    u8g2.drawStr(x, y, "T:E");
                } else {
                    u8g2.drawStr(x, y, "T:D");
                }

                // BOOST INDICATOR

                x = 110;
                y = 50;

                u8g2.setFont(u8g2_font_profont12_tr);
                if (boostEnabled) {
                    u8g2.drawStr(x, y, "B:E");
                } else {
                    u8g2.drawStr(x, y, "B:D");
                }

                // LED MODE INDICATOR

                x = 110;
                y = 60;
                u8g2.setFont(u8g2_font_profont12_tr);
                displayString = "L:";
                displayString += String(ledMode);
                displayString.toCharArray(displayBuffer, 4);
                u8g2.drawStr(x, y, displayBuffer);

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

                // VESC DATA INDICATOR

                x = 0;
                y = 26;
                String prefix;
                String suffix;
                float value;
                int decimals;
                int first, last;

                if (millis() - lastDispStateOneTime > displayStateOneChangeTime) {
                    displayStateOne = !displayStateOne;
                    lastDispStateOneTime = millis();
                }

                for (int i = 0; i < 2; i++) {
                    switch (i) {
                        case 0:  // >--- Speed
                            prefix = F("SPEED");
                            suffix = F("MPH");
                            value = vesc_values_realtime.speed;
                            decimals = 1;
                            break;
                        case 1:  // >--- Distance
                            if (displayStateOne) {
                                prefix = F("DISTANCE");
                                suffix = F("MI");
                                value = vesc_values_realtime.distanceTravelled;
                                decimals = 2;
                            } else {  // >--- Batt Voltage
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

                    y += 25;
                }
                break;
            case CLEAR:
            default:
                break;
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