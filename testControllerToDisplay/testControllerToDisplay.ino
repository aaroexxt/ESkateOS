
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

char displayBuffer[20];
String displayString = "";

float battBuffer[50];
unsigned long lastBattTime = 0;
byte battBufferPos = 0;

// End of constructor list

int rVal = 254;
int gVal = 1;
int bVal = 127;

int rDir = -1;
int gDir = 1;
int bDir = -1;

// constants to name the pins
const int rPin = 11;
const int gPin = 12;
const int bPin = 13;

const int buzzPin = 9;


void setup(void) {
  u8g2.begin();  
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(buzzPin, OUTPUT);
  pinMode(rPin, OUTPUT);
  pinMode(gPin, OUTPUT);
  pinMode(bPin, OUTPUT);

  delay(50);
  float avgBatt = 0;
  for (byte i=0; i<10; i++) {
    avgBatt += analogRead(A0) * (3.3 / 1023.0) * 2.0;
    delay(20);
  }
  avgBatt/=10.0;
  for (byte i=0; i<50; i++) {
    battBuffer[i] = avgBatt;
  }
  
  Serial.begin(115200);
  tone(buzzPin, 2500);
  delay(200);
  tone(buzzPin, 3000);
  delay(200);
  tone(buzzPin, 3500);
  delay(200);
  noTone(buzzPin);
}

float mapFloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void loop(void) {
  float throttPercent = mapFloat(analogRead(A1), 775, 280, 0, 100); 
  if (millis() - lastBattTime > 10) {
    float battVoltage= analogRead(A0) * (3.3 / 1023.0) * 2.0;
    battBuffer[battBufferPos] = battVoltage;
    battBufferPos++;
    if (battBufferPos > 49) battBufferPos = 0;

    lastBattTime = millis();
  }

  float avgBatt = 0;
  for (byte i=0; i<50; i++) {
    avgBatt += battBuffer[i];
  }
  avgBatt /= 50.0;
  
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB10_tr);
    displayString = "V: ";
    displayString += String(avgBatt);
    displayString.toCharArray(displayBuffer, 8);
    u8g2.drawStr(0,15,displayBuffer);

    displayString = "%: ";
    displayString += String(throttPercent);
    displayString.toCharArray(displayBuffer, 8);
    u8g2.drawStr(0, 35, displayBuffer);

    displayString = "TSW: ";
    displayString += digitalRead(A2) == HIGH ? "Y" : "N";
    displayString.toCharArray(displayBuffer, 8);
    u8g2.drawStr(0, 55, displayBuffer);

    displayString = "B1: ";
    displayString += digitalRead(2) == HIGH ? "Y" : "N";
    displayString.toCharArray(displayBuffer, 8);
    u8g2.drawStr(70, 15, displayBuffer);

    displayString = "B2: ";
    displayString += digitalRead(3) == HIGH ? "Y" : "N";
    displayString.toCharArray(displayBuffer, 8);
    u8g2.drawStr(70, 35, displayBuffer);

    displayString = "B3: ";
    displayString += digitalRead(4) == HIGH ? "Y" : "N";
    displayString.toCharArray(displayBuffer, 8);
    u8g2.drawStr(70, 55, displayBuffer);
  } while ( u8g2.nextPage() );

  analogWrite(rPin, rVal);
  analogWrite(gPin, gVal);
  analogWrite(bPin, bVal);

  // change the values of the LEDs
  rVal = rVal + rDir;
  gVal = gVal + gDir;
  bVal = bVal + bDir;

  // for each color, change direction if
  // you reached 0 or 255
  if (rVal >= 128 || rVal <= 0) {
    rDir = rDir * -1;
  }

  if (gVal >= 128 || gVal <= 0) {
    gDir = gDir * -1;
  }

  if (bVal >= 128 || bVal <= 0) {
    bDir = bDir * -1;
  }
  //delay(1000);
}
