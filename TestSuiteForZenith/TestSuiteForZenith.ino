#include <SPI.h>
#include "printf.h"
#include <nRF24L01.h>
#include <RF24.h>

#include "ADS1X15.h"

#include <SPI.h>
#include <SD.h>

#include <math.h>
#include <FRAM_MB85RC_I2C.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Adafruit_BMP280.h>


ADS1015 ADS(0x48);
RF24 radio(0,1); // CE, CSN
const byte addresses [][6] = {"00001", "00002"}; //write at addr 00002, read at addr 00001

Sd2Card card;
SdVolume volume;
SdFile root;

FRAM_MB85RC_I2C fram;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

const boolean debug = true;

#define INDICATOR_PIN 13
#define FETONE_PIN 23
#define FETTWO_PIN 22
#define BUZZER_PIN 20
#define SD_CHIPSELECT 21

const double ADC_RES_DIV_FACTOR_VBUS = 161.89;
const double ADC_RES_DIV_FACTOR_AUX = 18.26923;

void setup()
{
  Serial.begin(115200);
  delay(100);


  /*
  Radio
  */
  printf_begin();
  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]); //set address to recieve data   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MAX);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();              //This sets the module as receiver
  debugPrintln("Radio details:");
  radio.printDetails();
  Serial.println("\n\n");

  /*
  SD Card
  */
  debugPrintln("Testing SD card");
  if (!card.init(SPI_HALF_SPEED, SD_CHIPSELECT)) {
    Serial.println("SD init failed");
    //while (1);
  } else {
    Serial.println("SD init OK");
  }
  Serial.println("\n\n");

  /*
  FRAM
  */
  debugPrintln("Testing FRAM");
  Wire.begin();
  delay(50);
  fram.begin();
  Serial.println("\n\n");

  /*
  BNO055
  */
  debugPrintln("Testing BNO055");
  if (!bno.begin())
  {
    Serial.println("BNO055 init failed");
  } else {
    Serial.println("BNO055 init OK");

    delay(100);
    sensors_event_t orientationData , angVelocityData , linearAccelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    if (debug) {
      printEvent(&orientationData);
      printEvent(&angVelocityData);
      printEvent(&linearAccelData);

      int8_t boardTemp = bno.getTemp();
      Serial.print("temperature: ");
      Serial.println(boardTemp);
    }
  }
  Serial.println("\n\n");

  /*
  BMP280
  */
  debugPrintln("Testing BMP280");
  if (!bmp.begin()) {
    Serial.println("BMP280 init failed");
  } else {
    Serial.println("BMP280 init OK");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    sensors_event_t temp_event, pressure_event;
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);
    
    if (debug) {
      Serial.print("Temperature = ");
      Serial.print(temp_event.temperature);
      Serial.println(" *C");

      Serial.print("Pressure = ");
      Serial.print(pressure_event.pressure);
      Serial.println(" hPa");
    }
  }
  Serial.println("\n\n");

  /*
  ADC
  */
  debugPrintln("Testing ADC; if it hangs here it is not working");
  ADS.begin();
  ADS.setGain(0);
  delay(50);
  int16_t ads0 = ADS.readADC(0);  
  int16_t ads1 = ADS.readADC(1);  
  int16_t ads2 = ADS.readADC(2);  
  int16_t ads3 = ADS.readADC(3);
  float f = ADS.toVoltage(1);

  double vBus = (double)ads0*ADC_RES_DIV_FACTOR_VBUS*(double)f; //use conv factor
  double railFive = (double)ads3*ADC_RES_DIV_FACTOR_AUX*(double)f;

  if (debug) {
    Serial.print("measured vBus: ");
    Serial.print(vBus);

    /*for (int i=0; i<50; i++) {
      Serial.println(ADS.readADC(0)*ADC_RES_DIV_FACTOR_VBUS*f);
      delay(200);
    }*/
    Serial.println("v");
    Serial.print("measured 5v rail: ");
    Serial.print(railFive);
    Serial.println("v");
  }
  if (railFive < 4.5 || railFive > 5.5) {
    Serial.println("5v rail test failed");
    //while(1) {}
  } else {
    Serial.println("5v rail test OK");
  }
  Serial.println("\n\n");

  /*
  Mosfets
  */
  pinMode(FETONE_PIN, OUTPUT);
  pinMode(FETTWO_PIN, OUTPUT);
  digitalWrite(FETONE_PIN, LOW); //Set both mosfet channels to be off
  digitalWrite(FETTWO_PIN, LOW);
  debugPrintln("Checking FETs");
  //Now we write to each fet at a time and check if the corresponding adc pin is high
  Serial.println("FET channel 1 test");
  digitalWrite(FETONE_PIN, HIGH);
  delay(2000);

  Serial.println("FET channel 2 test");
  digitalWrite(FETONE_PIN, LOW);
  digitalWrite(FETTWO_PIN, HIGH);
  delay(2000);

  digitalWrite(FETTWO_PIN, LOW);
  delay(50);
  Serial.println("\n\n");

    /*
  Indicator LED
  */
  Serial.println("Indicator LED test");
  pinMode(13, OUTPUT);
  for(int i=0; i<10; i++) {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }
  Serial.println("\n\n");

  /*
  Buzzer
  */
  delay(50);
  Serial.println("Buzzer test");
  int c = 0;
  pinMode(BUZZER_PIN, OUTPUT);
  for (int c = 0; c<2000; c++) {
    tone(BUZZER_PIN, c);
    delay(1);
  }
  noTone(BUZZER_PIN);
  Serial.println("\n\n");

  Serial.println("Test concluded");
}

void loop() 
{
}

void printEvent(sensors_event_t* event) {
  Serial.println();
  Serial.print(event->type);
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }

  Serial.print(": x= ");
  Serial.print(x);
  Serial.print(" | y= ");
  Serial.print(y);
  Serial.print(" | z= ");
  Serial.println(z);
}

void debugPrint(String p) {
  if (debug) {
    Serial.print(p);
  }
}

void debugPrintln(String p) {
  if (debug) {
    Serial.println(p);
  }
}
