//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "TFT_eSPI.h" /* Please use the TFT library provided in the library. */

#define SEALEVELPRESSURE_HPA (1027)                  // default 1013.25

#define BUTTON_PIN                    35

float temperature, minTemperature, maxTemperature, lastTemperature;
float pressure;
float altitude;
float humidity, maxHumidity, minHumidity, lastHumidity;

uint32_t cpu_frequency;

char buf_temperature[20] = {' '};
char buf_minTemperature[20] = {' '};
char buf_maxTemperature[20] = {' '};
char buf_pressure[20] = {' '};
char buf_altitude[20] = {' '};
char buf_humidity[20] = {' '};
char buf_minHumidity[20] = {' '};
char buf_maxHumidity[20] = {' '};
char buf_reminingTime[20] = {' '};

Adafruit_BME280 bme; // I2C

int step = 1;
int reminingTime = 0;
int waitShort = 2500;
int waitShortRandom = 1250;
int waitLong = 3500;
int waitLongRandom = 1500;
int waitRemining = 500;
int waitReminingRandom = 250;
int debounceTime = 300;
int displayOffTime = 120000;
int calcTime = 300000;
int serialTime = 1000;
int readSensorTime = 500;
int clearTime = 86400000;

unsigned long lastReadSensor = 0;
unsigned long lastDisplayPrint = 0;
unsigned long lastDisplayPart = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastButtonChanged = 0;
unsigned long lastTurnOff = 0;
unsigned long lastClearMinMax = 0;
unsigned long lastCalc = 0;

bool initSensor = HIGH;
bool initDisplay = HIGH;
bool initSerial = HIGH;
bool buttonState = LOW;
bool lastButtonState = LOW;
bool buttonAction = LOW; 
bool turnOff = LOW;

//////////////////////////////////////////////////////////////////////

TFT_eSPI tft = TFT_eSPI();

//////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);
  Serial.println("Weatherstation");

  setCpuFrequencyMhz(80);               // Set CPU Frequenz 240, 160, 80, 40, 20, 10 Mhz
  
  cpu_frequency = getCpuFrequencyMhz();
  Serial.println(cpu_frequency);

  pinMode(BUTTON_PIN, INPUT);

  //tft.init();
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  //tft.setSwapBytes(true);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);


  lastTurnOff = millis();


  bool status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Read Values for initialization Min Max
  temperature = bme.readTemperature() + 1;
  minTemperature = temperature;
  maxTemperature = temperature;

  // Read Values for initialization Min Max
  humidity = bme.readHumidity();
  minHumidity = humidity;
  maxHumidity = humidity;

  // First Calculate the remining time for clear Min Max 
  reminingTime = round(((((millis() - lastClearMinMax) - clearTime) * (-1)) / 3600000));
}

//////////////////////////////////////////////////////////////////////

void loop() {

  /////////// Read Sensor Function ///////////
  if ((millis() - lastReadSensor > readSensorTime) || (initSensor == HIGH)) {
    temperature = bme.readTemperature() + 1;
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    humidity = bme.readHumidity();

    // Clear Min Max after 24h
    if (millis() - lastClearMinMax > clearTime) {
      minTemperature = temperature;
      maxTemperature = temperature;
      minHumidity = humidity;
      maxHumidity = humidity;

      lastClearMinMax = millis();
    }

    // Calculate Min Max
    if ((temperature < lastTemperature) && (temperature < minTemperature)){
      minTemperature = temperature;
    }
    if ((temperature > lastTemperature) && (temperature > maxTemperature)) {
      maxTemperature = temperature;
    }

    if ((humidity < lastHumidity) && (humidity < minHumidity)){
      minHumidity = humidity;
    }
    if ((humidity > lastHumidity) && (humidity > maxHumidity)) {
      maxHumidity = humidity;
    }

    lastTemperature = temperature;
    lastHumidity = humidity;



    initSensor = LOW;
    lastReadSensor = millis();
  }

  /////////// Serial Print Function ///////////

  
  if ((millis() - lastSerialPrint > serialTime) || (initSerial == HIGH)) {

    Serial.print("Temperature = "); Serial.print(temperature); Serial.println(" *C");
    Serial.print("TemperatureMin = "); Serial.print(minTemperature); Serial.println(" *C");
    Serial.print("TemperatureMax = "); Serial.print(maxTemperature); Serial.println(" *C");
    Serial.print("Pressure = "); Serial.print(pressure); Serial.println(" hPa");
    Serial.print("Approx. Altitude = "); Serial.print(altitude); Serial.println(" m");
    Serial.print("Humidity = "); Serial.print(humidity); Serial.println(" %");
    Serial.print("HumidityMin = "); Serial.print(minHumidity); Serial.println(" %");
    Serial.print("HumidityMax = "); Serial.print(maxHumidity); Serial.println(" %");
    Serial.println("");

    initSerial = LOW;
    lastSerialPrint = millis();
  }
  
  
  
  /////////// Print Display Function ///////////
  if ((millis() - lastDisplayPrint > 1) || (initDisplay == HIGH)) {

    // Temperature
    if ((millis() - lastDisplayPart > waitLong + random(waitLongRandom)) && (step == 1)) {
      int temperature_int = (int) temperature;
      float temperature_float = (abs(temperature) - abs(temperature_int)) * 100;
      int temperature_fra = (int)temperature_float;
      sprintf (buf_temperature, "%d.%d", temperature_int, temperature_fra);
      tft.fillScreen(TFT_BLACK);
      tft.drawString(buf_temperature, 20, 20, 6); tft.drawString(".", 160, 8, 6); tft.drawString("C", 174, 38, 4); 
      step = 2;
      lastDisplayPart = millis();
    }

    // Temperature Min
    if ((millis() - lastDisplayPart > waitShort + random(waitShortRandom)) && (step == 2)) {
      int minTemperature_int = (int) minTemperature;
      float minTemperature_float = (abs(minTemperature) - abs(minTemperature_int)) * 100;
      int minTemperature_fra = (int)minTemperature_float;
      sprintf (buf_minTemperature, "%d.%d", minTemperature_int, minTemperature_fra);
      tft.fillScreen(TFT_BLACK);
      tft.drawString(buf_minTemperature, 20, 20, 6); tft.drawString(".", 160, 8, 6); tft.drawString("C", 174, 38, 4); 
      tft.drawString("min", 120, 80, 4);
      step = 3;
      lastDisplayPart = millis();
    }

    // Temperature Max
    if ((millis() - lastDisplayPart > waitShort + random(waitShortRandom)) && (step == 3)) {
      int maxTemperature_int = (int) maxTemperature;
      float maxTemperature_float = (abs(maxTemperature) - abs(maxTemperature_int)) * 100;
      int maxTemperature_fra = (int)maxTemperature_float;
      sprintf (buf_maxTemperature, "%d.%d", maxTemperature_int, maxTemperature_fra);
      tft.fillScreen(TFT_BLACK);
      tft.drawString(buf_maxTemperature, 20, 20, 6); tft.drawString(".", 160, 8, 6); tft.drawString("C", 174, 38, 4); 
      tft.drawString("max", 120, 80, 4);
      step = 4;
      lastDisplayPart = millis();
    }

    // Pressure
    if ((millis() - lastDisplayPart > waitLong + random(waitLongRandom)) && (step == 4)) {
      int pressure_int = (int) pressure;
      float pressure_float = (abs(pressure) - abs(pressure_int)) * 100;
      int pressure_fra = (int)pressure_float;
      sprintf (buf_pressure, "%d.%d", pressure_int, pressure_fra);
      tft.fillScreen(TFT_BLACK);
      tft.drawString(buf_pressure, 20, 20, 6); tft.drawString("hPa", 174, 38, 4); 
      step = 5;
      lastDisplayPart = millis();
    }
    
    // Humidity
    if ((millis() - lastDisplayPart > waitLong + random(waitLongRandom)) && (step == 5)) {
      int humidity_int = (int) humidity;
      float humidity_float = (abs(humidity) - abs(humidity_int)) * 100;
      int humidity_fra = (int)humidity_float;
      sprintf (buf_humidity, "%d.%d", humidity_int, humidity_fra);
      tft.fillScreen(TFT_BLACK);
      tft.drawString(buf_humidity, 20, 20, 6); tft.drawString("%", 174, 38, 4); 
      step = 6;
      lastDisplayPart = millis();
    }

    // Humidity Min
    if ((millis() - lastDisplayPart > waitShort + random(waitShortRandom)) && (step == 6)) {
      int minHumidity_int = (int) minHumidity;
      float minHumidity_float = (abs(minHumidity) - abs(minHumidity_int)) * 100;
      int minHumidity_fra = (int)minHumidity_float;
      sprintf (buf_minHumidity, "%d.%d", minHumidity_int, minHumidity_fra);
      tft.fillScreen(TFT_BLACK);
      tft.drawString(buf_minHumidity, 20, 20, 6); tft.drawString("%", 174, 38, 4); 
      tft.drawString("min", 120, 80, 4);
      step = 7;
      lastDisplayPart = millis();
    }

    // Humidity Max
    if ((millis() - lastDisplayPart > waitShort + random(waitShortRandom)) && (step == 7)) {
      int maxHumidity_int = (int) maxHumidity;
      float maxHumidity_float = (abs(maxHumidity) - abs(maxHumidity_int)) * 100;
      int maxHumidity_fra = (int)maxHumidity_float;
      sprintf (buf_maxHumidity, "%d.%d", maxHumidity_int, maxHumidity_fra);
      tft.fillScreen(TFT_BLACK);
      tft.drawString(buf_maxHumidity, 20, 20, 6); tft.drawString("%", 174, 38, 4); 
      tft.drawString("max", 120, 80, 4);
      step = 8;
      lastDisplayPart = millis();
    }

    // Remining Time until Min Max is cleaned
    if ((millis() - lastDisplayPart > waitRemining + random(waitReminingRandom)) && (step == 8)) {
      sprintf (buf_reminingTime, "%d", reminingTime);
      tft.fillScreen(TFT_BLACK);
      tft.drawString(buf_reminingTime, 20, 20, 6); tft.drawString("h", 174, 38, 4); 
      step = 9;
      lastDisplayPart = millis();
    }

    // Go back to first Step
    if ((millis() - lastDisplayPart > 1) && (step == 9)) {
      lastDisplayPart = millis();
      lastDisplayPrint = millis();
      step = 1;
    }

    initDisplay = LOW;
  }

  // Read Button and do anything
  if (millis() - lastButtonChanged > debounceTime) {
  buttonState = digitalRead(BUTTON_PIN); 

    if (buttonState != lastButtonState) {
      lastButtonChanged = millis();
      lastButtonState = buttonState;

      if (buttonState == LOW) {
        //Serial.println("Button released");
        buttonAction = !buttonAction;

        if (buttonAction == HIGH) {
          Serial.println("HIGH");
          tft.writecommand(0x11);
          digitalWrite(TFT_BL, HIGH); // turns on backlight
          turnOff = LOW;
          lastTurnOff = millis();
        }
        if (buttonAction == LOW) {
          Serial.println("LOW");
          tft.writecommand(0x10); // Sleep
          digitalWrite(TFT_BL, LOW); // turns off backlight
          turnOff = HIGH;
        }
      }
    }
  }

  // Function turn off the Display after some time
  if ((millis() - lastTurnOff > displayOffTime) && (turnOff == LOW)) {
    tft.writecommand(0x10); // Sleep
    digitalWrite(TFT_BL, LOW); // turns off backlight
    turnOff = HIGH;
    buttonAction = !buttonAction;
  }

  // Calculate the remining time for clear Min Max
  if (millis() - lastCalc > calcTime) {
    reminingTime = round(((((millis() - lastClearMinMax) - clearTime) * (-1)) / 3600000));
    lastCalc = millis();
  }
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////