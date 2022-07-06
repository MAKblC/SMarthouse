// Здесь нужно вписать настройки вашего проекта:
// ID шаблона, имя устройства и токен (желательно на английском и без пробелов)
#define BLYNK_TEMPLATE_ID "XXXXXXXX"
#define BLYNK_DEVICE_NAME "XXXXXXXXX"
#define BLYNK_AUTH_TOKEN "XXXXXXXXXXXXXXXXXXXXXXX"

// Параметры вашего Wi-Fi соединения
char ssid[] = "XXXXXXXXXXXX";
char pass[] = "XXXXXXXXXXXXXX";
char auth[] = BLYNK_AUTH_TOKEN;

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>

#include "TLC59108.h"
#define HW_RESET_PIN 0 // Только програмнный сброс
#define I2C_ADDR TLC59108::I2C_ADDR::BASE
TLC59108 leds(I2C_ADDR + 7); // Без перемычек добавляется 3 бита адреса
TLC59108 leds2(I2C_ADDR + 0); // Без перемычек добавляется 3 бита адреса   // RGB модуль
TLC59108 leds3(I2C_ADDR + 6); // Без перемычек добавляется 3 бита адреса   // RGB модуль

#include <ESP32_Servo.h>                      // конфигурация сервомотора // servo configuration

#include "SparkFun_SGP30_Arduino_Library.h"  // датчик газа
SGP30 mySensor;

#include <VL53L0X.h>    /// датчик расстояния
VL53L0X lox;
#define HIGH_ACCURACY

#define sensor_addr 0x39         // датчик пламени
float ir_data = 0;
float vis_data = 0;

#include <Adafruit_MCP4725.h>                           // динамик
Adafruit_MCP4725 buzzer;
int ton;
int vol1 = 1000; // Уровень громкости = vol1-vol2
int vol2 = 900;  //

#include <Adafruit_LSM9DS1.h>                       // гироскоп
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#include "mcp3021.h"
uint8_t adcDeviceId =  0b00000001; // Адрес микросхемы A0
MCP3021 mcp3021;
const float air_value = 570.0;
const float water_value = 335.0;
const float moisture_0 = 0.0;
const float moisture_100 = 100.0;

#include "MCP3221.h"            // микрофон
const byte DEV_ADDR = 0x4D;
MCP3221 mcp3221(DEV_ADDR);

#define  wind   17                     // пин вентилятора // cooler pin 
#define  amper  14                     // пин амперметра

Servo myservo;
int pos = 1;            // начальная позиция сервомотора // servo start position
int prevangle = 1;      // предыдущий угол сервомотора // previous angle of servo

#include <BH1750FVI.h>        // добавляем библиотеку датчика освещенности // adding Light intensity sensor library  
BH1750FVI LightSensor_1;      // BH1750

#include <Adafruit_Sensor.h>  // добавляем библиотеку датчика температуры, влажности и давления // adding Temp Hum Bar sensor library
#include <Adafruit_BME280.h>  // BME280                         
Adafruit_BME280 bme280;       //

#define UPDATE_TIMER 1000
BlynkTimer timer_update;      // настройка таймера для обновления данных с сервера BLynk // Blynk update timer configuration

void setup()
{
  myservo.attach(13);             // пин сервомотора // servo pin

  init_sensor();

  pinMode( wind, OUTPUT );       // настройка пина вентилятора на выход // pump and cooler pins configured on output mode
  digitalWrite(wind, LOW);

  Serial.begin(115200);

  delay(512);
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);

  Wire.begin();

  leds.init(HW_RESET_PIN);
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  leds2.init(HW_RESET_PIN);
  leds2.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  leds3.init(HW_RESET_PIN);
  leds3.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);

  buzzer.begin(0x61); // С перемычкой адрес будет 0x60
  buzzer.setVoltage(0, false);   // выключение звука
  delay(1000);

  if (mySensor.begin() == false)
    Serial.println("No SGP30 Detected. Check connections.");
  mySensor.initAirQuality();

  if (!lsm.begin())
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

  LightSensor_1.begin();              // запуск датчика освещенности // turn the light intensity sensor on
  LightSensor_1.setMode(Continuously_High_Resolution_Mode);

  lox.init();
  lox.setTimeout(500);
#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  lox.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  lox.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  lox.setMeasurementTimingBudget(200000);
#endif

  bool bme_status = bme280.begin();
  if (!bme_status)
    Serial.println("Could not find a valid BME280 sensor, check wiring!");  // проверка  датчика температуры, влажности и давления // checking the temp hum bar sensor

  ledcSetup(5, 50, 10);
  ledcAttachPin(amper, 5);

  timer_update.setInterval(UPDATE_TIMER, readSendData);  // включаем таймер обновления данных  // turn on the update timer

  mcp3021.begin(adcDeviceId);
}

void readSendData() { // чтение данных и отправка на сервер

  float adc0 = mcp3021.readADC();
  float hum = map(adc0, air_value, water_value, moisture_0, moisture_100);
  Blynk.virtualWrite(V6, hum); delay(2);        // Отправка данных на сервер

  float t = bme280.readTemperature();
  float h = bme280.readHumidity();
  float p = bme280.readPressure() / 100.0F;
  Blynk.virtualWrite(V0, t); delay(2);        // Отправка данных на сервер Blynk  Температура // Temperature data send
  Blynk.virtualWrite(V1, h); delay(2);        // Отправка данных на сервер Blynk  Влажность   // Humidity data send
  Blynk.virtualWrite(V2, p); delay(2);        // Отправка данных на сервер Blynk  Давление    // Pressure data send

  mySensor.measureAirQuality();
  Blynk.virtualWrite(V7, mySensor.TVOC); delay(2);        // Отправка данных на сервер
  Blynk.virtualWrite(V4, mySensor.CO2); delay(2);        // Отправка данных на сервер

  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  Blynk.virtualWrite(V5, a.acceleration.x); delay(2);        // Отправка данных на сервер
  Blynk.virtualWrite(V8, a.acceleration.y); delay(2);        // Отправка данных на сервер
  Blynk.virtualWrite(V9, a.acceleration.z); delay(2);        // Отправка данных на сервер

  float dist = lox.readRangeSingleMillimeters();
  Blynk.virtualWrite(V10, dist); delay(2);        // Отправка данных на сервер

  poll_sensor();
  Blynk.virtualWrite(V11, ir_data); delay(2);        // Отправка данных на сервер

  float l = LightSensor_1.getAmbientLight();
  Blynk.virtualWrite(V3, l); delay(2);        // Отправка данных на сервер

  float snd = mcp3221.getVoltage();
  Blynk.virtualWrite(V12, snd); delay(2);        // Отправка данных на сервер
}

void loop()
{
  Blynk.run();                                          // запуск Blynk  // turn Blynk on
  timer_update.run();
}

BLYNK_WRITE(V16) // дверь
{
  int angle = param.asInt();
  if (prevangle < angle) {
    for (pos = prevangle; pos <= angle; pos += 1)
    {
      myservo.write(pos);
      delay(5);                                        // если угол задан больше предыдущего, то доводим до нужного угла в ++ // if the current angle>previous angle then going clockwise
    }
    prevangle = angle;
  }
  else if (prevangle > angle) {
    for (pos = prevangle; pos >= angle; pos -= 1)
    {
      myservo.write(pos);
      delay(5);                                       // если угол задан меньше предыдущего, то доводим до нужного угла в -- // if the current angle<previous angle then going counter-clockwise
    }
    prevangle = angle;
  }
}

BLYNK_WRITE(V13) //вентилятор
{
  int buttonstate2 = param.asInt ();
  if (buttonstate2 == 1) {
    digitalWrite(wind, HIGH);         // включить, если нажата кнопка "Вентилятор" // turn on the cooler if button = 1
  }
  else    {
    digitalWrite(wind, LOW);
  }
}

BLYNK_WRITE(V14) // свет
{
  int buttonstate2 = param.asInt ();
  if (buttonstate2 == 1) {
    leds.setBrightness(0, 0xff);
    leds.setBrightness(6, 0xff);
    leds2.setBrightness(0, 0xff);
    leds2.setBrightness(6, 0xff);
    leds3.setBrightness(0, 0xff);
    leds3.setBrightness(6, 0xff);
  }
  else    {
    leds.setBrightness(0, 0x00);
    leds.setBrightness(6, 0x00);
    leds2.setBrightness(0, 0x00);
    leds2.setBrightness(6, 0x00);
    leds3.setBrightness(0, 0x00);
    leds3.setBrightness(6, 0x00);
  }
}

BLYNK_WRITE(V17) // звук
{
  int buttonstate2 = param.asInt ();
  if (buttonstate2 == 1) {
    note(14, 400); note(2, 100); note(9, 400); note(7, 500); note(14, 300); note(9, 700);
    buzzer.setVoltage(0, false);
  }
}

BLYNK_WRITE(V15) // амперметр
{
  int pwr = param.asInt();
  ledcWrite(5, pwr);
  delay(10);
}

int note( int type, int duration) {   // нота (какая нота, длительность)
  switch (type) {
    case 1:   ton = 1000; break;
    case 2:   ton = 860;  break;
    case 3:   ton = 800;  break;
    case 4:   ton = 700;  break;
    case 5:   ton = 600;  break;
    case 6:   ton = 525;  break;
    case 7:   ton = 450;  break;
    case 8:   ton = 380;  break;
    case 9:   ton = 315;  break;
    case 10:  ton = 250;  break;
    case 11:  ton = 190;  break;
    case 12:  ton = 130;  break;
    case 13:  ton = 80;   break;
    case 14:  ton = 30;   break;
    case 15:  ton = 1;   break;
  }
  delay(10);
  for (int i = 0; i < duration; i++) {
    buzzer.setVoltage(vol1, false);
    buzzer.setVoltage(vol2, false);
    delayMicroseconds(ton);
  }
}

void init_sensor() { // датчик пламени
  Wire.begin();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x81);       // Регистр времени интегрирования АЦП
  Wire.write(0b00111111); // 180 мс, 65535 циклов
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x83);       // Регистр времени ожидания
  Wire.write(0b00111111); // 180 мс
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x90);         // Регистр усиления
  Wire.write(0b00000000);   // Усиление 1x
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x80);       // Регистр управления питанием
  Wire.write(0b00001011); // Включение ожидания, генератора, АЦП и ALS сенсора
  Wire.endTransmission();
}

void poll_sensor() { // датчик пламени
  unsigned int sensor_data[4];
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x94); // Начальный адрес регистров данных
  Wire.endTransmission();
  Wire.requestFrom(sensor_addr, 4);
  if (Wire.available() == 4) {
    sensor_data[0] = Wire.read();
    sensor_data[1] = Wire.read();
    sensor_data[2] = Wire.read();
    sensor_data[3] = Wire.read();
  }
  ir_data   = sensor_data[3] * 256.0 + sensor_data[2];
  vis_data = sensor_data[1] * 256.0 + sensor_data[0];
}