/*
Не забудьте поправить настройки и адреса устройств в зависмости от комплектации!  
*/

// Здесь нужно вписать настройки вашего проекта:
// ID шаблона, имя устройства и токен (желательно на английском и без пробелов)
#define BLYNK_TEMPLATE_ID "XXXXXXXX"
#define BLYNK_TEMPLATE_NAME "XXXXXXXXX"
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

// Выберите модуль светодиодов в вашей сборке (ненужные занесите в комментарии)
#define MGL_RGB1EN 1
//#define MGL_RGB3 1

/////////////////// модуль светодиодов ///////////////////
#ifdef MGL_RGB1EN
#include "TLC59108.h" // библиотека для модуля MGL-RGB1
#define HW_RESET_PIN 0 // Только програмнный сброс
#define I2C_ADDR TLC59108::I2C_ADDR::BASE
TLC59108 leds(I2C_ADDR + 7); // Без перемычек добавляется 3 бита адреса
TLC59108 leds2(I2C_ADDR + 0); // Все перемычки на модуле стоят
TLC59108 leds3(I2C_ADDR + 6); // Стоит только одна перемычка
#endif
#ifdef MGL_RGB3
#include <PCA9634.h>
PCA9634 ledsModul(0x08); // (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)
PCA9634 ledsModul2(0x10);
PCA9634 ledsModul3(0x70);
#endif

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

// Выберите гироскоп или датчик цвета в вашей сборке (ненужные занесите в комментарии)
//#define MGS_A9 1
#define MGS_CLM60 1
//#define MGS_A6 1

/////////////////// гироскоп и датчик цвета ///////////////////
#ifdef MGS_A9
#include <Adafruit_LSM9DS1.h>
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#endif
#ifdef MGS_A6
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;
#endif
#ifdef MGS_CLM60
#include "Adafruit_APDS9960.h"
Adafruit_APDS9960 apds9960;
#endif

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

#include <BH1750.h>       // добавляем библиотеку датчика освещенности // adding Light intensity sensor library  
BH1750 lightMeter;     // BH1750

// добавляем библиотеку датчика температуры, влажности и давления // adding Temp Hum Bar sensor library
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

#ifdef MGL_RGB1EN
  leds.init(HW_RESET_PIN);
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  leds2.init(HW_RESET_PIN);
  leds2.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  leds3.init(HW_RESET_PIN);
  leds3.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
#endif
#ifdef MGL_RGB3
  ledsModul.begin();
  ledsModul2.begin();
  ledsModul3.begin();
  for (int channel = 0; channel < ledsModul.channelCount(); channel++)
  {
    ledsModul.setLedDriverMode(channel, PCA9634_LEDOFF); // выключить все светодиоды в режиме 0/1
    ledsModul2.setLedDriverMode(channel, PCA9634_LEDOFF);
    ledsModul3.setLedDriverMode(channel, PCA9634_LEDOFF);
  }
  for (int channel = 0; channel < ledsModul.channelCount(); channel++)
  {
    ledsModul.setLedDriverMode(channel, PCA9634_LEDPWM); // установка режима ШИМ (0-255)
    ledsModul2.setLedDriverMode(channel, PCA9634_LEDPWM);
    ledsModul3.setLedDriverMode(channel, PCA9634_LEDPWM);
  }
#endif

  buzzer.begin(0x61); // С перемычкой адрес будет 0x60
  buzzer.setVoltage(0, false);   // выключение звука
  delay(1000);

  if (mySensor.begin() == false)
    Serial.println("No SGP30 Detected. Check connections.");
  mySensor.initAirQuality();

#ifdef MGS_A9
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
  }
  Serial.println("Found LSM9DS1 9DOF");
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
#endif
#ifdef MGS_A6
 if (!mpu.begin(0x69)) { // (0x68) (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
#endif
#ifdef MGS_CLM60
  if (!apds9960.begin()) {
    Serial.println("Failed to initialize device!");
  }
  // Инициализация режимов работы датчика
  apds9960.enableColor(true);
  apds9960.enableProximity(true);
#endif

  lightMeter.begin();              // запуск датчика освещенности // turn the light intensity sensor on
 
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
  if (!bme_status) {
    Serial.println("Не найден по адресу 0х77, пробую другой...");
    bme_status = bme280.begin(0x76);
    if (!bme_status)
      Serial.println("Датчик не найден, проверьте соединение");
  }

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

#ifdef MGS_A9
  lsm.read(); // данные гироскопа, акселерометра и магнетометра
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  Blynk.virtualWrite(V5, a.acceleration.x); delay(2);        // Отправка данных на сервер
  Blynk.virtualWrite(V8, a.acceleration.y); delay(2);        // Отправка данных на сервер
  Blynk.virtualWrite(V9, a.acceleration.z); delay(2);        // Отправка данных на сервер
#endif
#ifdef MGS_A6
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Blynk.virtualWrite(V5, a.acceleration.x); delay(2);        // Отправка данных на сервер
  Blynk.virtualWrite(V8, a.acceleration.y); delay(2);        // Отправка данных на сервер
  Blynk.virtualWrite(V9, a.acceleration.z); delay(2);        // Отправка данных на сервер

  Serial.print(" Xnorm = ");
  Serial.print(normGyro.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normGyro.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normGyro.ZAxis);
#endif
#ifdef MGS_CLM60
  uint16_t red_data   = 0;
  uint16_t green_data = 0;
  uint16_t blue_data  = 0;
  uint16_t clear_data = 0;
  uint16_t prox_data  = 0;
  // Определение цвета
  while (!apds9960.colorDataReady()) {
    delay(5);
  }
  apds9960.getColorData(&red_data, &green_data, &blue_data, &clear_data);
  // Определение близости препятствия
  prox_data = apds9960.readProximity();
  // Вывод измеренных значений в терминал
  Blynk.virtualWrite(V5, red_data); delay(2);        // Отправка данных на сервер
  Blynk.virtualWrite(V8, green_data); delay(2);        // Отправка данных на сервер
  Blynk.virtualWrite(V9, blue_data); delay(2);        // Отправка данных на сервер

  Serial.println("RED   = " + String(red_data));
  Serial.println("GREEN = " + String(green_data));
  Serial.println("BLUE  = " + String(blue_data));
  Serial.println("CLEAR = " + String(clear_data));
  Serial.println("PROX  = " + String(prox_data));
#endif

  float dist = lox.readRangeSingleMillimeters();
  Blynk.virtualWrite(V10, dist); delay(2);        // Отправка данных на сервер

  poll_sensor();
  Blynk.virtualWrite(V11, ir_data); delay(2);        // Отправка данных на сервер

  float l = lightMeter.readLightLevel();
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

#ifdef MGL_RGB1EN
    leds.setBrightness(0, 0xff);
    leds.setBrightness(6, 0xff);
    leds2.setBrightness(0, 0xff);
    leds2.setBrightness(6, 0xff);
    leds3.setBrightness(0, 0xff);
    leds3.setBrightness(6, 0xff);
#endif
#ifdef MGL_RGB3
    ledsModul.write1(1, 0xff);
    ledsModul2.write1(1, 0xff);
    ledsModul3.write1(1, 0xff);
    ledsModul.write1(6, 0xff);
    ledsModul2.write1(6, 0xff);
    ledsModul3.write1(6, 0xff);
#endif
  }
  else    {
#ifdef MGL_RGB1EN
    leds.setBrightness(0, 0x00);
    leds.setBrightness(6, 0x00);
    leds2.setBrightness(0, 0x00);
    leds2.setBrightness(6, 0x00);
    leds3.setBrightness(0, 0x00);
    leds3.setBrightness(6, 0x00);
#endif
#ifdef MGL_RGB3
    ledsModul.write1(1, 0x00);
    ledsModul2.write1(1, 0x00);
    ledsModul3.write1(1, 0x00);
    ledsModul.write1(6, 0x00);
    ledsModul2.write1(6, 0x00);
    ledsModul3.write1(6, 0x00);
#endif
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
