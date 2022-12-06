// Проверка всех устройств "Умный дом" Йотик М2

#include <Wire.h>

// Выберите модуль светодиодов в вашей сборке (ненужные занесите в комментарии)
#define MGL_RGB1EN 1
//#define MGL_RGB3 1

/////////////////// модуль светодиодов ///////////////////
#ifdef MGL_RGB1EN
#include "TLC59108.h" // библиотека для модуля MGL_RGB1
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
#include <MPU6050.h>
MPU6050 mpu;
#endif
#ifdef MGS_CLM60
#include "Adafruit_APDS9960.h"
Adafruit_APDS9960 apds9960;
#endif

#include <I2C_graphical_LCD_display.h>            // дисплей
I2C_graphical_LCD_display lcd;

#include "mcp3021.h"
uint8_t adcDeviceId =  0b00000001; // Адрес микросхемы A0 "0b00000000"
MCP3021 mcp3021;
const float air_value = 570.0; // калибровочные значения
const float water_value = 335.0;
const float moisture_0 = 0.0;
const float moisture_100 = 100.0;

#include "MCP3221.h"            // микрофон
const byte DEV_ADDR = 0x4D; // 0x5С // 0x48
MCP3221 mcp3221(DEV_ADDR);


#define  wind   17                     // пин вентилятора (16)
#define  amper  14                     // пин амперметра
#define  button 4                      // пин кнопки

Servo myservo;
int pos = 1;            // начальная позиция сервомотора // servo start position
int prevangle = 1;      // предыдущий угол сервомотора // previous angle of servo

const byte picture [] PROGMEM = {      // картинка
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0,
  0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
  0xF0, 0xF0, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
  0x80, 0xC0, 0xE0, 0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0x80,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
  0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFE,
  0xF8, 0xF0, 0xE0, 0xF0, 0xF8, 0xFC, 0x7F, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
  0x00, 0xF8, 0xFE, 0xFF, 0xFF, 0xFF, 0x1F, 0x07, 0x03, 0x03, 0x01, 0xE1, 0xE1, 0xE1, 0xE3, 0xE3,
  0xE3, 0xE3, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0xF8, 0xF8,
  0xF8, 0xF9, 0xFF, 0xFF, 0xFF, 0xDF, 0x8F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFC, 0xFF, 0xFF,
  0xFF, 0x0F, 0x03, 0xC3, 0xC1, 0xC0, 0xC0, 0xC4, 0xFC, 0xF8, 0xF9, 0xF1, 0x03, 0x07, 0x1F, 0xFF,
  0xFF, 0xFE, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
  0x03, 0x07, 0x0F, 0x07, 0x03, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
  0x00, 0x03, 0x0F, 0x3F, 0x7F, 0x7F, 0xFF, 0xFC, 0xF8, 0xF0, 0xF0, 0xF1, 0xF1, 0xF1, 0xFF, 0xFF,
  0xFF, 0x7F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF1, 0xF1, 0xF1,
  0xF1, 0xF1, 0xF1, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFC, 0xF0, 0xE0, 0xE1, 0xC3, 0xC7, 0xC7, 0xC7, 0xC7, 0xC3, 0xE1, 0xF0, 0xF8, 0xFE, 0xFF,
  0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#include <BH1750FVI.h>        // добавляем библиотеку датчика освещенности // adding Light intensity sensor library  
BH1750FVI LightSensor_1;      // BH1750

#include <Adafruit_Sensor.h>  // добавляем библиотеку датчика температуры, влажности и давления // adding Temp Hum Bar sensor library
#include <Adafruit_BME280.h>  // BME280                         
Adafruit_BME280 bme280;       //

void setup()
{
  myservo.attach(13);             // пин сервомотора // servo pin

  init_sensor();

  lcd.begin();
  lcd.gotoxy (0, 0);
  lcd.clear (0, 0, 128, 64, 0x00);  // очищаем поле дисплея


  pinMode( button, INPUT );
  pinMode( wind, OUTPUT );       // настройка пинов насоса и вентилятора на выход // pump and cooler pins configured on output mode
  digitalWrite(wind, LOW);

  Serial.begin(115200);
  delay(512);
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
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, 0x69))
  {
    Serial.println("MGS-A6 Не обнаружен! Проверьте адрес!"); // (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)
    delay(500);
  }
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);

  // Check settings
  Serial.println();

  Serial.print(" * Sleep Mode:        ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Clock Source:      ");
  switch (mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }

  Serial.print(" * Gyroscope:         ");
  switch (mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
  }

  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());

  Serial.println();
#endif
#ifdef MGS_CLM60
  if (!apds9960.begin()) {
    Serial.println("Failed to initialize device!");
  }
  // Инициализация режимов работы датчика
  apds9960.enableColor(true);
  apds9960.enableProximity(true);
#endif

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

  mcp3021.begin(adcDeviceId);
}

void loop()
{
  lcd.blit (picture, sizeof picture);
  Serial.println("display logo");
  delay(1500);

  float snd = mcp3221.getVoltage();
  Serial.println("sound level = " + String(snd, 1));
  delay(1500);

#ifdef MGS_A9
  lsm.read(); // данные гироскопа, акселерометра и магнетометра
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  Serial.println("accel x = " + String(a.acceleration.x, 1));
  Serial.println("accel y = " + String(a.acceleration.y, 1));
  Serial.println("accel z = " + String(a.acceleration.z, 1));
#endif
#ifdef MGS_A6
  Vector rawGyro = mpu.readRawGyro(); // Сырые значения
  Vector normGyro = mpu.readNormalizeGyro(); // Преобразованные значения

  Serial.print(" Xraw = ");
  Serial.print(rawGyro.XAxis);
  Serial.print(" Yraw = ");
  Serial.print(rawGyro.YAxis);
  Serial.print(" Zraw = ");
  Serial.println(rawGyro.ZAxis);

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
  Serial.println("RED   = " + String(red_data));
  Serial.println("GREEN = " + String(green_data));
  Serial.println("BLUE  = " + String(blue_data));
  Serial.println("CLEAR = " + String(clear_data));
  Serial.println("PROX  = " + String(prox_data));
#endif
  delay(1500);
#ifdef MGL_RGB1EN
  leds.setBrightness(3, 0xff);
  leds2.setBrightness(3, 0xff);
  leds3.setBrightness(3, 0xff);
  Serial.println("house is red");
  delay(1500);
  leds.setBrightness(3, 0x00);
  leds2.setBrightness(3, 0x00);
  leds3.setBrightness(3, 0x00);
  Serial.println("house is dark");
#endif
#ifdef MGL_RGB3
  ledsModul.write1(3, 0xff);
  ledsModul2.write1(3, 0xff);
  ledsModul3.write1(3, 0xff);
  Serial.println("house is red");
  delay(1500);
  ledsModul.write1(3, 0x00);
  ledsModul2.write1(3, 0x00);
  ledsModul3.write1(3, 0x00);
  Serial.println("house is dark");
#endif
  delay(1500);

  float dist = lox.readRangeSingleMillimeters();
  Serial.println("distance = " + String(dist, 1) + " mm");
  delay(1500);

  poll_sensor();
  Serial.println("Visible = " + String(vis_data, 1) + " μW/cm2");
  Serial.println("IR = " + String(ir_data, 1) + " μW/cm2");
  delay(1500);

  float l = LightSensor_1.getAmbientLight();
  Serial.println("light intensity = " + String(l, 1) + " lx");
  delay(1500);

  int  buttonState = digitalRead(button);
  if (buttonState == HIGH) {
    Serial.println ("button is not pushed");
  }
  else {
    Serial.println ("button pushed");
  }
  delay(1500);

  float adc0 = mcp3021.readADC();
  float hum = map(adc0, air_value, water_value, moisture_0, moisture_100);
  Serial.println("water level is = " + String(hum, 1) + " %");
  delay(1500);

  float t = bme280.readTemperature();
  float h = bme280.readHumidity();
  float p = bme280.readPressure() / 100.0F;
  Serial.println("air temerature = " + String(t, 1) + " °C");
  Serial.println("air humidity = " + String(h, 1) + " %");
  Serial.println("pressure = " + String(p, 1) + " mm Hg");
  delay(1500);

  mySensor.measureAirQuality();
  Serial.println("volatile organic compounds = " + String(mySensor.TVOC) + " ppm");
  Serial.println("carbon dioxide (CO2) = " + String(mySensor.CO2) + " ppm");
  delay(1500);

  myservo.write(10);
  Serial.println("servo position = 10°");
  delay(1512);
  myservo.write(170);
  Serial.println("servo position = 170°");
  delay(1512);

  digitalWrite(wind, HIGH);         // включить, если нажата кнопка "Вентилятор" // turn on the cooler if button = 1
  Serial.println("cooler on");
  delay(1512);
  digitalWrite(wind, LOW);
  Serial.println("cooler off");

  note(14, 400); note(2, 100); note(9, 400); note(7, 500); note(14, 300); note(9, 700);
  buzzer.setVoltage(0, false); // выключение звука
  Serial.println("Wooooow, what a music!");
  delay(1500);

  ledcWrite(5, 0);
  Serial.println("gauge is at zero level");
  delay(1500);
  ledcWrite(5, 1024);
  Serial.println("gauge is at full level");
  delay(1500);
}

/////////////////////////////////////////////////////////////////////////// все для музыки
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
/////////////////////////////////////////////////// для MGS-FR403
void init_sensor() {
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

// Получение данных с датчика
void poll_sensor() {
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
