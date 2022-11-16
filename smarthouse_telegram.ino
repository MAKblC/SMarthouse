#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

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
PCA9634 ledsModul2(0x07);
PCA9634 ledsModul3(0x06);
#endif

#include <ESP32_Servo.h>                      // конфигурация сервомотора 
Servo myservo;
int pos = 1;            // начальная позиция сервомотора // servo start position
int prevangle = 1;      // предыдущий угол сервомотора // previous angle of servo

#include "SparkFun_SGP30_Arduino_Library.h"  // датчик газа
SGP30 mySensor;

#include <VL53L0X.h>    /// датчик расстояния
VL53L0X lox;
#define HIGH_ACCURACY

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

// параметры сети
#define WIFI_SSID "XXXXXXXX"
#define WIFI_PASSWORD "XXXXXXXXX"
// токен вашего бота
#define BOT_TOKEN "XXXXXXXX:XXXXXXXXXXXXXX"

#include "mcp3021.h"
uint8_t adcDeviceId =  0b00000001; // Адрес микросхемы A0 "0b00000000"
MCP3021 mcp3021;
const float air_value = 570.0; // калибровочные значения
const float water_value = 335.0;
const float moisture_0 = 0.0;
const float moisture_100 = 100.0;

#define  wind   17                     // пин вентилятора (16)

#include <BH1750FVI.h>        // добавляем библиотеку датчика освещенности // adding Light intensity sensor library  
BH1750FVI LightSensor_1;      // BH1750

#include <Adafruit_Sensor.h>  // добавляем библиотеку датчика температуры, влажности и давления // adding Temp Hum Bar sensor library
#include <Adafruit_BME280.h>  // BME280                         
Adafruit_BME280 bme280;       //

const unsigned long BOT_MTBS = 1000; // период обновления сканирования новых сообщений

WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);
unsigned long bot_lasttime;

// ссылка для поста фотографии
String test_photo_url = "https://mgbot.ru/upload/logo-r.png";

// отобразить кнопки перехода на сайт с помощью InlineKeyboard
String keyboardJson1 = "[[{ \"text\" : \"Ваш сайт\", \"url\" : \"https://mgbot.ru\" }],[{ \"text\" : \"Перейти на сайт IoTik.ru\", \"url\" : \"https://www.iotik.ru\" }]]";

void setup()
{
  myservo.attach(13);             // пин сервомотора // servo pin
  pinMode( wind, OUTPUT );       // настройка пинов насоса и вентилятора на выход // pump and cooler pins configured on output mode
  digitalWrite(wind, LOW);

  Serial.begin(115200);
  delay(512);
  Serial.println();
  Serial.print("Connecting to Wifi SSID ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

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

  buzzer.begin(0x61); // С перемычкой адрес будет 0x60
  buzzer.setVoltage(0, false);   // выключение звука

  if (mySensor.begin() == false)
    Serial.println("No SGP30 Detected. Check connections.");
  mySensor.initAirQuality();
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

  mcp3021.begin(adcDeviceId);
}

// функция обработки новых сообщений
void handleNewMessages(int numNewMessages)
{
  Serial.print("handleNewMessages ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++)
  {
    String chat_id = bot.messages[i].chat_id;
    String text = bot.messages[i].text;
    text.toLowerCase();
    String from_name = bot.messages[i].from_name;
    if (from_name == "")
      from_name = "Guest";

    // выполняем действия в зависимости от пришедшей команды
    if ((text == "/sensors") || (text == "sensors")) // измеряем данные
    {
      float dist = lox.readRangeSingleMillimeters();
      float l = LightSensor_1.getAmbientLight();
      float adc0 = mcp3021.readADC();
      float hum = map(adc0, air_value, water_value, moisture_0, moisture_100);
      float t = bme280.readTemperature();
      float h = bme280.readHumidity();
      float p = bme280.readPressure() / 100.0F;
      mySensor.measureAirQuality();

      String welcome = "Показания датчиков:\n";
      welcome += "Temp: " + String(t, 1) + " C\n";
      welcome += "Hum: " + String(h, 0) + " %\n";
      welcome += "Press: " + String(p, 0) + " hPa\n";
      welcome += "Light: " + String(l, 0) + " Lx\n";
      welcome += "Water level: " + String(hum, 0) + " %\n";
      welcome += "TVOC: " + String(mySensor.TVOC, 0) + " ppb\n";
      welcome += "CO2: " + String(mySensor.CO2, 0) + " ppm\n";
      welcome += "Distance: " + String(dist, 0) + " mm\n";
#ifdef MGS_A9
      lsm.read(); // данные гироскопа, акселерометра и магнетометра
      sensors_event_t a, m, g, temp;
      lsm.getEvent(&a, &m, &g, &temp);
      welcome += "accel x = " + String(a.acceleration.x, 1) + "\n";
      welcome += "accel y = " + String(a.acceleration.y, 1) + "\n";
      welcome += "accel z = " + String(a.acceleration.z, 1) + "\n";
#endif
#ifdef MGS_A6
      Vector rawGyro = mpu.readRawGyro(); // Сырые значения
      Vector normGyro = mpu.readNormalizeGyro(); // Преобразованные значения
      welcome += "accel x = " + String(normGyro.XAxis, 1) + "\n";
      welcome += "accel y = " + String(normGyro.YAxis, 1) + "\n";
      welcome += "accel z = " + String(normGyro.ZAxis, 1) + "\n";
#endif
      bot.sendMessage(chat_id, welcome, "Markdown");
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
      welcome += "color: r:" + String(red_data) + " g:" + String(green_data) + " b:" + String(blue_data) + "\n";
#endif
    }

    if (text == "/photo") { // пост фотографии
      bot.sendPhoto(chat_id, test_photo_url, "а вот и фотка!");
    }

    if ((text == "/windon") || (text == "windon"))
    {
      digitalWrite(wind, HIGH);
      bot.sendMessage(chat_id, "Вентилятор включен", "");
    }
    if ((text == "/windoff") || (text == "windoff"))
    {
      digitalWrite(wind, LOW);
      bot.sendMessage(chat_id, "Вентилятор выключен", "");
    }

    if ((text == "/light") || (text == "light"))
    {
#ifdef MGL_RGB1EN
      leds.setBrightness(0, 0xff);
      leds2.setBrightness(0, 0xff);
      leds3.setBrightness(0, 0xff);
#endif
#ifdef MGL_RGB3
      ledsModul.write1(3, 0xff);
      ledsModul2.write1(3, 0xff);
      ledsModul3.write1(3, 0xff);
#endif
      bot.sendMessage(chat_id, "Свет включен", "");
    }
    if ((text == "/off") || (text == "off"))
    {
#ifdef MGL_RGB1EN
      leds.setBrightness(0, 0x00);
      leds2.setBrightness(0, 0x00);
      leds3.setBrightness(0, 0x00);
#endif
#ifdef MGL_RGB3
      ledsModul.write1(3, 0x00);
      ledsModul2.write1(3, 0x00);
      ledsModul3.write1(3, 0x00);
#endif
      bot.sendMessage(chat_id, "Свет выключен", "");
    }
    if ((text == "/color") || (text == "color"))
    {
#ifdef MGL_RGB1EN
      leds.setBrightness(0, random(0, 255));
      leds2.setBrightness(0, random(0, 255));
      leds3.setBrightness(0, random(0, 255));
#endif
#ifdef MGL_RGB3
      ledsModul.write1(3, random(0, 255));
      ledsModul2.write1(3, random(0, 255));
      ledsModul3.write1(3, random(0, 255));
#endif
      bot.sendMessage(chat_id, "Включены случайные цвета", "");
    }

    if (text == "/site") // отобразить кнопки в диалоге для перехода на сайт
    {
      bot.sendMessageWithInlineKeyboard(chat_id, "Выберите действие", "", keyboardJson1);
    }

    if (text == "/options") // клавиатура для управления теплицей
    {
      String keyboardJson = "[[\"/light\", \"/off\"],[\"/color\",\"/sensors\"],[\"/sound\",\"/windon\", \"/windoff\"],[\"/open\",\"/close\"]]";
      bot.sendMessageWithReplyKeyboard(chat_id, "Выберите команду", "", keyboardJson, true);
    }

    if ((text == "/start") || (text == "start") || (text == "/help") || (text == "help")) // команда для вызова помощи
    {
      bot.sendMessage(chat_id, "Привет, " + from_name + "!", "");
      bot.sendMessage(chat_id, "Я контроллер Йотик 32. Команды смотрите в меню слева от строки ввода", "");
      String sms = "Команды:\n";
      sms += "/options - пульт управления\n";
      sms += "/site - перейти на сайт\n";
      sms += "/photo - запостить фото\n";
      sms += "/help - вызвать помощь\n";
      bot.sendMessage(chat_id, sms, "Markdown");
    }

    if (text == "/sound")
    {
      note(14, 400); note(2, 100);
      buzzer.setVoltage(0, false); // выключение звука
    }
    if (text == "/open")
    {
      myservo.write(100);
      bot.sendMessage(chat_id, "дверь открыта", "");
    }
    if (text == "/close")
    {
      myservo.write(0);
      bot.sendMessage(chat_id, "дверь закрыта", "");
    }
  }
}

void loop() // вызываем функцию обработки сообщений через определенный период
{
  if (millis() - bot_lasttime > BOT_MTBS)
  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages)
    {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    bot_lasttime = millis();
  }
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
