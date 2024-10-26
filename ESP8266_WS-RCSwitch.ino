/*
   Метеостанция на ESP8266 от it4it.club
   https://it4it.club/topic/55-meteostantsiya-na-esp8266-ot-it4itclub/

   Задействованные пины
   ---------------------------------------
   NodeMCU Pins | ESP 07/12 Pins | Датчики
   ---------------------------------------
        D2      |      GPIO 4    |   SDA
        D1      |      GPIO 5    |   SCL
        3.3V    |      VCC       |   VCC
        GND     |      GND       |   GND
*/
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include <Wire.h>
#include <MD5Builder.h>
#include <Ticker.h>
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson

#include <SoftwareSerial.h>

#define MH_Z19_RX D7
#define MH_Z19_TX D8

SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX); // define MH-Z19

extern "C" {
#include "user_interface.h"
#include "espconn.h"
}
/*
   Для хранения конфигурации будем использовать SPIFFS
   config.h описывает простой способ чтения и записи
*/
#include "config.h"
config config;
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN            12         // Pin which is connected to the DHT sensor.
//#define DHTTYPE           DHT11     // DHT 11
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)
DHT dht(DHTPIN, DHTTYPE);
/*
   датчик атмосферного давления и температуры
   https://github.com/adafruit/Adafruit-BMP085-Library
*/
#include <Adafruit_BMP085.h>
Adafruit_BMP085 BMP085;

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 20, 4); // Устанавливаем дисплей

#include "uRTCLib.h"
uRTCLib rtc;
#define TIMEZONE 3
#include <TimeLib.h>


unsigned int  localPort = 2390;      // local port to listen for UDP packets
unsigned long ntp_time = 0;

IPAddress timeServerIP;
const char* ntpServerName = "ntp1.vniiftri.ru";

const int NTP_PACKET_SIZE = 48;
byte packetBuffer[ NTP_PACKET_SIZE];
WiFiUDP udp;




#define button1B 16  // пин кнопки button1

boolean button1S;   // храним состояния кнопок (S - State)
boolean button1F;   // флажки кнопок (F - Flag)
boolean button1R;   // флажки кнопок на отпускание (R - Release)
boolean button1P;   // флажки кнопок на нажатие (P - Press)
boolean button1H;   // флажки кнопок на удержание (H - Hold)
boolean button1D;   // флажки кнопок на двойное нажатие (D - Double)
boolean button1DP;  // флажки кнопок на двойное нажатие и отпускание (D - Double Pressed)

boolean backlighton = 0;

boolean chs_s = 0;

boolean co2init1 = 0;
boolean co2init2 = 1;
boolean co2ready = 0;
int co2err = 0;

#define double_timer 2000   // время (мс), отведённое на второе нажатие
#define hold 5000           // время (мс), после которого кнопка считается зажатой
#define debounce 200        // (мс), антидребезг
unsigned long button1_timer; // таймер последнего нажатия кнопки
unsigned long button1_double; // таймер двойного нажатия кнопки
unsigned long nextTimeBut = 0;   // таймер подсветки
unsigned long nextTimeRTC = 0;  // таймер для RTC
unsigned long nextTimeNTP = 0;  // таймер для NTP


#include <RCSwitch.h>

int temext = 0;
int humext = 0;
int batext = 0;
RCSwitch ExtSensor = RCSwitch();

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print (args,BYTE);
#endif
//Создаем массив значений на дисплее, от пустого к полному.
uint8_t graf0[ 8 ] = { 0x1F , 0x1F , 0x1F , 0x1F , 0x1F , 0x1F , 0x1F };
uint8_t graf1[ 8 ] = {B00000, B11111, B11111, B11111, B11111, B11111, B11111};
uint8_t graf2[ 8 ] = {B00000, B00000, B11111, B11111, B11111, B11111, B11111};
uint8_t graf3[ 8 ] = {B00000, B00000, B00000, B11111, B11111, B11111, B11111};
uint8_t graf4[ 8 ] = {B00000, B00000, B00000, B00000, B11111, B11111, B11111};
uint8_t graf5[ 8 ] = {B00000, B00000, B00000, B00000, B00000, B11111, B11111};
uint8_t graf6[ 8 ] = {B00000, B00000, B00000, B00000, B00000, B00000, B11111};
uint8_t graf7[ 8 ] = {B00000, B00000, B00000, B00000, B00000, B00000, B00000};
//Добавили символы для графиков

int stat[ 2 ][ 25 ]; //Возьмем 2х мерный массив, где один массив время (контроль действительности значения) и второй массив наши данные
//int stat[0]; //Давление
//int stat[1]; //Время

/*
   OTA
*/
bool ota = false;
const char * otaHost = "espWeatherStation";
const char * otaPass = "";
/*
   Переменные для реализации автоматического выбора режима работы WiFi
*/
const bool client = false;
const bool ap = true;
bool modeWiFi;
unsigned long regimeChangeTimer = 0;
unsigned long lostConnectionTimer = 0;
bool eventSTAconnected = false;
byte failConnections = 0;
unsigned long connectionBreakTimer = 0;
/*
   DNS сервер/клиент
*/
const byte DNS_PORT = 53;
DNSServer dnsServer;
bool dnsClient = false;
const char* dnsClientName = "espws";
/*
   имя и ключ для cookies
*/
String cookiesName = "esp-ws";
String cookiesKey;
/*
   счетчики и таймеры
*/
byte countAuthorizationFail = 0;          // счетчик неудачный попытов авторизации
unsigned long timerAuthorizationBan = 0;  // таймер для блокировки системы авторизации
unsigned long timerReadSensors = 0;       // таймер для переодического чтения показаний датчиков
unsigned long timerReset = 0;             // таймер для псевдо WD, сбрасывает контроллер при остановке loop()
unsigned long timerMQTT = 0;              // таймер для переодической отправки данных на MQTT сервер
unsigned long timerNarodmon = 0;          // таймер для переодической отправки данных на ресурс narodmon.ru
unsigned long timerDnsUpdate = 0;         // таймер для обновления записи на внешнем DNS сервере
unsigned long timerLogWrite = 0;          // таймер для переодической записи показаний в журнах
/*
   i2c шина
   задейстованные пины SDA и SCL
*/
#define pin_sda 4 // default sda pin
#define pin_scl 5 // default scl pin
/*
   WEB сервер
*/
ESP8266WebServer webServer(80);
File spiffsUploadFile;
String spiffsUploadFileName = "";
/*
   MQTT
*/
WiFiClient espMqttClient;
PubSubClient mqtt(espMqttClient);
/*
   MD5 hash
*/
MD5Builder _md5;
String md5(String str) {
  _md5.begin();
  _md5.add(String(str));
  _md5.calculate();
  return _md5.toString();
}
/*
   структура и переменные для хранения показаний датчиков
*/
struct sensor {
  float data = 0;         // последние показания полученные от датчика
  bool  status = false;   // статус датчика
  float log[144];         // журнал показаний датчика
  int   logPosition = 0;  // следующая позиция в журнале для записи показаний датчика
};
//sensor light;              // Уровень освещенности (lx)
sensor temperature;        // Температура (*C)
sensor pressure;           // Давление (mm)
sensor humidity;           // Влажность (%)
sensor temperatureExt;     // Температура (*C) Внешняя
sensor humidityExt;        // Влажность (%) Внешняя
sensor carbdiox;           // CO2 (ppm)
sensor batteryExt;         // Внешняя батарея
/*
  прерывания
*/
Ticker interrupt1;
Ticker interrupt2;
/*
  структура для реализации индикации состояния устройства через мигание светодиода
*/
struct blink {
  byte current  = 0; // текущее состояние
  byte step     = 0; // битовый шаг
  byte mode[7]  = {  // интервалы индикации
    0B00000000, // Светодиод выключен
    0B00000001, // Короткая вспышка раз в секунду
    0B00000101, // Две короткие вспышки раз в секунду
    0B00010101, // Три короткие вспышки раз в секунду
    0B01010101, // Частые короткие вспышки (4 раза в секунду)
    0B11111111, // Горит постоянно
    0B00001111  // Мигание по 0.5 сек
  };
  byte pin_led = 2;       // пин к которому подключен светодиод (2 для ESP-12)
};
blink status;
/*
   Измерение напряжения
*/
ADC_MODE(ADC_VCC);
/*

*/
void setup() {


  //stat[ 1 ][ 17 ] = -1;

  /*
    Для теста
    Максимальное количество TCP соединений, по умолчанию 5
  */
  espconn_tcp_set_max_con(5);
  /*
     Инициализация пинов
  */
  pinMode(status.pin_led, OUTPUT);
  digitalWrite(status.pin_led, HIGH);
  /*
     Инициализация консоли
     Желательно полностью избавиться от вывода в консоль
  */
  Serial.begin(115200);
  Serial.println("\n");
  /*
     Возможность шить контроллер по сети через Arduino IDE
  */
  if (ota) {
    ArduinoOTA.setPort(8266);
    ArduinoOTA.setHostname(otaHost);
    ArduinoOTA.setPassword(otaPass);
    ArduinoOTA.begin();
    Serial.println("WARNING: OTA Enable!!!\n");
  }
  /*
     Информация о флеш памяти
  */
  flashInfo();
  /*
     Инициализация шины i2c
  */
  Wire.begin(pin_sda, pin_scl);
  /*
     Инициализация сенсоров на шине i2c
     BH1750, SI7021, BMP085
  */
  pinMode(button1B, INPUT_PULLUP);
  //  gpio.mode(5,gpio.INPUT,gpio.PULLUP)
  nextTimeBut = millis() + 120000;
  nextTimeNTP = millis() + 600000;

  lcd.init();
  lcd.backlight();// Включаем подсветку дисплея
  lcd.print("iT4iT.CLUB");
  lcd.setCursor(7, 1);
  lcd.print("METEO");
  lcd.setCursor(6, 2);
  lcd.print("STATION");
  lcd.setCursor(12, 3);
  lcd.print("LCD 2004");

  lcd.createChar( 0 , graf0); // Подключаем доп. символы
  lcd.createChar( 1 , graf1);
  lcd.createChar( 2 , graf2);
  lcd.createChar( 3 , graf3);
  lcd.createChar( 4 , graf4);
  lcd.createChar( 5 , graf5);
  lcd.createChar( 6 , graf6);
  lcd.createChar( 7 , graf7);

  delay(3000);
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("LOADING");

  ExtSensor.enableReceive(14);


  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());



  //light.status = true;
  dht.begin();
  temperature.status = humidity.status = true;
  pressure.status = BMP085.begin();

  co2Serial.begin(9600); //Init sensor MH-Z19(14)

  /*
     File system object (SPIFFS)
     https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#file-system-object-spiffs

     |--------------|-------|---------------|--|--|--|--|--|
     ^              ^       ^               ^     ^
     Sketch    OTA update   File system   EEPROM  WiFi config (SDK)
  */
  if (SPIFFS.begin()) {
    //SPIFFS.remove(config.file);
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      Serial.printf("FS File: %s, size: %s\r\n", fileName.c_str(), formatBytes(fileSize).c_str());
    }
    Serial.println();
    if (!config.load()) config.save();
  } else ESP.restart();
  /*
     Поднимаем WiFi сеть
     В данной функции необходимо реализовывать логику переключения режимов ST и AP
  */
  WiFi.onEvent(WiFiEvent);
  if (!modeClient()) modeAP();
  /*
     webServer
     будем анализировать header заголовки запросов
  */
  // const char * headerkeys[] = {"User-Agent", "Cookie", "Accept-Encoding", "If-None-Match", "Origin"};
  const char * headerkeys[] = {"User-Agent", "Cookie", "Accept-Encoding", "If-None-Match"};
  size_t headerkeyssize = sizeof(headerkeys) / sizeof(char*);
  webServer.collectHeaders(headerkeys, headerkeyssize);
  /*
     webServer
     1. /api/sensors       (GET)  - обработка запросов показаний датчиков
     2. /api/sensors/log   (GET)  - список показаний датчиков за последнии 24 часа (144 points * 10 min = 24h)
     2. /api/settings      (POST) - работа с настройками
     3. /api/spiffs        (POST) - работа с файловой стистемой флеш памяти
     4. /api/spiffs        (GET)  - список файлов на флеш памяти
     5. /api/spiffs/delete (POST) - удаление файлов web сервера
     6. /api/system/info   (POST) - системная информация
     7. /api/system/reboot (POST) - перезагрузка микроконтроллера
     8. /api/system/hardReset (POST) - сброс всех настроек
     9. /api/system/i2c       (POST) - список устройств на i2c шине
    10. /api/system/update    (POST) - обновление прошивки микроконтроллера через http протокол
  */
  webServer.on("/api/sensors",          HTTP_GET,  web_api_sensors);
  webServer.on("/api/sensors/log",      HTTP_GET,  web_api_sensors_log);
  webServer.on("/api/settings",         HTTP_POST, web_api_settings);
  webServer.on("/api/spiffs",           HTTP_POST, web_api_spiffs_upload, web_api_spiffs_upload_header);
  webServer.on("/api/spiffs",           HTTP_GET,  web_api_spiffs_list);
  webServer.on("/api/spiffs/delete",    HTTP_POST, web_api_spiffs_delete);
  webServer.on("/api/system/info",      HTTP_POST, web_api_system_info);
  webServer.on("/api/system/reboot",    HTTP_POST, web_api_system_reboot);
  webServer.on("/api/system/hardReset", HTTP_POST, web_api_system_hardReset);
  webServer.on("/api/system/i2c",       HTTP_GET,  web_api_system_i2c_scaner);
  webServer.on("/api/system/update",    HTTP_POST, web_api_system_update, web_api_system_update_header);
  webServer.onNotFound(web_error_404);

  // генерируем ключ пустышку
  cookiesKey = salt(32);
  // устанавливаем прерывания
  interrupt1.attach(10, hangTest);
  interrupt2.attach_ms(125, displayStatus);
  // первое чтение данных с датчиков
  readSensors(); sensorLogWrite();
}
/*
  Индикация состояния устройства
*/
void displayStatus() {
  if (status.step > 7) status.step = 0;
  if (status.current) digitalWrite(status.pin_led, (status.mode[status.current] >> status.step++ & 1) ? LOW : HIGH);
}
/*
  Сброс контроллера при offline более 120 секунд
*/
void hangTest() {
  if (millis() - timerReset > 1200000) {
    status.current = 5;
    ESP.restart();
  }
}
void hangTestReset() {
  timerReset = millis();
}
/*
  Обработчик событий WiFi
*/
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    /*
      Используемые события
    */
    case WIFI_EVENT_STAMODE_CONNECTED:
      status.current = 1;
      eventSTAconnected = true;
      if (lostConnectionTimer != 0) lostConnectionTimer = 0;
      if (regimeChangeTimer != 0) regimeChangeTimer = 0;
      Serial.println("connected successfully");
      break;
    case WIFI_EVENT_STAMODE_DISCONNECTED:
      status.current = 0;
      if (lostConnectionTimer == 0 and eventSTAconnected) {
        lostConnectionTimer = millis();
        eventSTAconnected = false;
        Serial.println("connection lost");
      }
      break;
    case WIFI_EVENT_SOFTAPMODE_STACONNECTED:
      Serial.printf("a new connection, all connected users: %i\n", WiFi.softAPgetStationNum());
      break;
    case WIFI_EVENT_SOFTAPMODE_STADISCONNECTED:
      Serial.printf("disconnection, all connected users: %i\n", WiFi.softAPgetStationNum());
      break;
    /*
       Не используемые события
    */
    case WIFI_EVENT_STAMODE_AUTHMODE_CHANGE:
    case WIFI_EVENT_STAMODE_DHCP_TIMEOUT:
    case WIFI_EVENT_STAMODE_GOT_IP:
    case WIFI_EVENT_SOFTAPMODE_PROBEREQRECVED:
    case WIFI_EVENT_MODE_CHANGE:
      break;
  }
}
/*
   WiFi
   Режим клиента
*/
bool modeClient() {
  Serial.println("WiFi mode client");
  if (config.client_ssid.length()) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.client_ssid.c_str(), config.client_pass.length() ? config.client_pass.c_str() : NULL);
  } else return false;
  Serial.println("connecting to " + config.client_ssid);
  unsigned long time = millis();
  while (!WiFi.isConnected()) {
    delay(10);
    if (millis() - time > 30000) {
      Serial.println("connecting fail");
      WiFi.disconnect();
      delay(10);
      return false;
    }
  }
  if (!WiFi.getAutoConnect()) {
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
  }

  Serial.println();
  Serial.print("mac: ");
  Serial.println(WiFi.macAddress());
  Serial.print("hostname: ");
  Serial.println(WiFi.hostname());
  Serial.print("ip: ");
  Serial.println(WiFi.localIP());
  Serial.print("subnet mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("dns server: ");
  Serial.println(WiFi.dnsIP());
  Serial.println();

  webServer.begin();

  if ((dnsClient = MDNS.begin(dnsClientName)) == true) {
    MDNS.addService("http", "tcp", 80);
    timerDnsUpdate = millis();
  }

  modeWiFi = client;
  status.current = 1;
  lcd.setCursor(18, 1);
  lcd.print("CL");
  return true;
}
/*
   WiFi
   Режим точки доступа
*/
bool modeAP() {
  Serial.println("WiFi mode AP\n");
  if (config.softap_ssid.length()) {
    IPAddress apIP(10, 10, 10, 1);
    IPAddress netMsk(255, 255, 255, 0);

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, netMsk);
    WiFi.softAP(config.softap_ssid.c_str(), config.softap_pass.length() ? config.softap_pass.c_str() : NULL);

    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, String(dnsClientName) + ".local", apIP);
  } else return false;
  webServer.begin();
  modeWiFi = ap;
  regimeChangeTimer = millis();
  status.current = 2;
  lcd.setCursor(18, 1);
  lcd.print("AP");
  if (dnsClient) {
    dnsClient = false;
    timerDnsUpdate = 0;
  }
  return true;
}
/*
   HTTP
   Обновление прошивки микроконтроллера через web интерфейс
   HTTPMethod
*/
void web_api_system_update() {
  webServer.sendHeader("Connection", "close");
  if (authorized()) {
    if (!Update.hasError()) {
      webServer.sendHeader("Set-Cookie", cookiesName + "=0; Path=/; Expires=Thu, 02 Apr 1987 00:00:01 GMT");
      webServer.send(200, "text/plain", "{\"status\":true}");
      ESP.restart();
    } else webServer.send(200, "text/plain", "{\"status\":false,\"error\":" + String(Update.getError()) + "}");
  }
  else webServer.send(200, "text/plain", "{\"status\":false}");
}
/*
   HTTP
   Обновление прошивки микроконтроллера через web интерфейс
   THandlerFunction
*/
void web_api_system_update_header() {
  if (!authorized()) return;
  HTTPUpload& upload = webServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    //Serial.setDebugOutput(true);
    WiFiUDP::stopAll();
    Serial.printf("Firmware update. MD5: %s\n", upload.filename.c_str()); // MD5 in the update file name
    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (!Update.begin(maxSketchSpace)) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_END) {
    Update.setMD5(upload.filename.c_str());
    if (Update.end(true)) Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
    else Update.printError(Serial);
    //Serial.setDebugOutput(false);
  } else if (upload.status == UPLOAD_FILE_ABORTED) Update.end();
}


/*












*/
/*
   Основной цикл программы
*/
void loop() {

  lcdbacklight();

  if (millis() - nextTimeRTC > 250 or nextTimeRTC > millis()) {
    ShowTime();
    nextTimeRTC = millis();
  }

  if (modeWiFi == client) {
    if (nextTimeNTP <= millis()) {
      if ( GetNTP() ) {
        UpdateTime();
        nextTimeNTP = millis() + 43200000;
      }
    }
  }


  if (ota) ArduinoOTA.handle();
  // Сбрасываем псевдо сторожевой таймер
  hangTestReset();
  // Обрабатываем пользовательские http запросы
  webServer.handleClient();
  // Запросы к DNS серверу ESP8266 обрабатываем только в режиме AP
  if (modeWiFi == ap) dnsServer.processNextRequest();
  // Запросы к внешнему DNS серверу отправляем раз в час и только в режиме STA
  if (modeWiFi == client and dnsClient and (millis() > timerDnsUpdate + 3600000 or timerDnsUpdate > millis())) {
    MDNS.update();
    timerDnsUpdate = millis();
  }
  // Переводим контроллер для работы в режиме AP при потери связи с домашним WiFi
  if (modeWiFi == client and lostConnectionTimer != 0 and (millis() > lostConnectionTimer + 30000 or lostConnectionTimer > millis())) {
    lostConnectionTimer = 0;
    webServer.stop();
    WiFi.disconnect();
    modeAP();
  }
  // При потери связи с базовой станцией переходим в режим точки доступа и переодически ищем в эфире домашнюю сеть
  if (modeWiFi != client and regimeChangeTimer != 0 and config.client_ssid.length() and connectionBreakTimer == 0) {
    if (millis() > regimeChangeTimer + 30000 or regimeChangeTimer > millis()) {
      Serial.println("scanning of radio in search of a \"" + config.client_ssid + "\" access point");
      int n = WiFi.scanNetworks(false, true);
      if (n > 0) {
        bool find = false;
        String protect, hidden;
        for (int i = 0; i < n; ++i) {
          if (WiFi.SSID(i) == String(config.client_ssid)) find = true;
          protect = WiFi.encryptionType(i) == ENC_TYPE_NONE ? "<!>" : "protected";
          hidden = WiFi.isHidden(i) ? "*hidden" : " ";
          Serial.printf("%i: %s (%idBm) %s %s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), protect.c_str(), hidden.c_str());
        }
        if (find) {
          status.current = 0;
          Serial.println("wireless network is found, connect ...");
          dnsServer.stop();
          webServer.stop();
          WiFi.softAPdisconnect();
          if (!modeClient()) {
            modeAP();
            if (++failConnections >= 1) {
              connectionBreakTimer = millis();
              Serial.println("The next connection attempt will be made after 10 minutes!");
            }
          } else failConnections = 0;
        }
      } else Serial.println("no networks are available");
      Serial.println();
      regimeChangeTimer = millis();
    }
  }
  // Разрешаем повторные подключения к базовой станции после 5 минут ожидания после 3 неудачных подключений подряд
  if (connectionBreakTimer != 0 and (millis() > connectionBreakTimer + 5 * 60000 or connectionBreakTimer > millis())) connectionBreakTimer = failConnections = 0;
  // Снимаем ограничение на доступ к системе авторизации
  if (timerAuthorizationBan != 0 and (millis() > timerAuthorizationBan + 60000 or timerAuthorizationBan > millis())) countAuthorizationFail = timerAuthorizationBan = 0;

  // Читаем показания с датчиков каждые 5 сек
  if (millis() > timerReadSensors + 5000 or timerReadSensors > millis()) readSensors();
  // Записываем показания с датчикв в журнал каждые 10 минут (144 points * 10 min = 24h)
  if (millis() > timerLogWrite + 600000 or timerLogWrite > millis()) sensorLogWrite();
  // Отправка данных на внешние ресурсы только в режиме клиента
  if (modeWiFi == client) {
    // Отправка данных на ресурс narodmon.ru с интервалом 10 минут
    if (config.narodmon_id.length()) {
      if (millis() - timerNarodmon > 600000 or timerNarodmon > millis()) {
        Serial.println("narodmon try");
        HTTPClient httpClient;
        String query;
        
        //query += "&L1=" + String(light.data);
        query += "&T1=" + String(temperature.data);
        query += "&H1=" + String(humidity.data);
        query += "&P1=" + String(pressure.data);
        query += "&T2=" + String(temperatureExt.data);
        query += "&H2=" + String(humidityExt.data);
        query += "&C1=" + String(carbdiox.data);
        query += "&B1=" + String(batteryExt.data);
        httpClient.begin("narodmon.ru", 80, "/get?ID=" + config.narodmon_id + query + "&NAME=ESP8266 Meteo");
        httpClient.setUserAgent("weather station (ESP8266) iT4iT.CLUB");
        httpClient.setTimeout(5000);
        httpClient.GET();
        httpClient.end();
        timerNarodmon = millis();
        Serial.println("/get?ID=" + config.narodmon_id + query + "&NAME=ESP8266 Meteo");
        Serial.println("narodmon ok");
      }
    }
    // Отправка данный на MQTT сервер с интервалом 5 минут
    if (config.mqtt_server.length()) {
      if (millis() - timerMQTT > 300000 or timerMQTT > millis()) {
        mqttSendData();
        timerMQTT = millis();
      }
    }
  }
  CheckExtSens();
}


/*















*/
/*
  Прием данных с внешнего датчика
*/
void CheckExtSens() {

  if (ExtSensor.available()) {
    Serial.println("433 receive");

    unsigned  long msg = 1000000000;
    msg = ExtSensor.getReceivedValue();

    if (msg == 0) {
      Serial.print("Unknown encoding");
    } else {
      Serial.print("Received ");
      Serial.print( ExtSensor.getReceivedValue() );
      Serial.print(" / ");
      Serial.print( ExtSensor.getReceivedBitlength() );
      Serial.print("bit ");
      Serial.print("Protocol: ");
      Serial.println( ExtSensor.getReceivedProtocol() );

      if ( (ExtSensor.getReceivedProtocol() == 2) && (msg / 1000000000 == 1) )
      {
        int chs = 99;
        int battery = 0;
        //        msg = ExtSensor.getReceivedValue();
        temext = ((msg % 10000000) / 10000) - 400;
        humext = ((msg % 1000000000) / 10000000) - random(3);
        batext = (msg % 10000) / 10;

        battery = map(batext, 320, 395, 0, 9);
        battery = constrain(battery, 0, 9);

        batext = batext * 10;
        batext = constrain(batext, 0, 9999);

        chs = (msg / 1000000000) + ((msg % 1000000000) / 100000000) + ((msg % 100000000) / 10000000) + ((msg % 10000000) / 1000000) + ((msg % 1000000) / 100000) + ((msg % 100000) / 10000) + ((msg % 10000) / 1000) + ((msg % 1000) / 100) + ((msg % 100) / 10);
        if (chs > 9 ) chs = (chs / 10) + (chs % 10);
        if (chs > 9 ) chs = (chs / 10) + (chs % 10);

        //      chs = msg % 10;

        Serial.print("msg ");
        Serial.print( msg );
        Serial.print(" t ");
        Serial.print( temext );
        Serial.print(" h ");
        Serial.print( humext );
        Serial.print(" ch ");
        Serial.print( chs );
        Serial.print(" battery ");
        Serial.print( battery );
        Serial.println(  );

        lcd.setCursor(0, 2);
        lcd.print("           ");
        lcd.setCursor(0, 2);
        if (humext < 10) lcd.print(" ");
        lcd.print(humext);
        lcd.print("%");

        lcd.setCursor(4, 2);
        if (temext < 100 && temext >= 0)
          lcd.print("  ");
        else if (temext >= 100 )
          lcd.print(" ");
        else if (temext < 0 && temext > -100)
          lcd.print(" ");
        lcd.print( temext / 10.0 , 1 );
        lcd.print("\xDF");

        //     if ( chs == 77 )        // 2 последних символа в датчика
        //       lcd.print(" + ");
        //       else lcd.print(" - ");

        lcd.setCursor(16, 2);
        if      (battery > 8)    {
          lcd.printByte( 0 );
        }
        else if (battery > 7)    {
          lcd.printByte( 1 );
        }
        else if (battery > 6)    {
          lcd.printByte( 2 );
        }
        else if (battery > 5)    {
          lcd.printByte( 3 );
        }
        else if (battery > 4)    {
          lcd.printByte( 4 );
        }
        else if (battery > 3)    {
          lcd.printByte( 5 );
        }
        else if (battery > 2)    {
          lcd.printByte( 6 );
        }
        else    {
          lcd.print("\xDB");
        }



        if (chs == (msg % 10))
        {
          chs_s = 1;
        }
        else
        {
          chs_s = 0;
          Serial.print(" Chs INCOR ");
        }

      }


      /*     else if ( (ExtSensor.getReceivedProtocol() == 2) && ((msg / 1000000000) == 2) )
            {
        //        msg = ExtSensor.getReceivedValue();
              batext = (msg % 1000000000) / 100000;
            }

           else {Serial.println( "Wrong ReceivedProtocol" );}
      */
    }

    ExtSensor.resetAvailable();
  }
}


/*
  Считывание данныз СО2
*/
int readCO2()
{

  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  // command to ask for data
  char response[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // for answer


  for (int z = 0; z < 9; z++)
  {
    //    Serial.write(response[z]);
    Serial.print(response[z], HEX);
    Serial.print("-");
  }
  Serial.println();


  co2Serial.write(cmd, 9); //request PPM CO2
  co2Serial.readBytes(response, 9);

  for (int z = 0; z < 9; z++)
  {
    Serial.print(response[z], HEX);
    Serial.print("-");
  }
  Serial.println();


  if (response[0] == 0x00)
  {
    Serial.println("No co2 sensor!");
    return -1;
  }

  if (response[0] != 0xFF)
  {
    if (response[1] != 0xFF)
    {
      Serial.println("Wrong starting byte from co2 sensor!");
      co2err++;
      if (co2err > 24)     //co2err > 3 --  так надо
      {
        lcd.clear();
        lcd.setCursor(6, 0);
        lcd.print("RESTART (co2)e1");
        delay(5000);
        ESP.restart();
      }
      return -1;
    }
    else
    { if (response[2] != 0x86)
      {
        Serial.println("Wrong command from co2 sensor!");
        co2err++;
        if (co2err > 24)
        {
          lcd.clear();
          lcd.setCursor(6, 0);
        lcd.print("RESTART (co2)e2");
        delay(5000);
          ESP.restart();
        }
        return -1;
      }
      int responseHigh = (int) response[3];
      int responseLow = (int) response[4];
      int ppm = (256 * responseHigh) + responseLow;
      co2err = 0;
      return ppm;
    }
    return -1;
  }

  if (response[1] != 0x86)
  {
    Serial.println("Wrong command from co2 sensor!");
    co2err++;
    if (co2err > 24)
    {
      lcd.clear();
      lcd.setCursor(6, 0);
        lcd.print("RESTART (co2)e2");
        delay(5000);
      ESP.restart();
    }
    return -1;
  }

  int responseHigh = (int) response[2];
  int responseLow = (int) response[3];
  int ppm = (256 * responseHigh) + responseLow;
  co2err = 0;
  return ppm;
}



/**
   Выдача текущего NTP времени в серийный порт и обновление в RTC
*/
void UpdateTime(void) {

  time_t t = ntp_time;
  Serial.print("Weekday: ");
  Serial.println(weekday(t));
  Serial.print("Time: ");
  Serial.print(hour(t));
  Serial.print(":");
  Serial.print(minute(t));
  Serial.print(":");
  Serial.print(second(t));
  Serial.print(" Date: ");
  Serial.print(day(t));
  Serial.print("-");
  Serial.print(month(t));
  Serial.print("-");
  Serial.println(year(t));

  rtc.set(second(t), minute(t), hour(t), weekday(t), day(t), month(t), year(t) - 2000);
  //  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
}


/**
   Посылаем и парсим запрос к NTP серверу
*/
bool GetNTP(void) {
  WiFi.hostByName(ntpServerName, timeServerIP);
  sendNTPpacket(timeServerIP);
  delay(1000);

  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("No packet yet");
    return false;
  }
  else {
    Serial.print("packet received, length=");
    Serial.println(cb);
    // Читаем пакет в буфер
    udp.read(packetBuffer, NTP_PACKET_SIZE);
    // 4 байта начиная с 40-го сождержат таймстамп времени - число секунд
    // от 01.01.1900
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // Конвертируем два слова в переменную long
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    // Конвертируем в UNIX-таймстамп (число секунд от 01.01.1970
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;
    // Делаем поправку на местную тайм-зону
    ntp_time = epoch + TIMEZONE * 3600;
    Serial.print("Unix time = ");
    Serial.println(ntp_time);
  }
  return true;
}

/**
   Посылаем запрос NTP серверу на заданный адрес
*/
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // Очистка буфера в 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Формируем строку зыпроса NTP сервера
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // Посылаем запрос на NTP сервер (123 порт)
  udp.beginPacket(address, 123);
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}



/*
    Функция отвечает за подсветку дисплея
*/
void lcdbacklight() {

  //-------опрос кнопок--------
  button1S = !digitalRead(button1B);
  buttons1(); //отработка кнопок
  //-------опрос кнопок--------

  // отработка режимов (опускание флага обязательно!)
  if (button1P) {
    Serial.println("pressed");
    button1P = 0;
    lcd.backlight();// Включаем подсветку дисплея
    nextTimeBut = millis() + 10000;
    if ( backlighton )
    {
      backlighton = !backlighton;
    }
  }
  if (button1H) {
    Serial.println("hold");
    button1H = 0;
  }
  if (button1D) {
    Serial.println("double");
    button1D = 0;
    backlighton = !backlighton;
  }

  if ( backlighton )
  {
    lcd.setCursor(19, 0);
    lcd.print("*");
    lcd.backlight();// Включаем подсветку дисплея
  }
  else
  {
    lcd.setCursor(19, 0);
    lcd.print(" ");
  }

  if (nextTimeBut <= millis() && !backlighton) {
    //dooooo
    lcd.noBacklight();
    nextTimeBut = millis() + 5000;
  }
}


//------------------------ОТРАБОТКА КНОПОК-------------------------
void buttons1() {
  //-------------------------button1--------------------------
  // нажали (с антидребезгом)
  if (button1S && !button1F && millis() - button1_timer > debounce) {
    button1F = 1;
    button1_timer = millis();
  }
  // если отпустили до hold, считать отпущенной
  if (!button1S && button1F && !button1R && !button1DP && millis() - button1_timer < hold) {
    button1R = 1;
    button1F = 0;
    button1_double = millis();
  }
  // если отпустили и прошло больше double_timer, считать 1 нажатием
  if (button1R && !button1DP && millis() - button1_double > double_timer) {
    button1R = 0;
    button1P = 1;
  }
  // если отпустили и прошло меньше double_timer и нажата снова, считать что нажата 2 раз
  if (button1F && !button1DP && button1R && millis() - button1_double < double_timer) {
    button1F = 0;
    button1R = 0;
    button1DP = 1;
  }
  // если была нажата 2 раз и отпущена, считать что была нажата 2 раза
  if (button1DP && millis() - button1_timer < hold) {
    button1DP = 0;
    button1D = 1;
  }
  // Если удерживается более hold, то считать удержанием
  if (button1F && !button1D && !button1H && millis() - button1_timer > hold) {
    button1H = 1;
  }
  // Если отпущена после hold, то считать, что была удержана
  if (!button1S && button1F && millis() - button1_timer > hold) {
    button1F = 0;
    button1H = 0;
  }
  //-------------------------button1--------------------------
}
//------------------------ОТРАБОТКА КНОПОК-------------------------

/*
  Вывод времени с RTC
*/
void ShowTime() {
  rtc.refresh();
  int day = rtc.day();
  int month = rtc.month();
  int hour = rtc.hour();
  int minute = rtc.minute();
  int second = rtc.second();
  lcd.setCursor(0, 0);
  if (day < 10) lcd.print("0");
  lcd.print(day);
  lcd.print("-");
  if (month < 10) lcd.print("0");
  lcd.print(month);
  lcd.print("-20");
  lcd.print(rtc.year());
  lcd.print(" ");
  if (hour < 10) lcd.print("0");
  lcd.print(hour);
  lcd.print(":");
  if (minute < 10) lcd.print("0");
  lcd.print(minute);
  lcd.print(":");
  if (second < 10) lcd.print("0");
  lcd.print(second);
}



/*
    Функция отвечает за отправку данных на MQTT сервер
*/
bool mqttSendData() {
  Serial.println("MQTT try");
  mqtt.setServer(config.mqtt_server.c_str(), 1883);
  if (config.mqtt_login.length()) mqtt.connect("weather station", config.mqtt_login.c_str(), config.mqtt_pass.c_str());
  else mqtt.connect("weather station");
  if (mqtt.connected()) {
    //mqttPublish("/light", light.data);
    mqttPublish("temperature", temperature.data);
    mqttPublish("humidity", humidity.data);
    mqttPublish("pressure", pressure.data);
    mqttPublish("temperatureExt", temperatureExt.data);
    mqttPublish("humidityExt", humidityExt.data);
    mqttPublish("carbdiox", carbdiox.data);
    mqttPublish("batteryExt", batteryExt.data);
    mqtt.disconnect();
    Serial.println("MQTT ok");
    return true;
  }
  Serial.println("MQTT error");
  return false;
}
bool mqttPublish(String topic, float data) {
  if (config.mqtt_path.length()) topic = config.mqtt_path + "/" + topic;
  return mqtt.publish(topic.c_str(), String(data).c_str(), true);
}
/*
   Функция отвечает за авторизацию пользователя с использованием cookie
   Set-Cookie: NAME=VALUE; expires=DATE; path=PATH; domain=DOMAIN_NAME; secure
   https://ru.wikipedia.org/wiki/HTTP_cookie
*/
bool authorized() {
  // Если есть подозрения на взлом, отбиваем все попытки авторизации
  if (timerAuthorizationBan != 0 and countAuthorizationFail > 5) return false;
  // Переменные для идентификации пользователя
  String clientIP = webServer.client().remoteIP().toString();
  String userAgent;
  /*
     String origin = webServer.header("Origin");
     webServer.sendHeader("Access-Control-Allow-Origin", origin);
     webServer.sendHeader("Access-Control-Allow-Credentials", "true");
  */
  // ищем id браузера клиента
  if (webServer.hasHeader("User-Agent")) userAgent = webServer.header("User-Agent");
  else return false;
  // ищем cookies
  if (webServer.hasHeader("Cookie")) {
    String cookie = webServer.header("Cookie");
    if (cookie.length() > cookiesName.length() + 33) return false;
    if (cookie.indexOf(cookiesName + "=" + cookiesKey) != -1) return true;
    else {
      // Неоднократное срабатывание говорит о попытке взлома путем подбора ключа cookies
      countAuthorizationFail += 1;
      timerAuthorizationBan = millis();
      // Пытаемся удалить cookie у пользователя
      webServer.sendHeader("Set-Cookie", cookiesName + "=0; Path=/; Expires=Thu, 02 Apr 1987 00:00:01 GMT");
      return false;
    }
  }
  // обрабатываем форму авторизации
  if (webServer.hasArg("login") and webServer.hasArg("password")) {
    if (webServer.arg("login") == config.admin_login and webServer.arg("password") == config.admin_pass) {
      cookiesKey = md5(clientIP + salt(32) + userAgent);
      webServer.sendHeader("Set-Cookie", cookiesName + "=" + cookiesKey + "; Path=/");
      return true;
    }
    else {
      // Неоднократное срабатывание говорит о брутфорсе
      countAuthorizationFail += 1;
      timerAuthorizationBan = millis();
    }
  }
  // шьем бороду
  return false;
}
/*
   Функция считывающая показания сенсоров через установленный интервал времени
*/
void readSensors() {
  timerReadSensors = millis();

  float newhumidity = 0;
  float newtemperature = 0;
  //  if (light.status) light.data = BH1750.readLightLevel();
  //  if (temperature.status) temperature.data = dht.readTemperature();
  //  if (humidity.status) humidity.data = SI7021.getHumidityPercent();
  if (humidity.status) newhumidity = dht.readHumidity();
  if (temperature.status) newtemperature = dht.readTemperature();

  if (newtemperature <= 99 && newtemperature >= -99) temperature.data = newtemperature;

  if (newhumidity <= 100 && newhumidity >= 0) humidity.data = newhumidity;
  if (humidity.data > 100) humidity.data = 100;
  if (humidity.data < 0) humidity.data = 0;


  if (pressure.status) pressure.data = BMP085.readPressure() / 133.3;

  if (chs_s)
  {
    temperatureExt.data = temext / 10.0;
    humidityExt.data = humext;
    batteryExt.data = batext;
  }

  //  carbdiox.data = 400 + random(0, 2000);
  /*
    if (millis() > 60000) co2init1 = 1;
    if (co2init1 && co2init2)
      {
        co2Serial.begin(9600);
        co2init2 = 0;
        Serial.println("Starting MH-Z19.");
      }
  */
  if (millis() > 60000) co2ready = 1;
  if (co2ready)
  {
    Serial.println("reading data MH-Z19:");
    int ppm = readCO2();
    Serial.println("  PPM = " + String(ppm));
    if (ppm < 100 || ppm > 6000)
    {
      Serial.println("PPM not valid");
      //   lcd.setCursor(16, 2);
      //   lcd.print("-");
    }
    else
    {
      carbdiox.data = ppm;
      //   lcd.setCursor(16, 2);
      //   lcd.print("+");
    }
  }

  lcd.setCursor(0, 1);
  lcd.print("                 ");
  lcd.setCursor(0, 1);
  lcd.print(humidity.data, 0);
  lcd.print("%");
  if (humidity.data < 10) lcd.print("  ");
  else if (humidity.data < 100) lcd.print(" ");
  lcd.setCursor(5, 1);
  lcd.print(temperature.data, 1);
  //    lcd.print("C ");
  lcd.print("\xDF");
  lcd.setCursor(11, 1);
  lcd.print(pressure.data);


  lcd.setCursor(11, 2);
  if (carbdiox.data < 1000) lcd.print(" ");
  lcd.print(carbdiox.data, 0);
  /*
    light.data = random(12000, 15000);
    temperature.data = random(25, 26) / 0.01;
    humidity.data = random(51, 52);
    pressure.data = random(758, 759) * 133.3;
  */

  if (((millis() / 3600000) - stat[ 1 ][ 17 ]) >= 1.0 )
    //заполняем текущее значение раз 1 час (3600000)
  {
    //сдвигаем массив влево, чтобы освободить предпоследнюю ячейку
    int i = 0 ;
    for (i = 0 ; i < 17 ; i++) stat[ 0 ][i] = stat[ 0 ][i + 1 ];
    for (i = 0 ; i < 17 ; i++) stat[ 1 ][i] = stat[ 1 ][i + 1 ];
    //dps.getPressure(&Pressure); //Записываем значения давления
    //stat[ 0 ][ 11 ] = pressure.data/ 13.33 ;
    stat[ 0 ][ 17 ] = pressure.data * 10;
    stat[ 1 ][ 17 ] = millis() / 3600000;
    grafik( 0 , 1 , 0 );
  }
}
/*строим график,
  вызов усложнен для построения
  разных графиков из массива, первое
  число номер массива, затем
  периодичность выборки данных, потом
  начальное значение в массиве. Мы
  берем первый массив(0), берем каждое
  второе значение(2) и начинаем с начала
  (0)*/
/*Мы будем строить относительные
  график, поэтому рассчитаем интервал
  в который укладываются значения. Мы
  можем использовать только восемь
  строчек одной ячейки дисплея,
  поэтому разобьем интервал на 8
  промежутков.*/



int interval( int x) {
  int maxy = -5000 ;
  int inty = 0 ;
  int minx = minimum(x);
  for ( int k = 0 ; k <= 17 ; k++) {
    if (stat[ 1 ][k] != 0 ) { //если значение не ноль
      if (stat[x][k] > maxy) maxy = stat[x][k]; //считаем максимальное значение
    }
  }
  if (maxy == -5000 ) maxy = 0 ;
  //Строим интервал
  inty = maxy - minx;
  int intx = inty / 8 ;

  lcd.setCursor(18, 2);
  int intyz = round(inty / 10.0);
  // int intyz = inty / 10;
  // if ((inty % 10) >= 5) intyz +=1;

  if (intyz < 1) lcd.print("<1");
  else if (intyz < 10)
  {
    lcd.print(" ");
    lcd.print(intyz);
  }
  else lcd.print(intyz);

  /*  lcd.setCursor(17, 2);
    if (inty < 10) lcd.print("  ");
    else if (inty < 100) lcd.print(" ");
    lcd.print(inty);
  */

  return intx;
}
/*При расчете интервала нам нужно
  минимальное значение в массиве:*/
int minimum ( int d) {
  int minx = 32767 ;
  for ( int i = 0 ; i <= 17 ; i++) {
    if (stat[ 1 ][i] != 0 ) { //если значение не ноль
      if (stat[d][i] < minx) minx = stat[d][i]; //считаем минимальное значение
    }
  }
  if (minx == 50000 ) minx = 0 ;
  return minx;
}
/*Строим сам график, если значений в
  массиве времени нет, выводим
  прочерк. Вызов функции усложнен для
  построения разных графиков из
  массива. Первое число номер массива,
  затем периодичность выборки данных,
  потом начальное значение в массиве.
  Мы берем первый массив(0), берем
  каждое второе значение(2) и начинаем
  с начала(0):*/
void grafik( int x, int y, int z) {
  int intx = interval(x); //вызовем функцию расчета интервала графика
  int minx = minimum(x); //Уточним минимальное значение
  lcd.setCursor( 0 , 3 );
  lcd.print( "P:" ); //Подпись на дисплее

  for ( int i = z; i <= 17 ; i = i + y) {
    if (stat[x][i] == 0 )  { //проверяем есть ли данные о времени
      lcd.print( "-" ); //если значений нет, отображаем прочерк
    } else if (stat[x][i] > (minx + intx * 7 )) {
      lcd.printByte( 0 );
    } else if (stat[x][i] > (minx + intx * 6 )) {
      lcd.printByte( 1 );
    } else if (stat[x][i] > (minx + intx * 5 )) {
      lcd.printByte( 2 );
    } else if (stat[x][i] > (minx + intx * 4 )) {
      lcd.printByte( 3 );
    } else if (stat[x][i] > (minx + intx * 3 )) {
      lcd.printByte( 4 );
    } else if (stat[x][i] > (minx + intx * 2 )) {
      lcd.printByte( 5 );
    } else if (stat[x][i] > (minx + intx)) {
      lcd.printByte( 6 );
    } else {
      lcd.printByte( 7 );
    }
  }
}


/*
   HTTP API
   Ответ на запрос показаний датчиков
*/
void web_api_sensors() {
  //Serial.print(webServer.client().remoteIP().toString() + ": ");
  //Serial.print("handleFileRead: /api/sensors");
  String answer;

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& sensor = jsonBuffer.createObject();

  //sensor["light"] = light.data;
  sensor["temperature"] = temperature.data;
  sensor["humidity"] = humidity.data;
  sensor["pressure"] = pressure.data;
  sensor["temperatureExt"] = temperatureExt.data;
  sensor["humidityExt"] = humidityExt.data;
  sensor["carbdiox"] = carbdiox.data;
  sensor["batteryExt"] = batteryExt.data;

  sensor.printTo(answer);
  webServer.sendHeader("Cache-Control", "no-store, no-cache");
  webServer.send(200, "text/plain", answer);
}
/*
  Функция добавляет новую запись в журнал и помечает следующую позицию для перезаписи
*/
void sensorLogUpdate(struct sensor * sensor, String name = "") {
  unsigned int logSize = (sizeof(sensor->log) / sizeof(float *)) - 1;
  if (name.length()) Serial.print("LU " + String(sensor->logPosition) + "x" + String(logSize));
  sensor->log[sensor->logPosition++] = sensor->data;
  if (sensor->logPosition > logSize) sensor->logPosition = 0;
  if (name.length()) {
    Serial.print("\n" + name + ": ");
    for (int i = 0; i <= logSize; i++) Serial.print(" [" + String(i) + "]" + sensorDelFloatNull(sensor->log[i]));
    Serial.println();
  }
}
/*
  Функция инициализирует добавление новых записей в журналы датчиков
*/
void sensorLogWrite() {
  timerLogWrite = millis();

  sensorLogUpdate(&carbdiox/*, "carbdiox"*/);
  sensorLogUpdate(&temperature/*, "temperature"*/);
  sensorLogUpdate(&humidity/*, "humidity"*/);
  sensorLogUpdate(&pressure/*, "pressure"*/);
  sensorLogUpdate(&temperatureExt/*, "temperatureExt"*/);
  sensorLogUpdate(&humidityExt/*, "humidityExt"*/);
  sensorLogUpdate(&batteryExt/*, "batteryExt"*/);
}
/*
  Функция формирует правильную последовательность показаний с учётом текущего смещения 0 отметки в журнале
*/
String sensorLogRead(struct sensor * sensor) {
  String log;
  unsigned int logSize = (sizeof(sensor->log) / sizeof(float *)) - 1;
  for (int i = sensor->logPosition; i <= logSize; i++) log += (log.length() ? "," : "") + sensorDelFloatNull(sensor->log[i]);
  for (int i = 0; i < sensor->logPosition; i++) log += (log.length() ? "," : "") + sensorDelFloatNull(sensor->log[i]);
  return log;
}
/*
  Функция необходима для удаления лишних символов при передаче показаний журнала в web интерфейс
*/
String sensorDelFloatNull(float data) {
  if (data == 0) return "0";
  if (data - (int)data == 0) return String((int)data);
  return String(data);
}
/*
  HTTP_API
  Журнал показаний за последние сутки
*/
void web_api_sensors_log() {
  String json;
  unsigned long timeAdjustment = (millis() < timerLogWrite) ? 0 : millis() - timerLogWrite;

  json += "\"timeAdjustment\":" + String(timeAdjustment) + ",";
  json += "\"carbdiox\":[" + sensorLogRead(&carbdiox) + "],";
  json += "\"temperature\":[" + sensorLogRead(&temperature) + "],";
  json += "\"humidity\":[" + sensorLogRead(&humidity) + "],";
  json += "\"pressure\":[" + sensorLogRead(&pressure) + "],";
  json += "\"temperatureExt\":[" + sensorLogRead(&temperatureExt) + "],";
  json += "\"humidityExt\":[" + sensorLogRead(&humidityExt) + "],";
  json += "\"batteryExt\":[" + sensorLogRead(&batteryExt) + "]";

  webServer.sendHeader("Cache-Control", "no-store, no-cache");
  webServer.send(200, "text/plain", "{" + json + "}");
}
/*
   HTTP API
   Доступ к настройкам только для автооризованных пользователей
*/
void web_api_settings() {
  webServer.sendHeader("Cache-Control", "no-store, no-cache");
  if (!authorized()) {
    webServer.send(200, "text/html", "{\"status\":false}");
    return;
  }

  if (webServer.hasArg("save")) {
    if (!config.save(webServer.arg("save"))) {
      webServer.send(200, "text/html", "{\"status\":false}");
      return;
    }
  }

  webServer.send(200, "text/html", config.jsonSecurityLoad());
}
/*
   HTTP API
   Загрузка файлов Web сервера по http протоколу
   HTTPMethod
*/
void web_api_spiffs_upload() {
  if (!authorized()) webServer.send(404, "text/plain", "404 Not Found");
}
/*
   HTTP API
   Загрузка файлов Web сервера по http протоколу
   THandlerFunction
*/
void web_api_spiffs_upload_header() {
  hangTestReset();
  HTTPUpload& upload = webServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    if (!authorized()) {
      webServer.send(404, "text/plain", "404 Not Found");
      return;
    }
    WiFiUDP::stopAll();
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    if (filename == config.file) {  // <- НЕ УДАЛЯТЬ! DO NOT DELETE! 请勿卸下！ENTFERNEN SIE NICHT! NE PAS RETIRER!
      webServer.send(404, "text/plain", "404 Not Found");
      return;
    }
    Serial.print("handleFileUpload Name: "); Serial.println(filename);
    spiffsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (spiffsUploadFile) spiffsUploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (spiffsUploadFile) {
      spiffsUploadFile.close();
      Serial.print("handleFileUpload Size: "); Serial.println(formatBytes(upload.totalSize));
      webServer.send(200, "text/plain", "{\"status\":true}");
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (spiffsUploadFile) {
      spiffsUploadFile.close();
      Serial.print("handleFileUpload: aborted");
      webServer.send(200, "text/plain", "{\"status\":false}");
    }
  }
  /*
    String tmpFileName = "/tmp";
    hangTestReset();

    HTTPUpload& upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
    if (!authorized()) {
      webServer.send(404, "text/plain", "404 Not Found");
      return;
    }
    WiFiUDP::stopAll();
    spiffsUploadFileName = upload.filename;
    if (!spiffsUploadFileName.startsWith("/")) spiffsUploadFileName = "/" + spiffsUploadFileName;
    Serial.print("handleFileUpload Name: "); Serial.println(spiffsUploadFileName);
    spiffsUploadFile = SPIFFS.open(tmpFileName, "w");

    } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (spiffsUploadFile) spiffsUploadFile.write(upload.buf, upload.currentSize);

    } else if (upload.status == UPLOAD_FILE_END) {
    if (spiffsUploadFile) spiffsUploadFile.close();
    if (SPIFFS.exists(spiffsUploadFileName)) SPIFFS.remove(spiffsUploadFileName);
    SPIFFS.rename(tmpFileName, spiffsUploadFileName);
    SPIFFS.remove(tmpFileName);
    Serial.print("handleFileUpload Size: "); Serial.println(formatBytes(upload.totalSize));
    webServer.send(200, "text/plain", "{\"status\":true}");
    spiffsUploadFileName = String("");

    } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (spiffsUploadFile) spiffsUploadFile.close();
    if (SPIFFS.exists(tmpFileName)) SPIFFS.remove(tmpFileName);
    Serial.print("handleFileUpload: aborted");
    webServer.send(200, "text/plain", "{\"status\":false}");
    spiffsUploadFileName = String("");
    }
  */
}
/*
   HTTP API
   Получение списка файлов на Flash памяти
*/
void web_api_spiffs_list() {
  if (!authorized()) {
    webServer.send(404, "text/plain", "404 Not Found");
    return;
  }

  String output = "";
  int count = 0;
  uint32_t realSize = ESP.getFlashChipRealSize();

  Dir dir = SPIFFS.openDir("/");
  while (dir.next()) {
    String fileName = dir.fileName().substring(1);
    size_t fileSize = dir.fileSize();
    if ("/" + fileName != config.file) {
      if (++count > 1) output += ",";
      output += "{\"name\":\"" + fileName + "\",\"size\":" + String(fileSize) + "}";
    }
  }
  webServer.send(200, "text/plain", "{\"count\":" + String(count) + ",\"flashSize\":" + String(realSize) + ",\"list\":[" + output + "]}");
}
/*
   HTTP API
   Удаление файла с Flash памяти
*/
void web_api_spiffs_delete() {
  if (!authorized()) {
    webServer.send(404, "text/plain", "404 Not Found");
    return;
  }
  if (!webServer.hasArg("path")) {
    webServer.send(500, "text/plain", "Bad arguments");
    return;
  }

  String path = webServer.arg("path");
  if (!path.startsWith("/")) path = "/" + path;
  if (path.endsWith("/")) path += "index.htm";
  if (path == config.file) {  // <- НЕ УДАЛЯТЬ! DO NOT DELETE! 请勿卸下！ENTFERNEN SIE NICHT! NE PAS RETIRER!
    webServer.send(500, "text/plain", "Bad arguments");
    return;
  }
  if (SPIFFS.exists(path)) {
    SPIFFS.remove(path);
    webServer.send(200, "text/plain", "{\"status\":true}");
  } else webServer.send(200, "text/plain", "{\"status\":false}");
  Serial.println("handleFileDelete: " + path);
}
/*
   HTTP API
   Получение основной информации о микроконтроллере и его программе
*/
void web_api_system_info() {
  if (!authorized()) {
    webServer.send(404, "text/plain", "404 Not Found");
    return;
  }
  String answer;
  answer += "\"mac\":\"" + String(WiFi.macAddress()) + "\",";                // uint8_t (array[6])
  answer += "\"vcc\":\"" + String(ESP.getVcc() * 0.001) + " V\",";           // uint16_t
  answer += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";               // uint32_t
  answer += "\"chipId\":" + String(ESP.getChipId()) + ",";                   // uint32_t
  answer += "\"sdkVersion\":\"" + String(ESP.getSdkVersion()) + "\",";       // const char *
  answer += "\"coreVersion\":\"" + ESP.getCoreVersion() + "\",";             //
  answer += "\"cpuFreqMHz\":\"" + String(ESP.getCpuFreqMHz()) + " MHz\",";   // uint8_t

  answer += "\"flashRealSize\":" + String(ESP.getFlashChipRealSize()) + ","; // uint32_t
  answer += "\"flashChipId\":" + String(ESP.getFlashChipId()) + ",";         // uint32_t
  answer += "\"flashChipSpeed\":\"" + String(int(ESP.getFlashChipSpeed() * 0.000001)) + " MHz\","; // uint32_t

  answer += "\"sketchSize\":" + String(ESP.getSketchSize()) + ",";           // uint32_t
  answer += "\"sketchMD5\":\"" + ESP.getSketchMD5() + "\",";                 //
  answer += "\"freeSketchSpace\":" + String(ESP.getFreeSketchSpace()) + ","; // uint32_t

  answer += "\"resetReason\":\"" + ESP.getResetReason() + "\",";             //
  answer += "\"resetInfo\":\"" + ESP.getResetInfo() + "\",";                 //

  answer += "\"bootVersion\":\"" + String(ESP.getBootVersion()) + "\"";      // uint8_t
  //answer += "\"bootMode\":\"" + String(ESP.getBootMode()) + "\"";          // uint8_t

  webServer.send(200, "text/plain", "{" + answer + "}");
}
/*
   HTTP API
   Перезагрузка
*/
void web_api_system_reboot() {
  if (!authorized()) {
    webServer.send(404, "text/plain", "404 Not Found");
    return;
  }
  status.current = 5;
  webServer.send(200, "text/plain", "{\"status\":true}");
  ESP.restart();
}
/*
  HTTP API
  Сброс настроек устройства
*/
void web_api_system_hardReset() {
  if (!authorized()) {
    webServer.send(404, "text/plain", "404 Not Found");
    return;
  }
  status.current = 5;
  webServer.send(200, "text/plain", "{\"status\":true}");
  config.remove();
  WiFi.mode(WIFI_OFF);
  delay(10000);
  ESP.restart();
}
/*
   HTTP API
   Сканер i2c шины
   http://playground.arduino.cc/Main/I2cScanner
*/
void web_api_system_i2c_scaner() {
  if (!authorized()) {
    webServer.send(404, "text/plain", "404 Not Found");
    return;
  }
  byte error, address, count = 0;
  String answer;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0 or error == 4) {
      answer += answer.length() ? ",\"" : "{\"";
      if (address < 16) answer += '0';
      answer += String(address, HEX) + "\":" + (!error ? '1' : '0');
      count++;
    }
  }
  if (answer.length()) answer += "}";
  webServer.send(200, "text/plain", "{\"count\":" + String(count) + ", \"list\":[" + answer + "]}");
}
/*
   HTTP
   error 404
*/
void web_error_404() {
  if (!fs_handle_file(webServer.uri())) webServer.send(404, "text/plain", "404 Not Found");
}
/*
   File system object (SPIFFS)
   Обработчик запросов на доступ к реально существующим файлам
*/
bool fs_handle_file(String path) {
  webServer.sendHeader("Server", "ESP8266");
  webServer.sendHeader("Cache-Control", "no-transform, private, max-age=86400, s-maxage=86400");
  // [!] Доступ к конфигу только через API [!]
  if (path == config.file) return false; // <- НЕ УДАЛЯТЬ! DO NOT DELETE! 请勿卸下！ENTFERNEN SIE NICHT! NE PAS RETIRER!
  if (path.endsWith("/")) path += "index.htm";
  Serial.print(webServer.client().remoteIP().toString() + ": " + path);
  String fileType = fs_getContentType(path);
  if (webServer.hasHeader("Accept-Encoding")) {
    String encoding = webServer.header("Accept-Encoding");
    if (encoding.indexOf("gzip") != -1 and SPIFFS.exists(path + ".gz")) path += ".gz";
  }
  if (SPIFFS.exists(path)) {
    File file = SPIFFS.open(path, "r");
    size_t fSize = file.size();
    // [!] Подобие системы кеширвоания [!]
    // [!] Необходимо быть уверенным, что размер файла после модификации изменился [!]
    webServer.sendHeader("ETag", String(fSize));
    if (webServer.hasHeader("If-None-Match")) {
      String Etag = webServer.header("If-None-Match");
      if (Etag == String(fSize)) {
        webServer.send(304, "text/plain", "Not Modified");
        file.close();
        Serial.println(" 304, Etag " + Etag);
        return true;
      }
    }
    Serial.println(" 200");
    webServer.streamFile(file, fileType);
    file.close();
    return true;
  }
  Serial.println(" 404");
  return false;
}
/*
   File system object (SPIFFS)
   Функция пересчета размера файлов
*/
String formatBytes(size_t bytes) {
  if (bytes < 1024) return String(bytes) + "B";
  else if (bytes < pow(1024, 2)) return String(bytes / 1024) + "KB";
  else if (bytes < pow(1024, 3)) return String(bytes / pow(1024, 2)) + "MB";
  else return String(bytes / pow(1024, 3)) + "GB";
}
/*
   File system object (SPIFFS)
   Функция участвует в генерации заголовка при ответе клиенту
*/
String fs_getContentType(String& path) {
  if (path.endsWith(".html"))       return "text/html";
  else if (path.endsWith(".htm"))   return "text/html";
  else if (path.endsWith(".css"))   return "text/css";
  else if (path.endsWith(".txt"))   return "text/plain";
  else if (path.endsWith(".js"))    return "application/javascript";
  else if (path.endsWith(".png"))   return "image/png";
  else if (path.endsWith(".gif"))   return "image/gif";
  else if (path.endsWith(".jpg"))   return "image/jpeg";
  else if (path.endsWith(".ico"))   return "image/x-icon";
  else if (path.endsWith(".svg"))   return "image/svg+xml";
  else if (path.endsWith(".ttf"))   return "application/x-font-ttf";
  else if (path.endsWith(".otf"))   return "application/x-font-opentype";
  else if (path.endsWith(".woff"))  return "application/font-woff";
  else if (path.endsWith(".woff2")) return "application/font-woff2";
  else if (path.endsWith(".eot"))   return "application/vnd.ms-fontobject";
  else if (path.endsWith(".sfnt"))  return "application/font-sfnt";
  else if (path.endsWith(".xml"))   return "text/xml";
  else if (path.endsWith(".pdf"))   return "application/pdf";
  else if (path.endsWith(".zip"))   return "application/zip";
  else if (path.endsWith(".gz"))    return "application/x-gzip";
  return "application/octet-stream";
  //return "text/plain";
}
/*
   Информация о флеш памяти
*/
void flashInfo() {
  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  Serial.printf("Flash real id:   %08X\r\n", ESP.getFlashChipId());
  Serial.printf("Flash real size: %u\r\n", realSize);
  Serial.printf("Flash ide size: %u\r\n", ideSize);
  Serial.printf("Flash ide speed: %u\r\n", ESP.getFlashChipSpeed());
  Serial.printf("Flash ide mode:  %s\r\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));
  Serial.printf("Flash Chip configuration %s\r\n", ideSize != realSize ? "wrong!" : "ok");
  Serial.println();
}
/*
   Генератор соли
*/
String salt(int strlen) {
  String answer;
  char simbol[82] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
    '!', '@', '#', '$', '%', '^', '&', '*', '(', ')', ' ', '-', '=', '~', '`', '?', ',', '.', ':', ';'
  };
  for (byte i = 0; i < strlen - 1; i++) answer += simbol[secureRandom(0, 81)];
  return md5(answer);
}
