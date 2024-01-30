#include <Arduino.h>
#include <Wire.h>
#include <BLEServer.h>
#include <BLEDevice.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "SPIFFS.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <EEPROM.h>
#include <BLE2902.h>
#include "esp_sleep.h"
#include <TickTwo.h>
#include <OneButton.h>
#include <ElegantOTA.h>
#include <Arduino_JSON.h>

String getOutputStates();
void notifyClients(String state);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void initWebSocket();
void initSPIFFS();
String readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
bool initWiFi();
String processor(const String &var);
void alapparameter();
void allrelayoff();
void setDelay(String d, String index, uint32_t &delay, uint16_t eepromAddress, uint32_t maxDelay);
void setZone(String d, String index, uint16_t &zone, uint16_t eepromAddress, uint16_t minZone, uint16_t maxZone);
void setVent(String d, String index, int relayIndex);
void recvMsg(uint8_t *dataWebserial, size_t len);
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);
void atlagolas();
void ventillatorvezerles();
void relek();
void kiirzonak();
void kiirkesleltetesek();
void kiiras();
void kiir(String szoveg, uint16_t ertek);
void onConnect(BLEClient *pclient);
void onDisconnect(BLEClient *pclient);
bool connectToServer(BLEUUID serviceUUID, BLEUUID charUUID);
void onResult(BLEAdvertisedDevice advertisedDevice);
void onConnect(BLEServer *pServer);
void onDisconnect(BLEServer *pServer);
void eeprom_check();
void eeprom_commit();
void checkAndReset(int value, uint32_t defaultValue, uint16_t address, const char *name);
void eeprom_valid();
void readeepromparameter();
void blekliens();
void bleszerver(BLEUUID serviceUUID, BLEUUID charUUID);
void fct_Watchdog();
void fct_goToSleep();
void fct_ledUpdate();
void fct_WatchdogReset();
void click();
void doubleClick();
void LongPressStop();
void fct_OnStateLed();
void fct_OnStateLedWithWifi();
void fct_OnStateLedBleConnected();
void fct_counterFromBoot();
void loop2(void *pvParameter);
void ledPwmBlinking(int blinkNumber);
void rebootEsp();
void onOTAStart();
void onOTAProgress(size_t current, size_t final);
void onOTAEnd(bool success);
void print_wakeup_reason();
void wifiOn();
void wifiInterface();

TickTwo watchDOG(fct_Watchdog, 1000, 0, MILLIS);
TickTwo counterFromBoot(fct_counterFromBoot, 1000, 0, MILLIS);
TickTwo ledUpdate(fct_ledUpdate, 1000, 0, MILLIS);
TickTwo OnStateLed(fct_OnStateLed, 50);
TickTwo OnStateLedWithWifi(fct_OnStateLedWithWifi, 300);
TickTwo OnStateLedBleConnected(fct_OnStateLedBleConnected, 900);
static boolean ledStateOnstateLed;
static boolean ledStateWifistateLed;
static boolean stopServer = false;
static boolean reboot = false;  // reboothoz hogy felébredjen ram-ban tárolom majd
static boolean hutesUzemmod = false;
static boolean kalibralas = false;
static boolean teszteles = false;
static uint8_t bootCounter = 0;
RTC_DATA_ATTR int bootCount = 0;
const uint16_t serverEnd = 600;

#define BUTTON_PIN_BITMASK 0x200000000  // 2^33 in hex
// const uint64_t WAKEUP_LOW_PIN_BITMASK = 0b001111;

static uint32_t watchdogCounter = 0;
static uint32_t fromBootCounter = 0;
const uint32_t timetosleep = 1800;      // ennyi ido utan sleep, masodperc
const uint8_t scanTime = 30;            // meddig szkennelje az eszkozoket bekapcsolaskor, 0 akkor folyamatosan
const uint8_t intervallum = 10;         // ennyi atlagat veszi
static uint8_t szamlalo = intervallum;  // a kezdőértéke az intervallum
static uint16_t adattemp;
static uint16_t teljesitmenytemp;
static uint16_t teljesitmeny;
static uint16_t elozoTeljesitmenyzona = 0;
static uint8_t teljesitmenyzona = 0;
static uint16_t adat;

static uint16_t randNumber;

static boolean vent = true;
static boolean reset = false;
static uint32_t erzekelo;
const uint16_t memoria_meret = 200;  // használt memória mérete
static unsigned long time_now;
static unsigned long time_nowrele;
static unsigned long time_elapsedtime = 0;
static unsigned long time_elapsedtimerele = 0;
static unsigned long time_elapsedtime_elozoTeljesitmenyzona = 0;
static unsigned long now;
static unsigned long elapsedtime = 0;
const uint16_t period = 1000;  // a loop masodpercenkent egyszer fut
static uint32_t periodrele = 0;
static boolean releteszt = false;
static unsigned long previousMillis1 = 0;
static unsigned long previousMillis2 = 0;
static unsigned long time_now_click;
static unsigned long startMillis;        // LED_wifi pwm írásának utolsó idelye
const uint16_t ledChanelInterval = 125;  // LED_wifi pwm periódusa
static uint32_t kesleltetesend;
static uint32_t kesleltetes0;
static uint32_t kesleltetes1;
static uint32_t kesleltetes2;
static uint32_t kesleltetes3;
static uint32_t kesleltetessprint;
const char welcome[] = "Hello! Ezen az IP cimen az ESP32 ventillator vezerlo mukodik!. Webserial: IP/webserial. ";
const char help1[] = "Parancsok: help, reset, wifireset, off, lcdon, lcdoff, run, teszt, reboot, venton, ventoff, vent1on, vent1off, vent2on, vent2off, vent3on, vent3off, mymaxheartrate, myftp, erzekeloheart, erzekelopower, milyenerzekelo, sprintzona(watt/bpm), alapteljesitmeny(watt/bpm), elsozona(watt/bpm), masodikzona(watt/bpm), zonak? ";
const char help2[] = ",kesleltetesnulla(masodperc), kesleltetesegy(masodperc), kesleltetesketto(masodperc), kesleltetesharom(masodperc), kesleltetessprint(masodperc), kesleltetesend(masodperc), kesleltetesek?, hutesuzemmodbe, hutesuzemmodki, kalibralasbe, kalibralaski ";
String inString = "";

byte delta[8] = {  // LCD-hez, delta jel
  0b00100,
  0b00100,
  0b01010,
  0b01010,
  0b11011,
  0b10001,
  0b10001,
  0b11111
};

// Set number of outputs
#define NUM_OUTPUTS 4
// Assign each GPIO to an output
const uint8_t outputGPIOs[NUM_OUTPUTS] = { 10, 9, 8, 2 };

#define RELAY_NO true
#define NUM_RELAYS 3                                  // hany darab rele
const uint8_t relayGPIOs[NUM_RELAYS] = { 10, 9, 8 };  // relekimenetek
#define relayOutlet 2                                 // kulso aljzat
#define relayEN 21
static uint16_t ZONE_1;  // ventillator 2. fokozat
static uint16_t ZONE_2;  // ventillator 3. fokozat
static uint16_t sprintzona;
static uint16_t alapteljesitmeny;

BLEUUID serviceerzekeloUUID;
BLEUUID charerzekeloUUID;
#define SERVICE1_UUID (BLEUUID((uint16_t)0x180D))  // heart
#define CHARACTERISTIC1_UUID (BLEUUID((uint16_t)0x2A37))
#define SERVICE2_UUID (BLEUUID((uint16_t)0x1818))  // power
#define CHARACTERISTIC2_UUID (BLEUUID((uint16_t)0x2A63))

BLEDescriptor *pDescr;
BLE2902 *pBLE2902;
// BLECharacteristic* pCharacteristic;
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;

static boolean doConnect = false;
static boolean connected = false;
static boolean notification = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define LED_wifi 5    // wifi hiba
#define LED_eeprom 4  // eeprom hiba
static uint16_t dutyCycleLed;
// setting PWM properties
const uint32_t freq = 5000;
const uint8_t ledChannel = 0;
const uint8_t resolution = 8;
#define WAKEUP_PIN 3  // felebreszteshez
OneButton button(WAKEUP_PIN, true);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

static unsigned long ota_progress_millis = 0;

// Search for parameter in HTTP POST request
const char *PARAM_INPUT_1 = "ssid";
const char *PARAM_INPUT_2 = "pass";
const char *PARAM_INPUT_3 = "ip";
const char *PARAM_INPUT_4 = "gateway";

static boolean wifiOk = false;

// Variables to save values from HTML form
String ssid;
String pass;
String ip;
String gateway;

// File paths to save input values permanently
const char *ssidPath = "/ssid.txt";
const char *passPath = "/pass.txt";
const char *ipPath = "/ip.txt";
const char *gatewayPath = "/gateway.txt";

IPAddress localIP;
// Set your Gateway IP address
IPAddress localGateway;
IPAddress subnet(255, 255, 0, 0);

// Timer variables
static unsigned long previousMillis = 0;
const long interval = 60000;  // interval to wait for Wi-Fi connection (milliseconds)

// Set LED GPIO
const int ledPin = 4;
// Stores LED state

String ledState;

String getOutputStates() {
  JSONVar myArray;
  for (int i = 0; i < NUM_OUTPUTS; i++) {
    myArray["gpios"][i]["output"] = String(outputGPIOs[i]);
    myArray["gpios"][i]["state"] = String(digitalRead(outputGPIOs[i]));
  }
  String jsonString = JSON.stringify(myArray);
  return jsonString;
}

void notifyClients(String state) {
  ws.textAll(state);
}

void onOTAStart() {
  // Log when OTA has started
  if (stopServer == false)
    Serial.println("OTA update started!");
  // <Add your own code here>
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    if (strcmp((char *)data, "states") == 0) {
      notifyClients(getOutputStates());
    } else {
      int gpio = atoi((char *)data);
      if (gpio == outputGPIOs[0]) {
        digitalWrite(gpio, !digitalRead(gpio));
        if (digitalRead(gpio)) {
          releteszt = false;
        } else {
          releteszt = true;
        }
        if (!digitalRead(gpio)) {
          digitalWrite(outputGPIOs[1], HIGH);
          digitalWrite(outputGPIOs[2], HIGH);
        }
      }
      if (gpio == outputGPIOs[1]) {
        digitalWrite(gpio, !digitalRead(gpio));
        if (digitalRead(gpio)) {
          releteszt = false;
        } else {
          releteszt = true;
        }
        if (!digitalRead(gpio)) {
          digitalWrite(outputGPIOs[0], HIGH);
          digitalWrite(outputGPIOs[2], HIGH);
        }
      }
      if (gpio == outputGPIOs[2]) {
        digitalWrite(gpio, !digitalRead(gpio));
        if (digitalRead(gpio)) {
          releteszt = false;
        } else {
          releteszt = true;
        }
        if (!digitalRead(gpio)) {
          digitalWrite(outputGPIOs[0], HIGH);
          digitalWrite(outputGPIOs[1], HIGH);
        }
      }
      if (gpio == outputGPIOs[3]) {
        if (!connected) {
          digitalWrite(gpio, !digitalRead(gpio));
        }
      }
      notifyClients(getOutputStates());
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      if (stopServer == false)
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      if (stopServer == false)
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    if (stopServer == false)
      Serial.println("An error has occurred while mounting SPIFFS");
  }
  if (stopServer == false)
    Serial.println("SPIFFS mounted successfully");
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    if (stopServer == false)
      Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    if (stopServer == false)
      Serial.println("OTA update finished successfully!");
  } else {
    if (stopServer == false)
      Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
  unsigned long timeNowonOTAEnd = millis();
  while (millis() < timeNowonOTAEnd + 2000) {
  }
  rebootEsp();
}

// Read File from SPIFFS
String readFile(fs::FS &fs, const char *path) {
  if (stopServer == false)
    Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    if (stopServer == false)
      Serial.println("- failed to open file for reading");
    return String();
  }

  String fileContent;
  while (file.available()) {
    fileContent = file.readStringUntil('\n');
    break;
  }
  return fileContent;
}

// Write file to SPIFFS
void writeFile(fs::FS &fs, const char *path, const char *message) {
  if (stopServer == false)
    Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    if (stopServer == false)
      Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    if (stopServer == false)
      Serial.println("- file written");
  } else {
    if (stopServer == false)
      Serial.println("- write failed");
  }
}

// Initialize WiFi
bool initWiFi() {
  if (ssid == "" || ip == "") {
    if (stopServer == false)
      Serial.println("Undefined SSID or IP address.");
    return false;
  }

  WiFi.mode(WIFI_STA);
  localIP.fromString(ip.c_str());
  localGateway.fromString(gateway.c_str());

  if (!WiFi.config(localIP, localGateway, subnet)) {
    if (stopServer == false)
      Serial.println("STA Failed to configure");
    return false;
  }
  WiFi.begin(ssid.c_str(), pass.c_str());
  if (stopServer == false)
    Serial.println("Connecting to WiFi...");

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  while (WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      if (stopServer == false)
        Serial.println("Failed to connect.");
      return false;
    }
  }

  if (stopServer == false)
    Serial.println(WiFi.localIP());
  return true;
}

// Replaces placeholder with LED state value
String processor(const String &var) {
  if (var == "STATE") {
    if (digitalRead(ledPin)) {
      ledState = "ON";
    } else {
      ledState = "OFF";
    }
    return ledState;
  }
  return String();
}

void alapparameter() {
  periodrele = 0;
  connected = false;
  notification = false;
  doConnect = false;
  szamlalo = intervallum;
  teljesitmeny = 0;
  teljesitmenytemp = 0;
  elozoTeljesitmenyzona = 0;
  adat = 0;
}

void allrelayoff() {
  ledcWrite(ledChannel, 0);
  for (uint8_t b = 1; b <= NUM_RELAYS; b++) {  // minden rele elengedese, a kimenetket magas szintre kell allitani
    pinMode(relayGPIOs[b - 1], OUTPUT);
    if (RELAY_NO) {
      digitalWrite(relayGPIOs[b - 1], HIGH);
    } else {
      digitalWrite(relayGPIOs[b - 1], LOW);
    }
  }
}

void setDelay(String d, String index, uint32_t &delay, uint16_t eepromAddress, uint32_t maxDelay) {
  if (d.indexOf(index) >= 0) {
    if (stopServer == false)
      WebSerial.println(inString.toInt());
    if (inString.toInt() != 0 && inString.toInt() > 0 && inString.toInt() < maxDelay) {
      uint32_t i = inString.toInt();
      delay = i * 1000;
      EEPROM.put(eepromAddress, i);
      eeprom_commit();
      if (stopServer == false)
        WebSerial.print("A ");
      if (stopServer == false)
        WebSerial.print(index);
      if (stopServer == false)
        WebSerial.print(": ");
      if (stopServer == false)
        WebSerial.print(delay / 1000);
      if (stopServer == false)
        WebSerial.println(" másodperc értékű lett.");
    } else {
      if (stopServer == false)
        WebSerial.println("Hibas adat!");
      kiirkesleltetesek();
    }
  }
}

void setZone(String d, String index, uint16_t &zone, uint16_t eepromAddress, uint16_t minZone, uint16_t maxZone) {
  if (d.indexOf(index) >= 0) {
    if (stopServer == false)
      WebSerial.println(inString.toInt());
    if (inString.toInt() != 0 && inString.toInt() > minZone && inString.toInt() < maxZone) {
      uint16_t i = inString.toInt();
      zone = i;
      EEPROM.put(eepromAddress, i);
      eeprom_commit();
      kiirzonak();
    } else {
      if (stopServer == false)
        WebSerial.println("Hibas " + index + " adat!");
      kiirzonak();
    }
  }
}

void setVent(String d, String index, int relayIndex) {
  if (d.indexOf(index) >= 0) {
    if (d == index + "on" && vent) {
      releteszt = true;
      teljesitmenyzona = 0;
    }
    for (uint8_t e = 1; e <= NUM_RELAYS; e++) {
      digitalWrite(relayGPIOs[e - 1], HIGH);
    }
    digitalWrite(relayGPIOs[relayIndex], LOW);
  }
  if (d == index + "off") {
    releteszt = false;
    teljesitmenyzona = 0;
    vent = false;
    releteszt = true;
    allrelayoff();
  }
}

void recvMsg(uint8_t *dataWebserial, size_t len) {
  if (stopServer == false)
    WebSerial.println("Received data...");
  String d = "";
  inString = "";
  for (uint8_t i = 0; i < len; i++) {
    d += char(dataWebserial[i]);
    uint16_t incar = char(dataWebserial[i]);
    if (isDigit(incar)) {
      inString += (char)incar;
    }
  }
  if (stopServer == false)
    WebSerial.println(d);

  setDelay(d, "kesleltetesnulla", kesleltetes0, 40, 120000);
  setDelay(d, "kesleltetesegy", kesleltetes1, 50, 120000);
  setDelay(d, "kesleltetesketto", kesleltetes2, 60, 120000);
  setDelay(d, "kesleltetesharom", kesleltetes3, 70, 120000);
  setDelay(d, "kesleltetessprint", kesleltetessprint, 80, 120000);
  setDelay(d, "kesleltetesend", kesleltetesend, 110, 600000);

  setZone(d, "alapteljesitmeny", alapteljesitmeny, 100, 0, ZONE_1);
  setZone(d, "elsozona", ZONE_1, 10, alapteljesitmeny, ZONE_2);
  setZone(d, "masodikzona", ZONE_2, 20, ZONE_1, sprintzona);
  setZone(d, "sprintzona", sprintzona, 30, ZONE_2, 1000);

  setVent(d, "vent1", 0);
  setVent(d, "vent2", 1);
  setVent(d, "vent3", 2);

  if (d == "zonak?") {
    kiirzonak();
  }
  if (d == "kesleltetesek?") {
    kiirkesleltetesek();
  }
  if (d == "ventoff") {
    if (stopServer == false)
      WebSerial.println("Ventillatorok kikapcsolva.");
    vent = false;
    releteszt = false;
    allrelayoff();
  }
  if (d == "venton") {
    if (stopServer == false)
      WebSerial.println("Ventillatorok visszakapcsolva.");
    elozoTeljesitmenyzona = 0;
    periodrele = 0;
    releteszt = false;
    vent = true;
  }
  if (d == "reboot") {
    ledPwmBlinking(3);
    rebootEsp();
  }
  if (d == "teszt") {
    teszteles = true;
    hutesUzemmod = false;
    kalibralas = false;
    bootCounter = 2;
    EEPROM.put(150, kalibralas);
    eeprom_commit();
    EEPROM.put(140, hutesUzemmod);
    eeprom_commit();
    EEPROM.put(90, teszteles);
    eeprom_commit();
    EEPROM.put(160, bootCounter);
    eeprom_commit();
    ledPwmBlinking(3);
    rebootEsp();
  }
  if (d == "run") {
    teszteles = false;
    bootCounter = 0;
    EEPROM.put(90, teszteles);
    eeprom_commit();
    EEPROM.put(160, bootCounter);
    eeprom_commit();
    ledPwmBlinking(3);
    rebootEsp();
  }
  if (d == "reset") {
    reset = true;
    eeprom_check();
    eeprom_valid();
    readeepromparameter();
    reset = false;
    ledPwmBlinking(3);
    rebootEsp();
  }
  if (d == "off") {
    if (stopServer == false)
      Serial.println("Shutting down...");
    esp_deep_sleep_enable_gpio_wakeup(BIT(D1), ESP_GPIO_WAKEUP_GPIO_LOW);  // biztos ami biztos
    esp_deep_sleep_start();
  }
  if (d == "wifireset") {
    wifi_config_t current_conf;
    esp_wifi_get_config((wifi_interface_t)ESP_IF_WIFI_STA, &current_conf);
    memset(current_conf.sta.ssid, 0, sizeof(current_conf.sta.ssid));
    memset(current_conf.sta.password, 0, sizeof(current_conf.sta.password));
    esp_wifi_set_config((wifi_interface_t)ESP_IF_WIFI_STA, &current_conf);
    SPIFFS.remove(ssidPath);
    SPIFFS.remove(passPath);
    SPIFFS.remove(ipPath);
    SPIFFS.remove(gatewayPath);
    ledPwmBlinking(3);
    rebootEsp();
  }
  if (d == "lcdon") {
    // kijelzo kezeles
    if (stopServer == false)
      WebSerial.println("LCD ON!");
  }
  if (d == "lcdoff") {
    // kijelzo kezeles
    if (stopServer == false)
      WebSerial.println("LCD OFF!");
  }
  if (d == "hutesuzemmodbe") {
    hutesUzemmod = true;
    kalibralas = false;
    teszteles = false;
    bootCounter = 2;
    EEPROM.put(90, teszteles);
    eeprom_commit();
    EEPROM.put(150, kalibralas);
    eeprom_commit();
    EEPROM.put(140, hutesUzemmod);
    eeprom_commit();
    EEPROM.put(160, bootCounter);
    eeprom_commit();
    ledPwmBlinking(3);
    rebootEsp();
  }
  if (d == "hutesuzemmodki") {
    hutesUzemmod = false;
    bootCounter = 0;
    EEPROM.put(140, hutesUzemmod);
    eeprom_commit();
    EEPROM.put(160, bootCounter);
    eeprom_commit();
    ledPwmBlinking(3);
    rebootEsp();
  }
  if (d == "kalibralasbe") {
    kalibralas = true;
    hutesUzemmod = false;
    teszteles = false;
    bootCounter = 2;
    EEPROM.put(90, teszteles);
    eeprom_commit();
    EEPROM.put(140, hutesUzemmod);
    eeprom_commit();
    EEPROM.put(150, kalibralas);
    eeprom_commit();
    EEPROM.put(160, bootCounter);
    eeprom_commit();
    ledPwmBlinking(3);
    rebootEsp();
  }
  if (d == "kalibralaski") {
    kalibralas = false;
    bootCounter = 0;
    EEPROM.put(150, kalibralas);
    eeprom_commit();
    EEPROM.put(160, bootCounter);
    eeprom_commit();
    ledPwmBlinking(3);
    rebootEsp();
  }
  if (d == "help") {
    if (stopServer == false)
      WebSerial.println(help1);
    if (stopServer == false)
      WebSerial.println(help2);
  }
  if (d.indexOf("mymaxheartrate") >= 0) {
    if (inString.toInt() != 0 && inString.toInt() > 0 && inString.toInt() < 220) {
      uint16_t mymaxheartrate = inString.toInt();
      ZONE_1 = mymaxheartrate * 0.75;
      ZONE_2 = mymaxheartrate * 0.85;
      sprintzona = mymaxheartrate * 0.96;
      erzekelo = 111;
      alapteljesitmeny = 100;
      EEPROM.put(10, ZONE_1);
      eeprom_commit();
      EEPROM.put(20, ZONE_2);
      eeprom_commit();
      EEPROM.put(30, sprintzona);
      eeprom_commit();
      EEPROM.put(100, alapteljesitmeny);
      eeprom_commit();
      EEPROM.put(120, erzekelo);
      eeprom_commit();
      ledPwmBlinking(3);
      rebootEsp();
    }
  }
  if (d.indexOf("myftp") >= 0) {
    if (inString.toInt() != 0 && inString.toInt() > 0 && inString.toInt() < 600) {
      uint16_t myftpzone = inString.toInt();
      ZONE_1 = myftpzone * 0.75;
      ZONE_2 = myftpzone * 0.90;
      sprintzona = myftpzone * 1.18;
      erzekelo = 222;
      alapteljesitmeny = 20;
      EEPROM.put(10, ZONE_1);
      eeprom_commit();
      EEPROM.put(20, ZONE_2);
      eeprom_commit();
      EEPROM.put(30, sprintzona);
      eeprom_commit();
      EEPROM.put(100, alapteljesitmeny);
      eeprom_commit();
      EEPROM.put(120, erzekelo);
      eeprom_commit();
      ledPwmBlinking(3);
      rebootEsp();
    }
  }
  if (d.indexOf("erzekeloheart") >= 0) {
    erzekelo = 111;
    EEPROM.put(120, erzekelo);
    eeprom_commit();
    ledPwmBlinking(3);
    rebootEsp();
  }
  if (d.indexOf("erzekelopower") >= 0) {
    erzekelo = 222;
    EEPROM.put(120, erzekelo);
    eeprom_commit();
    ledPwmBlinking(3);
    rebootEsp();
  }
  if (d.indexOf("milyenerzekelo") >= 0) {
    if (erzekelo == 111) {
      if (stopServer == false)
        WebSerial.println("Pulzusmero.");
    } else if (erzekelo == 222) {
      if (stopServer == false)
        WebSerial.println("Wattmero.");
    } else if (erzekelo == 0) {
      if (stopServer == false)
        WebSerial.println("Parameter hiba! reset kell!");
      if (stopServer == false)
        WebSerial.println("Ujrainditas!");
      unsigned long time_start_reset = millis();
      while ((millis() - time_start_reset) < 1000) {
        ;
      }
      erzekelo = 222;  // alapbol wattmero lesz
      EEPROM.put(120, erzekelo);
      eeprom_commit();
      ledPwmBlinking(3);
      rebootEsp();
    }
  }
}

static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  ESP_LOG_BUFFER_HEX("my value", pData, length);  // valamiért csak így működik
  unsigned long currentMillis = millis();
  if (deviceConnected) {
    if (currentMillis - previousMillis1 >= 10) {
      previousMillis1 = currentMillis;
      pCharacteristic->setValue(pData, length);
      pCharacteristic->notify();
    }
  } else if (oldDeviceConnected) {
    if (currentMillis - previousMillis2 >= 500) {
      previousMillis2 = currentMillis;
      pServer->startAdvertising();
      oldDeviceConnected = deviceConnected;
    }
  } else {
    oldDeviceConnected = deviceConnected;
  }

  uint16_t data = (pData[3] << 8) | pData[2];  // konvertalas egesz szamma

  if (erzekelo == 111) {
    adattemp = pData[1];  // heart_rate
  } else if (erzekelo == 222) {
    adattemp = data;  // power_watt
  }

  adat = adattemp;  // valamiért csak így működik

  if (currentMillis - startMillis >= ledChanelInterval) {
    dutyCycleLed = adat;
    if (erzekelo == 111) {
      dutyCycleLed = map(dutyCycleLed, 0, (ZONE_2 / 0.85), 1, 255);  // dutyCycleLed nem lehet 255-nél nagyobb
    } else if (erzekelo == 222) {
      dutyCycleLed = map(dutyCycleLed, 0, (ZONE_2 / 0.9), 1, 255);
    }
    dutyCycleLed = min(dutyCycleLed, static_cast<uint16_t>(255));
    ledcWrite(ledChannel, dutyCycleLed);
    startMillis = currentMillis;
  }
}

void atlagolas() {
  if (szamlalo > 0) {
    szamlalo--;
    teljesitmenytemp += adat;
  } else if (szamlalo < 0 || szamlalo > intervallum)  // hiba, nem lehet nullánál kisebb vagy az intervallumnal nagyobb
  {
    szamlalo = intervallum;
  }

  if (szamlalo == 0) {
    szamlalo = intervallum;
    teljesitmeny = teljesitmenytemp / intervallum;
    teljesitmenytemp = 0;
  }
}

void ventillatorvezerles() {

  if (teljesitmeny > 0 && !teszteles) {
    fct_WatchdogReset;
  }

  if (!releteszt && vent) {
    if (teljesitmeny == 0 || teljesitmeny < alapteljesitmeny) {
      teljesitmenyzona = 0;
    } else if (teljesitmeny < ZONE_1 && teljesitmeny > alapteljesitmeny) {  // ne kapcsoljon be ha picit megforgatom a hajtóművet pl. láncolajozás
      teljesitmenyzona = 1;
    } else if (teljesitmeny < ZONE_2 && teljesitmeny >= ZONE_1) {
      teljesitmenyzona = 2;
    } else if (teljesitmeny < sprintzona && teljesitmeny >= ZONE_2) {
      teljesitmenyzona = 3;
    } else if (teljesitmeny >= sprintzona) {
      teljesitmenyzona = 4;
    }

    if (teljesitmenyzona > elozoTeljesitmenyzona) {
      for (uint8_t e = 1; e <= NUM_RELAYS; e++) {
        pinMode(relayGPIOs[e - 1], OUTPUT);
        digitalWrite(relayGPIOs[e - 1], HIGH);
      }
      switch (teljesitmenyzona) {
        case 1:
          digitalWrite(relayGPIOs[0], LOW);
          periodrele = kesleltetes1;
          time_elapsedtimerele = millis();
          break;
        case 2:
          digitalWrite(relayGPIOs[1], LOW);
          periodrele = kesleltetes2;
          time_elapsedtimerele = millis();
          break;
        case 3:
          digitalWrite(relayGPIOs[2], LOW);
          periodrele = kesleltetes3;
          time_elapsedtimerele = millis();
          break;
        case 4:
          digitalWrite(relayGPIOs[2], LOW);
          periodrele = kesleltetessprint;
          time_elapsedtimerele = millis();
          break;
        default:
          digitalWrite(relayGPIOs[0], LOW);
          periodrele = kesleltetes0;
          time_elapsedtimerele = millis();
          break;
      }
    }

    if (teljesitmenyzona == 0 && elozoTeljesitmenyzona >= 1) {
      periodrele = kesleltetesend;
      time_elapsedtimerele = millis();
      for (uint8_t i = 1; i <= NUM_RELAYS; i++) {
        pinMode(relayGPIOs[i - 1], OUTPUT);
        digitalWrite(relayGPIOs[i - 1], HIGH);
      }
      digitalWrite(relayGPIOs[0], LOW);
    }

    elozoTeljesitmenyzona = teljesitmenyzona;

    unsigned long timeNowRelekElott = millis();
    while (millis() < timeNowRelekElott + 100) {
    }

    relek();
  }
}

void relek() {
  if (vent && !releteszt) {
    time_nowrele = millis();
    if (time_nowrele - time_elapsedtimerele >= periodrele) {
      time_elapsedtimerele = time_nowrele;
      for (uint8_t e = 1; e <= NUM_RELAYS; e++) {
        pinMode(relayGPIOs[e - 1], OUTPUT);
        digitalWrite(relayGPIOs[e - 1], HIGH);
      }
      switch (teljesitmenyzona) {
        case 1:
          digitalWrite(relayGPIOs[0], LOW);
          periodrele = kesleltetes1;
          break;
        case 2:
          digitalWrite(relayGPIOs[1], LOW);
          periodrele = kesleltetes2;
          break;
        case 3:
          digitalWrite(relayGPIOs[2], LOW);
          periodrele = kesleltetes3;
          break;
        case 4:
          digitalWrite(relayGPIOs[2], LOW);
          periodrele = kesleltetessprint;
          break;
        default:
          periodrele = kesleltetes0;
          break;
      }
    }
  }
}

void kiirzonak() {
  if (stopServer == false) {
    kiir("Alapteljesitmeny: ", alapteljesitmeny);
    kiir(" Zona 1: ", ZONE_1);
    kiir(" Zona 2: ", ZONE_2);
    kiir(" sprintzona: ", sprintzona);
    WebSerial.println(" (watt/bpm) értékűek.");
  }
}

void kiirkesleltetesek() {
  if (stopServer == false) {
    kiir("A kesleltetes0: ", kesleltetes0 / 1000);
    kiir(" kesleltetes1 ", kesleltetes1 / 1000);
    kiir(" kesleltetes2: ", kesleltetes2 / 1000);
    kiir(" kesleltetes3: ", kesleltetes3 / 1000);
    kiir(" kesleltetessprint: ", kesleltetessprint / 1000);
    kiir(" kesleltetesend: ", kesleltetesend / 1000);
    kiir(" 'periodrele' valtozo erteke: ", periodrele / 1000);
    WebSerial.println(" ... másodperc értékűek.");
  }
}

void kiiras() {
  if (stopServer == false) {
    now = millis();
    if (now - elapsedtime >= 2500) {
      elapsedtime = now;
      if (erzekelo == 222) {
        kiir("Atlag teljesitmeny: ", teljesitmeny);
        kiir(" Pillanatnyi teljesitmeny: ", adat);
      }
      if (erzekelo == 111) {
        kiir("Atlag pulzus: ", teljesitmeny);
        kiir(" Pillanatnyi pulzus: ", adat);
      }
      if (releteszt) {
        WebSerial.println("A releteszt aktív!");
      }
      kiir("Watchdog számláló: ", watchdogCounter);
    }
  }
}

void kiir(String szoveg, uint16_t ertek) {
  WebSerial.print(szoveg);
  WebSerial.println(ertek);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {
  }
  void onDisconnect(BLEClient *pclient) {
    alapparameter();
    allrelayoff();
  }
};

bool connectToServer(BLEUUID serviceUUID, BLEUUID charUUID) {
  if (stopServer == false)
    Serial.print("Forming a connection to ");
  if (stopServer == false)
    Serial.println(myDevice->getAddress().toString().c_str());
  BLEClient *pClient = BLEDevice::createClient();
  if (stopServer == false)
    Serial.println(" - Created client");
  pClient->setClientCallbacks(new MyClientCallback());
  pClient->connect(myDevice);
  if (stopServer == false)
    Serial.println(" - Connected to server");
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);

  if (pRemoteService == nullptr) {
    if (stopServer == false)
      Serial.print("Failed to find our service UUID: ");
    if (stopServer == false)
      Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  if (stopServer == false)
    Serial.println(" - Found our service");

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);

  if (pRemoteCharacteristic == nullptr) {
    if (stopServer == false)
      Serial.print("Failed to find our characteristic UUID: ");
    if (stopServer == false)
      Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  if (stopServer == false)
    Serial.println(" - Found our characteristic");
  if (pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    if (stopServer == false)
      Serial.print("The characteristic value was: ");
    if (stopServer == false)
      Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  }

  connected = true;
  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (stopServer == false)
      Serial.print("BLE Advertised Device found: ");
    if (stopServer == false)
      Serial.println(advertisedDevice.toString().c_str());
    bool isHeartService = erzekelo == 111 && advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(SERVICE1_UUID);
    bool isPowerService = erzekelo == 222 && advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(SERVICE2_UUID);
    if (isHeartService || isPowerService) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    BLEDevice::startAdvertising();
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

void eeprom_check() {
  if (EEPROM.read(0) != 0x55 || reset) {
    if (!reset) {  // mivel ha nem en kezdemenyztem akkor gond van
      ledPwmBlinking(4);
    }
    if (stopServer == false)
      Serial.println("EEPROM is not formatted!");
    if (stopServer == false)
      Serial.println("Formatting EEPROM...");
    for (uint8_t i = 0; i < memoria_meret; i++) {
      EEPROM.write(i, 0);
      EEPROM.commit();
    }
    EEPROM.write(0, 0x55);
    EEPROM.write(1, 0xAA);
    EEPROM.commit();
    if (stopServer == false)
      Serial.println("Done!");
  } else {
    if (stopServer == false)
      Serial.println("EEPROM is formatted!");
  }
}

void eeprom_commit() {
  if (EEPROM.commit()) {
    if (stopServer == false)
      Serial.println("EEPROM successfully committed");
  } else {
    if (stopServer == false)
      Serial.println("ERROR! EEPROM commit failed");
  }
}

void checkAndReset(int value, uint32_t defaultValue, uint16_t address, const char *name) {
  uint32_t upperLimit = 600000;
  if (reset || value < 0 || value > upperLimit || ZONE_1 == 0 || ZONE_2 == 0 || sprintzona == 0 || erzekelo == 0 || kesleltetes2 == 0 || kesleltetes3 == 0 || kesleltetessprint == 0 || kesleltetesend == 0)  // szuroprobaszeruen megnez par erteket eeprom.check utan
  {
    value = defaultValue;
    EEPROM.put(address, value);
    eeprom_commit();
    if (stopServer == false)
      Serial.print(name);
    if (stopServer == false)
      Serial.println(" alapertekre irva a memoriaban!");
  }
  szamlalo = intervallum;
}

void eeprom_valid() {
  checkAndReset(alapteljesitmeny, 20, 100, "alapteljesitmeny");
  checkAndReset(ZONE_1, 160, 10, "ZONE_1");
  checkAndReset(ZONE_2, 215, 20, "ZONE_2");
  checkAndReset(sprintzona, 300, 30, "sprintzona");
  checkAndReset(kesleltetes0, 1500, 40, "kesleltetes0");
  checkAndReset(kesleltetes1, 4500, 50, "kesleltetes1");
  checkAndReset(kesleltetes2, 35500, 60, "kesleltetes2");
  checkAndReset(kesleltetes3, 55500, 70, "kesleltetes3");
  checkAndReset(kesleltetessprint, 65500, 80, "kesleltetessprint");
  checkAndReset(teszteles, false, 90, "teszteles");
  checkAndReset(kesleltetesend, 300000, 110, "kesleltetesend");
  checkAndReset(erzekelo, 222, 120, "erzekelo");
  checkAndReset(reboot, false, 130, "reboot");
  checkAndReset(hutesUzemmod, false, 140, "hutesUzemmod");
  checkAndReset(kalibralas, false, 150, "kalibralas");
  checkAndReset(bootCounter, 0, 160, "bootCounter");
}

void readeepromparameter() {
  EEPROM.get(10, ZONE_1);
  EEPROM.get(20, ZONE_2);
  EEPROM.get(30, sprintzona);
  EEPROM.get(40, kesleltetes0);
  EEPROM.get(50, kesleltetes1);
  EEPROM.get(60, kesleltetes2);
  EEPROM.get(70, kesleltetes3);
  EEPROM.get(80, kesleltetessprint);
  EEPROM.get(90, teszteles);
  EEPROM.get(100, alapteljesitmeny);
  EEPROM.get(110, kesleltetesend);
  EEPROM.get(120, erzekelo);
  EEPROM.get(130, reboot);
  EEPROM.get(140, hutesUzemmod);
  EEPROM.get(150, kalibralas);
  EEPROM.get(160, bootCounter);
}

void blekliens() {
  BLEDevice::init("ESP32kliens");
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);  // bluetooth modul teljesitmenye
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
  int pwrAdv = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
  int pwrScan = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_SCAN);
  int pwrDef = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT);
  if (stopServer == false)
    WebSerial.println("Üdvözli az ESP32 ventillátor vezérlő! Csatlakozás a BLE eszközhöz...");
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), false);
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);  // scantime
}

void bleszerver(BLEUUID serviceUUID, BLEUUID charUUID) {
  BLEDevice::init("ESP32szerver");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(serviceUUID);
  pCharacteristic = pService->createCharacteristic(
    charUUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(serviceUUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  if (stopServer == false)
    Serial.println("Waiting a client connection to notify...");
}

void fct_ledUpdate() {
  if (connected == true && wifiOk == true) {
    OnStateLedBleConnected.start();
    OnStateLedWithWifi.stop();
    OnStateLed.stop();
  } else if (connected == false && wifiOk == true) {
    OnStateLedBleConnected.stop();
    OnStateLedWithWifi.start();
    OnStateLed.stop();
  } else if (connected == true && wifiOk == false) {
    OnStateLedBleConnected.start();
    OnStateLedWithWifi.stop();
    OnStateLed.start();
  } else if (connected == false && wifiOk == false) {
    OnStateLedBleConnected.stop();
    OnStateLedWithWifi.stop();
    OnStateLed.start();
  }
}

void fct_Watchdog() {
  if (watchdogCounter < 0 || watchdogCounter > timetosleep)  // ha gond lenne, ill ha adat == 0
  {
    watchdogCounter = 0;  // watchdogreset
  }
  watchdogCounter++;
  if (watchdogCounter == timetosleep) {
    if (teljesitmeny == 0) {
      digitalWrite(LED_wifi, LOW);
      allrelayoff();  // relek kikapcsolása
      digitalWrite(relayOutlet, HIGH);
      digitalWrite(relayEN, LOW);
      if (stopServer == false)
        Serial.println("Minden rele ki!(watchdog)");
      ledPwmBlinking(3);
      if (stopServer == false)
        Serial.println("Going to sleep now");
      delay(500);
      esp_deep_sleep_start();
      if (stopServer == false)
        Serial.println("This will never be printed");
    }
  }
}

void fct_WatchdogReset() {
  watchdogCounter = 0;
  if (stopServer == false)
    Serial.println("WatchdogReset!");
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      if (stopServer == false)
        Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      if (stopServer == false)
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      if (stopServer == false)
        Serial.println("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      if (stopServer == false)
        Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      if (stopServer == false)
        Serial.println("Wakeup caused by ULP program");
      break;
    default:
      if (stopServer == false)
        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}

void click() {
  fct_WatchdogReset();
  stopServer = false;
  fromBootCounter = 0;
  counterFromBoot.start();
  Serial.begin(115200);
  wifiOn();
  ledPwmBlinking(1);
}

void doubleClick() {
  if (stopServer == false)
    Serial.println("Minden WiFi parameter resetelve! Reboot.");
  delay(500);
  wifi_config_t current_conf;
  esp_wifi_get_config((wifi_interface_t)ESP_IF_WIFI_STA, &current_conf);
  memset(current_conf.sta.ssid, 0, sizeof(current_conf.sta.ssid));
  memset(current_conf.sta.password, 0, sizeof(current_conf.sta.password));
  esp_wifi_set_config((wifi_interface_t)ESP_IF_WIFI_STA, &current_conf);
  SPIFFS.remove(ssidPath);
  SPIFFS.remove(passPath);
  SPIFFS.remove(ipPath);
  SPIFFS.remove(gatewayPath);
  ledPwmBlinking(3);
  rebootEsp();
}  // doubleClick

void LongPressStop() {
  digitalWrite(LED_wifi, LOW);
  allrelayoff();  // relek kikapcsolása
  digitalWrite(relayOutlet, HIGH);
  digitalWrite(relayEN, LOW);
  ledPwmBlinking(3);
  esp_deep_sleep_start();
}  // longPress stop

void fct_OnStateLed() {
  digitalWrite(LED_wifi, ledStateOnstateLed);
  ledStateOnstateLed = !ledStateOnstateLed;
}

void fct_OnStateLedWithWifi() {
  digitalWrite(LED_wifi, ledStateWifistateLed);
  ledStateWifistateLed = !ledStateWifistateLed;
}

void fct_OnStateLedBleConnected() {
  digitalWrite(LED_wifi, ledStateWifistateLed);
  ledStateWifistateLed = !ledStateWifistateLed;
}

void ledPwmBlinking(int blinkNumber) {
  for (int i = 0; i < blinkNumber; i++) {
    for (int j = 0; j < 2; j++) {
      time_now_click = millis();
      while ((millis() - time_now_click) < 100) {
        ledcWrite(ledChannel, j * 255);
      }
    }
  }
  ledcWrite(ledChannel, 0);
}

void fct_counterFromBoot() {

  if (fromBootCounter < 0)  // ha gond lenne
  {
    fromBootCounter = 0;
  }
  if (fromBootCounter < serverEnd) {
    fromBootCounter++;
  }
  if (fromBootCounter == 5) {
    digitalWrite(relayOutlet, LOW);  // hosszabító bekapcsolása - edzogorgo felkapcsolasa
  }
  if (fromBootCounter == 90 && !hutesUzemmod && !teszteles && !kalibralas) {
    if (!connected) {
      digitalWrite(relayOutlet, HIGH);  // hosszabító lekapcsolása ha nincs bluetooth csatlakozas
    }
  }
  if (fromBootCounter == serverEnd && !hutesUzemmod && !teszteles && !kalibralas)  // webserver leállítása
  {
    stopServer = true;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    counterFromBoot.stop();
  } else if (fromBootCounter == serverEnd && (hutesUzemmod || teszteles || kalibralas)) {
    stopServer = false;
    counterFromBoot.stop();
  }
}

void fct_goToSleep() {
  delay(500);
  if (bootCounter < 0 || bootCounter > 2) {
    if (stopServer == false)
      Serial.print("bootCounter parameter hiba!");
    bootCounter = 0;
    EEPROM.put(160, bootCounter);
    eeprom_commit();
  }
  if (bootCounter > 0) {
    bootCounter = bootCounter - 1;
    EEPROM.put(160, bootCounter);
    eeprom_commit();
    if (bootCounter == 0) {
      teszteles = false;
      hutesUzemmod = false;
      kalibralas = false;
      EEPROM.put(90, teszteles);
      eeprom_commit();
      EEPROM.put(150, kalibralas);
      eeprom_commit();
      EEPROM.put(140, hutesUzemmod);
      eeprom_commit();
      EEPROM.put(160, bootCounter);
      eeprom_commit();
    }
  }
  if (!reboot) {
    esp_sleep_wakeup_cause_t cause;
    cause = esp_sleep_get_wakeup_cause();
    if (stopServer == false)
      Serial.print("Wakeup caused by(number): ");
    if (stopServer == false)
      Serial.println(cause);
    switch (cause) {
      case ESP_SLEEP_WAKEUP_UNDEFINED:
        if (stopServer == false)
          Serial.println("Reset was not caused by exit from deep sleep.");

        esp_deep_sleep_enable_gpio_wakeup(BIT(D1), ESP_GPIO_WAKEUP_GPIO_LOW);  // biztos ami biztos
        esp_deep_sleep_start();
        break;
      default:
        esp_deep_sleep_enable_gpio_wakeup(BIT(D1), ESP_GPIO_WAKEUP_GPIO_LOW);  // biztos ami biztos
        if (stopServer == false)
          Serial.printf("Wakeup caused by: %d\n", cause);
        break;
    }
  }
  if (reboot) {
    reboot = false;
    EEPROM.put(130, reboot);
    eeprom_commit();
  }
}

void rebootEsp() {
  reboot = true;
  EEPROM.put(130, reboot);
  eeprom_commit();
  delay(100);
  ESP.restart();
}

void wifiOn() {
  initSPIFFS();
  initWebSocket();
  // Load values saved in SPIFFS
  ssid = readFile(SPIFFS, ssidPath);
  pass = readFile(SPIFFS, passPath);
  ip = readFile(SPIFFS, ipPath);
  gateway = readFile(SPIFFS, gatewayPath);
  if (stopServer == false)
    Serial.println(ssid);
  if (stopServer == false)
    Serial.println(pass);
  if (stopServer == false)
    Serial.println(ip);
  if (stopServer == false)
    Serial.println(gateway);
  if (initWiFi()) {
    wifiOk = true;
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/index.html", "text/html", false);
    });
    server.serveStatic("/", SPIFFS, "/");
    server.begin();
  } else {
    wifiOk = false;
    // Connect to Wi-Fi network with SSID and password
    if (stopServer == false)
      Serial.println("Setting AP (Access Point)");
    // NULL sets an open Access Point
    WiFi.softAP("ESP-WIFI-MANAGER", NULL);
    IPAddress IP = WiFi.softAPIP();
    if (stopServer == false)
      Serial.print("AP IP address: ");
    if (stopServer == false)
      Serial.println(IP);

    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/wifimanager.html", "text/html");
    });

    server.serveStatic("/", SPIFFS, "/");

    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for (int i = 0; i < params; i++) {
        AsyncWebParameter *p = request->getParam(i);
        if (p->isPost()) {
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            ssid = p->value().c_str();
            if (stopServer == false) Serial.print("SSID set to: ");
            if (stopServer == false) Serial.println(ssid);
            // Write file to save value
            writeFile(SPIFFS, ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            pass = p->value().c_str();
            if (stopServer == false) Serial.print("Password set to: ");
            if (stopServer == false) Serial.println(pass);
            // Write file to save value
            writeFile(SPIFFS, passPath, pass.c_str());
          }
          // HTTP POST ip value
          if (p->name() == PARAM_INPUT_3) {
            ip = p->value().c_str();
            if (stopServer == false) Serial.print("IP Address set to: ");
            if (stopServer == false) Serial.println(ip);
            // Write file to save value
            writeFile(SPIFFS, ipPath, ip.c_str());
          }
          // HTTP POST gateway value
          if (p->name() == PARAM_INPUT_4) {
            gateway = p->value().c_str();
            if (stopServer == false) Serial.print("Gateway set to: ");
            if (stopServer == false) Serial.println(gateway);
            // Write file to save value
            writeFile(SPIFFS, gatewayPath, gateway.c_str());
          }
        }
      }
      request->send(200, "text/plain", "Done. ESP will restart, connect to your router and go to IP address: " + ip);
      delay(3000);
      rebootEsp();
    });
    server.begin();
  }
}

void wifiInterface() {
  ElegantOTA.begin(&server);  // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
  // Disable Auto Reboot
  ElegantOTA.setAutoReboot(false);
  WebSerial.begin(&server);
  WebSerial.msgCallback(recvMsg);
  server.begin();
  if (stopServer == false)
    Serial.println("Webserial and ElegantOTA started!");
  if (stopServer == false)
    Serial.println("HTTP server started:" + WiFi.localIP().toString());
}

void setup() {
  pinMode(LED_wifi, OUTPUT);     // gpio4
  pinMode(LED_eeprom, OUTPUT);   // gpio5
  pinMode(relayOutlet, OUTPUT);  // kulso aljzat
  pinMode(relayEN, OUTPUT);
  for (int i = 0; i < NUM_OUTPUTS; i++) {
    pinMode(outputGPIOs[i], OUTPUT);
  }
  pinMode(WAKEUP_PIN, INPUT_PULLUP);
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(LED_eeprom, ledChannel);
  allrelayoff();
  digitalWrite(relayOutlet, HIGH);  // hosszabító lekapcsolása BOOT utan
  esp_deep_sleep_enable_gpio_wakeup(BIT(D1), ESP_GPIO_WAKEUP_GPIO_LOW);
  Serial.begin(115200);  // a serial.print-eket kikommenteltem, ez csak azért kell hogy íráskor ne kelljen a reset + boot gombokat nyomogatni
  delay(100);
  ++bootCount;
  if (stopServer == false)
    Serial.println("Boot number: " + String(bootCount));
  // Print the wakeup reason for ESP32
  print_wakeup_reason();
  EEPROM.begin(memoria_meret);
  delay(100);
  eeprom_check();
  readeepromparameter();
  eeprom_valid();
  delay(100);
  fct_goToSleep();
  watchDOG.start();
  Serial.println("Start watchdog counter!");
  ledUpdate.start();
  counterFromBoot.start();
  OnStateLedBleConnected.stop();
  OnStateLedWithWifi.stop();
  OnStateLed.stop();
  button.attachDoubleClick(doubleClick);                             // wifi parameter reset
  button.attachLongPressStop(LongPressStop);                         // tesztmod aktivalasa
  button.attachClick(click);                                         // reset watchdog
  xTaskCreatePinnedToCore(loop2, "loop2", 10000, NULL, 1, NULL, 1);  // loop 2 10000-nél lehet kevesebb is, én nem kísérleteztem, gomb kezelés, led kezelés, otp update...
  wifiOn();
  wifiInterface();
  digitalWrite(relayEN, HIGH);  // relek engedelyezese
  stopServer = false;
  Serial.print("Hutesuzemmod aktív?: ");
  Serial.println(hutesUzemmod);
  if (hutesUzemmod < 0 || hutesUzemmod > 1 || teszteles < 0 || teszteles > 1 || kalibralas < 0 || kalibralas > 1) {
    kalibralas = false;
    hutesUzemmod = false;
    EEPROM.put(140, hutesUzemmod);
    eeprom_commit();
    teszteles = false;
    Serial.println("hutesUzemmod/teszteles/kalibralas parameter hiba!");
  }
  Serial.print("Tesztelés aktív?: ");
  Serial.println(teszteles);
  if (!teszteles && !hutesUzemmod && !kalibralas)  // teszteleshez
  {
    blekliens();
    if (erzekelo == 111) {
      serviceerzekeloUUID = SERVICE1_UUID;
      charerzekeloUUID = CHARACTERISTIC1_UUID;

      bleszerver(serviceerzekeloUUID, charerzekeloUUID);
    } else if (erzekelo == 222) {

      serviceerzekeloUUID = SERVICE2_UUID;
      charerzekeloUUID = CHARACTERISTIC2_UUID;
      bleszerver(serviceerzekeloUUID, charerzekeloUUID);
    }
  }
}

void loop() {
  time_now = millis();
  if (time_now - time_elapsedtime >= period) {
    time_elapsedtime = time_now;
    if (kalibralas < 0 || kalibralas > 1) {
      kalibralas = false;
      if (stopServer == false)
        Serial.println("kalibralas parameter hiba!");
    }
    if (!kalibralas) {
      if (hutesUzemmod < 0 || hutesUzemmod > 1) {
        hutesUzemmod = false;
        EEPROM.put(140, hutesUzemmod);
        eeprom_commit();
        if (stopServer == false)
          Serial.println("hutesUzemmod parameter hiba!");
      }
      if (!hutesUzemmod) {
        if (teszteles < 0 || teszteles > 1) {
          teszteles = false;
          if (stopServer == false)
            Serial.println("teszteles parameter hiba!");
        }
        readeepromparameter();
        if (teszteles) {
          if (stopServer == false)
            WebSerial.println("Vigyazz! A Teszt mod aktiv! (webserial: run)");
          connected = true;
          notification = true;
          doConnect = false;
          randNumber = random(60, 350);
          adat = randNumber;
        }
        if (doConnect == true) {
          if (connectToServer(serviceerzekeloUUID, charerzekeloUUID)) {
            if (stopServer == false)
              Serial.println("Csatlakozva a BLE szerverhez!");
            if (stopServer == false)
              WebSerial.println("Csatlakozva a BLE szerverhez!");
          } else {
            if (stopServer == false)
              Serial.println("Nem lehet csatlakozni a BLE szerverhez!");
            if (stopServer == false)
              WebSerial.println("Nem lehet csatlakozni a BLE szerverhez!");
          }
          doConnect = false;
        }
        if (connected) {
          if (notification == false) {
            if (stopServer == false)
              Serial.println("Adatok fogadasanak bekapcsolasa!");
            if (stopServer == false)
              WebSerial.println("Adatok fogadasanak bekapcsolasa!");
            const uint8_t onPacket[] = { 0x01, 0x0 };
            pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t *)onPacket, 2, true);
            notification = true;
          }
          digitalWrite(relayOutlet, LOW);  // hosszabító bekapcsolása
          if (teljesitmeny > 0 && !teszteles) {
            fct_WatchdogReset();
          }
          atlagolas();
          ventillatorvezerles();
          kiiras();
        } else if (doScan) {
          if (stopServer == false)
            Serial.println("Lecsatlakozott.");
          if (stopServer == false)
            Serial.println("Minden rele ki! Lecsatlakozas utan. ");
          alapparameter();
          BLEDevice::getScan()->start(120);  // ha lecsatlakozott ujra keres
        }
      } else if (hutesUzemmod) {
        if (stopServer == false)
          Serial.println("Hutes-uzemmod aktiv.");
        if (stopServer == false)
          WebSerial.println("Vigyazz! Hutesuzzemod aktiv! (webserial: hutesuzemmodki)");
        fct_WatchdogReset();
      }

    } else if (kalibralas) {
      if (stopServer == false)
        Serial.println("Kalibralas-uzemmod aktiv.");
      if (stopServer == false)
        WebSerial.println("Vigyazz! Kalibralas aktiv! (webserial: kalibralaski)");
    }
  }
}

void loop2(void *pvParameter) {

  while (1) {
    if (stopServer == false) {
      ElegantOTA.loop();
      ws.cleanupClients();
    }
    watchDOG.update();
    if (fromBootCounter < serverEnd) {
      counterFromBoot.update();
    }
    ledUpdate.update();
    OnStateLedBleConnected.update();
    OnStateLed.update();
    OnStateLedWithWifi.update();
    button.tick();
  }
}