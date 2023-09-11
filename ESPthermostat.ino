// started with https://iotprojectsideas.com/temperature-control-with-esp8266-asyncwebserver/

// todo:
// timer ?
// persist settings

#include <Arduino.h>

#if defined(ESP8266)
/* ESP8266 Dependencies */
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#elif defined(ESP32)
/* ESP32 Dependencies */
#include <WiFi.h>
#include <AsyncTCP.h>
#endif

#include <Ticker.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ESPDash.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "creds.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool debug = true;
//#define AP_Mode
#ifdef AP_Mode
/* Your SoftAP WiFi Credentials */
const char* ssid = "TempratureController"; // SSID
const char* password = ""; // Password
#else
const char* ssid = SSID; // SSID
const char* password = PASS; // Password
#endif

#define MQTT_HOST IPAddress(10,0,0,3)
#define MQTT_PORT 1883

// Setpoint and hysteresis values (in degrees Celsius)
float setpoint = 27;
float hysteresis = 1;

volatile float currentTemp = 1.0;
bool heaterOn;

unsigned long	stopTime=0;

// Pin numbers for the heating and cooling relays
#define HEATING_PIN 14 //D5 of esp8266
//const int COOLING_PIN = 12; //D6 of esp8266

#define DALLAS_RES 12  // 9-12 bits of resolution
#define ONE_WIRE_BUS 13 //D7  of esp8266
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress addrThermometer;
Ticker dallasReady;
bool conversionRunning = false;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

Ticker oledmqttTimer;

// Control mode (automatic or manual)
bool automaticMode = 1;
bool heaterbtn = 0;

// counter to cycle info top row of oled
int info_mode;
bool flipper;   // flipper for flashing border

volatile bool _300msFlag;
volatile bool updateMqttFlag;

/* Start Webserver */
AsyncWebServer server(80);

/* Attach ESP-DASH to AsyncWebServer */
ESPDash dashboard(&server);
/*
  Dashboard Cards
  Format - (Dashboard Instance, Card Type, Card Name, Card Symbol(optional) )
*/
Card temperature(&dashboard, TEMPERATURE_CARD, "Current Temperature", "°C");
Card tempSet(&dashboard, TEMPERATURE_CARD, "Temperature Set Point", "°C");
Card tempHyss(&dashboard, TEMPERATURE_CARD, "Hysteresis Range", "°C");
Card setTemp(&dashboard, SLIDER_CARD, "Temperature Set Point", "", 0, 50);
Card setHyss(&dashboard, SLIDER_CARD, "Hysteresis Range", "", 0, 5);
Card autoMode(&dashboard, BUTTON_CARD, "Auto Mode");
Card ModeStatus(&dashboard, STATUS_CARD, "Auto Mode Status");
Card Heater(&dashboard, BUTTON_CARD, "Heater");
Card TimeNow(&dashboard, GENERIC_CARD, "Time Now");
Card TimeStop(&dashboard, GENERIC_CARD, "Time Stop");
Card TimeAdd(&dashboard, BUTTON_CARD, "Timer+30m");

void enable_heater() {
      digitalWrite(HEATING_PIN, HIGH);
      heaterOn = true;
}

void disable_heater() {
      digitalWrite(HEATING_PIN, LOW);
      heaterOn = false;
}

void connectToWifi() {
  Serial.println(F("Connecting to Wi-Fi..."));
  WiFi.begin(ssid, password);
}

void connectToMqtt() {
  Serial.println(F("Connecting to MQTT..."));
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println(F("Connected to Wi-Fi."));
  connectToMqtt();
  timeClient.begin();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println(F("Disconnected from Wi-Fi."));
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

char MQTT_SETPOINT[] = "thermo/setpoint";
char MQTT_HYSTERESIS[] = "thermo/hysteresis";
char MQTT_MODE[] = "thermo/mode";
char MQTT_STOPTIME[] = "thermo/stoptime";

void onMqttConnect(bool sessionPresent) {
  Serial.println(F("Connected to MQTT."));
  Serial.print(F("Session present: "));
  Serial.println(sessionPresent);
//  uint16_t packetIdSub = mqttClient.subscribe("thermo/setpoint", 2);
  mqttClient.subscribe(MQTT_SETPOINT, 2);
  mqttClient.subscribe(MQTT_HYSTERESIS, 2);
  mqttClient.subscribe(MQTT_MODE, 2);
  mqttClient.subscribe(MQTT_STOPTIME, 2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println(F("Disconnected from MQTT."));

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(10, connectToMqtt);
  }
}
#define MPL 32
char mqttpayload[MPL];
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  if (len >= MPL) return;
  memcpy(mqttpayload, payload, MPL);
  mqttpayload[len]=0;

  if (!strcmp(topic,MQTT_SETPOINT)) {
    float t = atof(mqttpayload);
    if (t > 0.001) {
      setpoint = t;
      if (debug) Serial.printf ("mqtt: got setpoint %2.2f\n",setpoint);
    }
  } else if (!strcmp(topic,MQTT_HYSTERESIS)) {
    hysteresis = atof(mqttpayload);
    if (debug) Serial.printf ("mqtt: got hysteresis %2.2f\n",hysteresis);
  } else if (!strcmp(topic,MQTT_MODE)) {
    automaticMode = atoi(mqttpayload);
    if (debug) Serial.printf ("mqtt: got mode %d\n",automaticMode);
  } else if (!strcmp(topic,MQTT_STOPTIME)) {
    if (strlen(payload) > 1) {
      if (debug) Serial.printf ("mqtt: got stopTime str %s\n",mqttpayload);
      long unsigned old_stopTime = stopTime;
      if (mqttpayload[0] == '+') {
        stopTime = timeClient.getEpochTime()+atol(mqttpayload+1);
        automaticMode = true;
      } else {
        stopTime = atol(mqttpayload);
/* maybe dont need any of this...
        if (debug) Serial.printf ("mqtt: got stopTime %u\n",stopTime);
        // only set auto if its an update some time after boot.
        // let the mqtt stuff decice on bootup
        if ((stopTime > timeClient.getEpochTime()) && (millis() > 3000) ){
          automaticMode = true;
        } else {
          stopTime = 0;
        }
*/
      }
      if (old_stopTime != stopTime) updateMqttFlag = true;
    }
  } else {
    Serial.println("Publish received.");
    Serial.print("  topic: ");
    Serial.println(topic);
    Serial.print("  qos: ");
    Serial.println(properties.qos);
    Serial.print("  dup: ");
    Serial.println(properties.dup);
    Serial.print("  retain: ");
    Serial.println(properties.retain);
    Serial.print("  len: ");
    Serial.println(len);
    Serial.print("  index: ");
    Serial.println(index);
    Serial.print("  total: ");
    Serial.println(total);
  }
}
void setup() {
  Serial.begin(115200);
  Serial.print("\r\n\r\n");

  // Initialize the relays and temperature sensor
  pinMode(HEATING_PIN, OUTPUT);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1); // Draw 1X-scale text
  display.setTextColor(SSD1306_WHITE);

 // Start up the dallas library
  sensors.begin();
  sensors.setResolution(DALLAS_RES);

  if (!sensors.getAddress(addrThermometer, 0)) {
    Serial.println(F("Unable to find address for Device 0"));
  }
  Serial.print(F("Device 0 Address: ")); 
  for (uint8_t i = 0; i < 8; i++)
  {
    if (addrThermometer[i] < 16) Serial.print("0");
    Serial.print(addrThermometer[i], HEX);
  }
  Serial.println();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

#ifdef AP_Mode
  /* Start Access Point */
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
#else
  display.setCursor(4, 4);
  display.println(F("Connecting to AP:"));
  display.setCursor(4, 13);
  display.println(ssid);
  display.drawRect(0,0,128,64,SSD1306_WHITE);
  display.display();


  Serial.print(F("Connecting to "));
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  display.setCursor(4, 22);
  int retries = 20;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    display.print(".");
    display.display();
    if (--retries == 0) {
      WiFi.mode(WIFI_OFF);
      Serial.print(F("rebooting in 1 sec"));
      display.clearDisplay();
      display.setCursor(2, 28);
      display.print(F("rebooting in 1 sec"));
      for (int i=0; i<32; i+=2) {
        display.drawRect( 0+i, 0+i, 128-2*i, 64-2*i, SSD1306_WHITE);
        display.display();
        delay(1000/16);
      }
      ESP.restart();
    }
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
#endif

  disable_heater();

  /* Start AsyncWebServer */
  server.begin();

// timezone is in seconds
  timeClient.setTimeOffset(10*60*60);

  autoMode.attachCallback([&](int value) {
    automaticMode = value;
    updateMqttFlag = 1;
  });

  setTemp.attachCallback([&](int value) {
    setpoint = value;
    updateMqttFlag = 1;
  });

  setHyss.attachCallback([&](int value) {
    hysteresis = value;
    updateMqttFlag = 1;
  });

  Heater.attachCallback([&](int value) {
    heaterbtn = value;
    if (automaticMode == 0) {
      if (value == 1) {
	      enable_heater();
      } else {
        disable_heater();
      }
    }
  });

  TimeAdd.attachCallback([&](int value) {
    #define ADD 10
//    #define ADD 30*60
    if (debug) Serial.printf ("stop was %d ", stopTime);
    if (stopTime) {
      stopTime += ADD;
    } else {
      stopTime = timeClient.getEpochTime()+ADD;
    }
    automaticMode = true;
    updateMqttFlag = true;
    if (debug) Serial.printf (" now %d \n", stopTime);

  });

  oledmqttTimer.attach_ms(300,setOledFlag);
}

void setOledFlag(){
  _300msFlag=true;
}

void moveCursor(int xdiff, int ydiff) {
  int tx=display.getCursorX() + xdiff;
  int ty=display.getCursorY() + ydiff;
  if ((tx<0) || (tx>=SCREEN_WIDTH)) tx=display.getCursorX();
  if ((ty<0) || (ty>=SCREEN_HEIGHT)) tx=display.getCursorY();
  display.setCursor(tx,ty);
} 

#define MAX_INFO_MODE 4
void update_oled() {

  long zero=millis();

  IPAddress me;
  #ifdef AP_Mode
    me = WiFi.softAPIP();
  #else
    me =  WiFi.localIP();
  #endif

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2); // Draw 2X-scale text
  display.setCursor(4, 2);
  switch (info_mode) {
    case 0:
    default:
      if (strlen( me.toString().c_str() )-3 > 9 ) {
        display.setTextSize(1); // too big => Draw 1X-scale text
        display.printf("IP: "); display.print(me);
      } else { // can fit 9 digits with squished . spacing
      // some hackery to save some pixels around the .
      // at 2x font can fit 9 digits with 3 .
      display.setTextSize(2); // Draw 3X-scale text
                          display.print(me[0]);
        moveCursor(-4,0); display.print (".");
        moveCursor(-3,0); display.print(me[1]);
        moveCursor(-4,0); display.print (".");
        moveCursor(-3,0); display.print(me[2]);
        moveCursor(-4,0); display.print (".");
        moveCursor(-3,0); display.print(me[3]);
      }
        break;

    case 1:
        display.printf("RAM:%6d", ESP.getFreeHeap());
        break;

    case 2:
        display.printf("Up:%7d", millis()/1000);
        break;

    case 3:
        display.printf("RSSI:%5d", WiFi.RSSI());
        break;
  }

  display.setTextSize(3); // Draw 3X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(9, 21);
  // ugly hack here to squeeze the space around decimal point
  char temp[8];   sprintf(temp, "%2.2f",currentTemp);
  temp[2] = 0;                // replace . with nul
  int t_int = atoi(temp);     // grab the int part
  int t_frac = atoi(temp+3);  // grab the fractional part

// digits left of decimal point
  display.printf("%2d", t_int);
// draw a pretier circle for the decimal pt
  display.fillCircle(display.getCursorX()+2, display.getCursorY()+16, 3, WHITE);
// digits right of decimal point
  moveCursor(8,0); display.printf("%02d", t_frac);

  flipper = !flipper;
  if (flipper) {
    display.drawRect(display.getCursorX()+2, display.getCursorY(), 5, 5, WHITE);     // put degree symbol ( ° )
  } else {
    display.fillRect(display.getCursorX()+2, display.getCursorY(), 5, 5, WHITE);     // put degree symbol ( ° )
  }

  display.setCursor(display.getCursorX()+9, display.getCursorY());
  display.println("C");

// Bottom status line  
  display.setTextSize(2); // Draw 1X-scale text
  switch (info_mode) {
    case 0:
    case 2:
      display.setCursor(4, 47);
      display.printf("%s", automaticMode ? "Auto" : "Man.");
      display.setCursor(SCREEN_WIDTH-3-48, 47);
      display.printf("%s", heaterOn ? "HEAT" : "IDLE");
      break;

    case 1:
    case 3:
    // write the set temperature
      display.setCursor(2, 47);
      if (hysteresis < 0.001) {
        display.printf(" Set %2.1f",setpoint);
      } else {
        display.printf(" %2.1f%c%2.1f",setpoint,0xf0, hysteresis/2);
      }
      break;
  }

  display.display();

  if (heaterOn) {
    display.invertDisplay(true);
  } else {
    display.invertDisplay(false);
  }
}

void update_mqtt_log(){
  static char mqttmsg[128];
  snprintf (mqttmsg, 127, "%s,%d,%2.2f,%2.2f,%2.2f,%d,%d"
        , WiFi.macAddress().c_str(),millis(),currentTemp, setpoint
        , hysteresis, heaterOn, automaticMode);
  mqttClient.publish("thermo/data", 0, true, mqttmsg);
}

void update_mqtt_settings(){
  static char mqttmsg[128];

  snprintf (mqttmsg, 127, "%d",automaticMode);
  mqttClient.publish(MQTT_MODE, 0, true, mqttmsg);

  snprintf (mqttmsg, 127, "%2.2f",setpoint);
  mqttClient.publish(MQTT_SETPOINT, 0, true, mqttmsg);

  snprintf (mqttmsg, 127, "%2.2f",hysteresis);
  mqttClient.publish(MQTT_HYSTERESIS, 0, true, mqttmsg);

  snprintf (mqttmsg, 127, "%u",stopTime);
  mqttClient.publish(MQTT_STOPTIME, 0, true, mqttmsg);
}

void readTemp() {
  currentTemp = sensors.getTempCByIndex(0);
  conversionRunning = false;
}

// LOOP   LOOP   LOOP   LOOP   LOOP   LOOP

int _300ms_counter=0;
void loop() {

  if (!conversionRunning) {
    conversionRunning = true;
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
    sensors.setWaitForConversion(true);
    dallasReady.once_ms(750 / (1 << (12 - sensors.getResolution() )) , readTemp);
  }

  // flag set every 300ms from ticker
  if (_300msFlag==true) {
    _300msFlag=false;
    update_oled();
    dashboard.sendUpdates();

    // every 3000 ms
    if (++_300ms_counter > 10) {
      _300ms_counter = 0;
      // log data ever 3 secs
      update_mqtt_log();

      // ~3secs is also a good time to change the status line on oled
      info_mode=(info_mode+1) % MAX_INFO_MODE;

      timeClient.update();
//      if (debug) Serial.println(timeClient.getFormattedTime());
    }
  }

  if (updateMqttFlag==true) {
    updateMqttFlag=false;
    update_mqtt_settings();
  }

  // If in automatic mode, control the heating and cooling relays based on the setpoint and hysteresis
  if (automaticMode == 1) {
    ModeStatus.update("Enabled", "success");
    if (currentTemp > -55) {
      if (currentTemp > setpoint + hysteresis/2) {
        disable_heater();
      } else if (currentTemp < setpoint - hysteresis/2) {
        enable_heater();
      }
    } else {
      disable_heater();
    }
    if ((stopTime > 0) && (timeClient.getEpochTime() > stopTime)) {
      automaticMode = false;
      disable_heater();
      stopTime = 0;
    }
  }

  else if (automaticMode == 0) {
    ModeStatus.update("Disabled", "danger");
    if (heaterbtn == 1) {
    	enable_heater();
    }
    else {
      disable_heater();
    }
  }
  /* Update Card Values */
  temperature.update(currentTemp);
  tempSet.update(setpoint);
  tempHyss.update(hysteresis);
  setHyss.update(hysteresis);
  setTemp.update(setpoint);
  autoMode.update(automaticMode);
  Heater.update(heaterbtn);
  TimeNow.update(timeClient.getFormattedTime());

  if (stopTime > 0) {
    if (timeClient.getEpochTime()>stopTime) {
      TimeStop.update("passed");
    } else {
      unsigned long t = stopTime-timeClient.getEpochTime();
      unsigned int t_days = (t / 86400L);
      unsigned int t_hours =(t % 86400L) / 3600;
      unsigned int t_mins = (t % 3600) / 60;
      unsigned int t_secs = t % 60;
      char t_str[24];
      snprintf (t_str, 23, "%d days, %2d:%02d:%02d", t_days, t_hours, t_mins, t_secs);
      TimeStop.update(t_str);
    }
  } else {
      TimeStop.update("idle");
  }

  delay(100);
}

