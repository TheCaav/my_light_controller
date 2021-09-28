#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <Keypad.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#ifndef APSSID
#define STASSID "raspi-webgui";
#define STAPSK "BYip3sj5pbnYtswmM3r4"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;
const char* mqtt_server = "10.3.141.1";
HTTPClient http;
unsigned long startTime;

WiFiUDP udp;
uint16_t port = 8125;
IPAddress host(10, 3, 141, 1);
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte colPins[COLS] = {23,22,2,21}; //connect to the column pinouts of the keypad
byte rowPins[ROWS] = {19,18,15,4}; //connect to the row pinouts of the keypad
//byte colPins[COLS] = {10,36,8,33};
//byte rowPins[ROWS] = {30, 7, 31,9};

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();



void sendPost(String key, String val) {
  http.begin("http://10.3.141.234:80/?" + key + "=" + val);
  http.POST("1");
  http.end();
}

void printBMEValues()
{
    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);

    udp.beginPacket(host, port);
    String message = "temp,place=room,sensor=bme280:" + String(temp_event.temperature) + "|g";
    char chars[50];
    message.toCharArray(chars, 40, 0);
    udp.print(message.c_str());
    Serial.println(chars);
    udp.endPacket();

    udp.beginPacket(host, port);
    message = "humi,place=room,sensor=bme280:" + String(humidity_event.relative_humidity) + "|g";
    chars[50];
    message.toCharArray(chars, 40, 0);
    udp.print(message.c_str());
    Serial.println(chars);
    udp.endPacket();

    udp.beginPacket(host, port);
    message = "pres,place=room,sensor=bme280:" + String(pressure_event.pressure) + "|g";
    char chars2[50];
    message.toCharArray(chars2, 40, 0);
    udp.print(message.c_str());
    Serial.println(chars2);
    udp.endPacket();

    Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");

    Serial.print(F("Humidity = "));
    Serial.print(humidity_event.relative_humidity);
    Serial.println(" %");

    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa");

    Serial.println();
}

void setup(){
  Serial.begin(9600);
  Serial.println("Starting:");
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // use other pins for i2c
  Wire.begin(26, 27);

  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname("ESP32-LightController");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
      delay(500);
      Serial.print(".");
  }
  WiFi.setAutoReconnect(true);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  if (!bme.begin(0x76))
    {
        Serial.println("Could not find BME280 Sensor!");
    }
    else
    {
        Serial.println("-- Default Test BME280 --");
        Serial.println("normal mode, 16x oversampling for all, filter off,");
        Serial.println("0.5ms standby period");
        bme_temp->printSensorDetails();
        bme_pressure->printSensorDetails();
        bme_humidity->printSensorDetails();
  }
  startTime = millis();
}
  
void loop(){
  char key = keypad.getKey();
    // just print the pressed key
   if (key){
    Serial.println(key);
  } 
  
  // this checkes if 4 is pressed, then do something. Here  we print the text but you can control something.
  if (key == '1'){
    sendPost("color", "0xff0000");
  } else if (key == '2'){
    sendPost("animation", "rainbowWithGlitter");
    sendPost("frequency", "1000");
    sendPost("changeHue", "true");
  } else if (key == 'A'){
    sendPost("color", "0xFFFFFF");
  } else if (key == '4'){
    sendPost("color", "0x00ff00");
  } else if (key == '5'){
    sendPost("animation", "confetti");
  } else if (key == 'B'){
    sendPost("color", "0xAAAAAA");
  } else if (key == '7'){
    sendPost("color", "0x0000ff");
  } else if (key == '8'){
    sendPost("colorTemperature", "DirectSunlight");
  } else if (key == 'C'){
    sendPost("color", "0x555555");
  } else if (key == '*'){
    sendPost("ToggleHueRotation", "");
  } else if (key == '0'){
    sendPost("colorTemperature", "Candle");
  } else if (key == 'D'){
    sendPost("color", "0x000000");
    sendPost("frequency", "120");
  }
  if (millis() - startTime >= 5000) {
    // 5 seconds have elapsed. ... do something interesting ...
    startTime = millis();
    printBMEValues();
   }

}