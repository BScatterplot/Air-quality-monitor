/*********************************************************************
Air quality monitor
Measures temperature, humidity, barometric pressure, and CO2 levels.

Uses a BME280 humidity sensor and an XENSIV PAS CO2 sensor.

BME280 needs to have SDO grounded, resulting in an address of 0x76.

**********************************************************************/

//defines & includes:

//BME280
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define BME_I2C_ADDR 0x76

//XENSIV PAS CO2
#include <Arduino.h>
#include <pas-co2-ino.hpp>
#define XEN_I2C_FREQ_HZ  100000  

//Wifi
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#define MSG_BUFFER_SIZE (500)
#include "secret.h"

//const char* ssid = "******";
//const char* wifi_pw = "******";
//const char* mqtt_server = "******";
//const char* clientId = "******";
//const char* mqtt_user = "******";
//const char* mqtt_pw = "******";

const char* outTopic_info = "airquality/info";
const char* outTopic_data = "airquality/data";

Adafruit_BME280 bme;    //BME280 object
PASCO2Ino co2_sensor;        //PASCO2 object

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
char mqttOutMsg[MSG_BUFFER_SIZE];

unsigned long meas_period = 60000;       //Measurement period in ms
Error_t co2_err;

void setup() 
{
    Serial.begin(9600);
    while(!Serial);    // time to get serial running
    Serial.println(F("Air quality monitor"));

    //BME280
    unsigned bme_status;
    bme_status = bme.begin(BME_I2C_ADDR);
    if (!bme_status)
    {
      Serial.println("Error connecting to BME280");
      while (1) delay(10);
    }
    
    //PASCO2
    Wire.begin();
    Wire.setClock(XEN_I2C_FREQ_HZ);
    co2_err = co2_sensor.begin();
    if (co2_err != XENSIV_PASCO2_OK)
    {
      Serial.print("PASCO2 init error: ");
      Serial.println(co2_err);
      while (1) delay(10);
    }
    co2_err = co2_sensor.startMeasure(meas_period/1000);
    if (co2_err != XENSIV_PASCO2_OK)
    {
      Serial.print("PASCO2 start measure error: ");
      Serial.println(co2_err);
      while (1) delay(10);
    }
    //wifi
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    delay(meas_period);
}

void loop()
{
  client.loop();

  if ((millis() - lastMsg) > meas_period)
  {
    lastMsg = millis();
  
    if (!client.connected())
    {
      reconnect();
    }


    float curCo2, curTempC, curPressureHPa, curHumidity;
    int16_t co2ppm;

    //Read BME280 first since it feeds into the CO2 sensor
    curTempC = bme.readTemperature();
    curPressureHPa = (bme.readPressure() / 100.0F);
    curHumidity = bme.readHumidity();

    Serial.printf("Temperature: %.1f °C\n",curTempC);
    Serial.printf("Pressure: %.1f hPa\n", curPressureHPa);
    Serial.printf("Humidity: %.1f\n",curHumidity);
    
    co2_err = co2_sensor.getCO2(co2ppm);
    if (co2_err != XENSIV_PASCO2_OK)
    {  
      if (co2_err == XENSIV_PASCO2_ERR_COMM);
      {  delay(600);
        co2_err = co2_sensor.getCO2(co2ppm);
        if (co2_err != XENSIV_PASCO2_OK)
        {
          Serial.print("Get CO2 error: ");
          Serial.println(co2_err);
        }
      }
    }
    Serial.printf("CO2 level: %d ppm\n",co2ppm);

    //Example code sets the pressure reference, delays, then reads the value.
    co2_err = co2_sensor.setPressRef(curPressureHPa);
    if (co2_err != XENSIV_PASCO2_OK)
    {
      Serial.print("Pressure reference error: ");
      Serial.println(co2_err);
    }

    format_msg(mqttOutMsg, curTempC, curPressureHPa, curHumidity, co2ppm);
    client.publish(outTopic_data, mqttOutMsg);
    Serial.println(mqttOutMsg);
  }
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Attempting Wifi connection to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, wifi_pw);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  Serial.print("Attempting MQTT connection...");
  
  // Attempt to connect
  if (client.connect(clientId, mqtt_user, mqtt_pw)) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    client.publish(outTopic_info, "Connected");
  } else {
    Serial.print("failed, rc= ");
    Serial.println(client.state());
  }
}

void format_msg(char* buffer, float tempC, float pressureHPa, float humidity, unsigned int lvlCO2)
{
  snprintf(buffer, MSG_BUFFER_SIZE, "{\n\"TempC\":%.1f,\n\"Humidity\":%.1f,\n\"CO2\":%d,\n\"Pressure\":%.1f\n}", tempC, humidity, lvlCO2, pressureHPa);
}