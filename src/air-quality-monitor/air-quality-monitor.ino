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

Adafruit_BME280 bme;    //BME280 object
PASCO2Ino co2_sensor;        //PASCO2 object

unsigned long meas_period = 5000;       //Measurement period in ms
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
}

void loop()
{
  delay(meas_period);

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
}
