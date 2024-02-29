/*
  Need to measure:
  - Position
  - External pressure / altitude
  - Humidity
  - Internal and external temperature
  - Attitude
  - Attitude rate
  - Acceleration
  - Compass heading
  - Battery current
  - Battery temperature

Uses 30310 bytes of storage
  CSV format:
  Timestamp (ms), Sample Count, Date (if available), Latitude, Longitude, Ext. Pressure (Pa), Ext. Altitude (m), Humidity (%), Int. Temperature (°C), Ext. Temperature (°C), Rot X (°), Rot Y (°), Compass Heading (°), Rot X Rate (°/s), Rot Y Rate (°/s), Rot Z Rate (°/s), Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), Battery Current (A), Battery Temperature (°C)
  Heading: East is negative, West is positive
*/

#include <Wire.h>
#include <SparkFunBME280.h>
#include <SparkFunLSM9DS1.h>
#include <SPI.h>
#include <SdFat.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include "sensors.h"
#include "funcs.h"

SdFat SD;


void setup() {
  pinMode(2,OUTPUT); // Error Indicator light
  pinMode(3,OUTPUT); // Extra light (GPS fix indicator? would need to add jumper between FIX pin and this pin)
  pinMode(4,OUTPUT); // Extra light
  pinMode(5,OUTPUT); // Extra light
  pinMode(10,OUTPUT); // SPI
  Serial.begin(115200);
  swSerial.begin(9600);
  Wire.begin();

  bme280_int.setI2CAddress(0x77);
  bme280_ext.setI2CAddress(0x76);
  lsm9ds1.settings.accel.scale = 4;

  errorCode += bme280_int.beginI2C() ? 0 : 10;
  errorCode += bme280_ext.beginI2C() ? 0 : 20;
  errorCode += lsm9ds1.begin() ? 0 : 1;
  errorCode += SD.begin(10) ? 0 : 100;
  Serial.println(errorCode);
  handleErrors(errorCode);
  dataFile = SD.open("OZ3test1.csv", FILE_WRITE);
  if (!dataFile) handleErrors(200);
}

void loop() {
  smartdelay(5000);

  SensorData data = measureAllSensors();
  printSensorDataCSV(data);
  // txCallSign();
}



void printSensorDataCSV(const SensorData& data) {
  dataFile.print(data.timestamp);
  dataFile.print(F(","));
  dataFile.print(data.sampleCount);
  dataFile.print(F(","));

   // dataFile.print(data.year);   //Removing date saves ~200 bytes
  // dataFile.print(F(","));      //Because the mission is < 2 hours, no date needed
  // dataFile.print(data.month);
  // dataFile.print(F(","));
  // dataFile.print(data.day);
  // dataFile.print(F(","));
  dataFile.print(data.gmtHour);
  dataFile.print(F(","));
  dataFile.print(data.gmtMin);
  dataFile.print(F(","));
  dataFile.print(data.gmtSec);
  dataFile.print(F(","));
  dataFile.print(data.latitude, 6);
  dataFile.print(F(","));
  dataFile.print(data.longitude, 6);
  dataFile.print(F(","));
  dataFile.print(data.gpsAltitude);
  dataFile.print(F(","));
  // dataFile.print(data.gpsHeading);
  // dataFile.print(F(","));
  dataFile.print(data.gpsSpeed);
  dataFile.print(F(","));

  dataFile.print(data.extPressure);
  dataFile.print(F(","));
  dataFile.print(data.extAltitude);
  dataFile.print(F(","));
  dataFile.print(data.humidity);
  dataFile.print(F(","));
  dataFile.print(data.intTemp);
  dataFile.print(F(","));
  dataFile.print(data.extTemp);
  dataFile.print(F(","));
  dataFile.print(data.attitude[0]);
  dataFile.print(F(","));
  dataFile.print(data.attitude[1]);
  dataFile.print(F(","));
  dataFile.print(data.attitude[2]);
  dataFile.print(F(","));
  dataFile.print(data.attitudeRate[0]);
  dataFile.print(F(","));
  dataFile.print(data.attitudeRate[1]);
  dataFile.print(F(","));
  dataFile.print(data.attitudeRate[2]);
  dataFile.print(F(","));
  dataFile.print(data.acceleration[0]);
  dataFile.print(F(","));
  dataFile.print(data.acceleration[1]);
  dataFile.print(F(","));
  dataFile.print(data.acceleration[2]);
  dataFile.print(F(","));
  dataFile.print(data.batteryCurrent);
  dataFile.print(F(","));
  dataFile.println(data.batteryTemp);
  dataFile.flush();
  // Serial.print(F("GPS fix: "));
  // Serial.println(data.gpsFix ? F("false") : F("true"));
  Serial.print(F("lat: "));
  Serial.println(data.latitude);
  Serial.print(F("lon: "));
  Serial.println(data.longitude);
}