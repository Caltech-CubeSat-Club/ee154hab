#ifndef DEFS_H
#define DEFS_H

File dataFile;
BME280 bme280_int;
BME280 bme280_ext;
LSM9DS1 lsm9ds1;

SoftwareSerial gps(8, 9);
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

int errorCode = 0;
int sampleNumber = 0;

struct SensorData {
  unsigned long timestamp = 0;
  int sampleCount = 0;
  String gps = "";
  float extPressure = 0;
  float extAltitude = 0;
  float humidity = 0;
  float intTemp = 0;
  float extTemp = 0;
  float attitude[3];      // Roll, Pitch, CompassHeading
  float attitudeRate[3];  // RollRate, PitchRate, YawRate
  float acceleration[3];  // AccelX, AccelY, AccelZ
  float batteryCurrent = 0;
  float batteryTemp = 0;
};


#endif