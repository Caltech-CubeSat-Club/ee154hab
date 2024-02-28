#ifndef SENSORS_H
#define SENSORS_H

BME280 bme280_int;
BME280 bme280_ext;
LSM9DS1 lsm9ds1;

TinyGPSPlus gps;
SoftwareSerial swSerial(8, 9);

int sampleNumber = 0;

struct SensorData {
  unsigned long timestamp = 0;
  int sampleCount = 0;
  //String date = "";
  String gmtTime = "";
  float latitude = 0;
  float longitude = 0;
  float gpsAltitude = 0;
  float gpsHeading = 0;
  float gpsSpeed = 0;
  bool gpsFix = false;
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


/*    ACCELEROMETER/GYRO/MAGNETOMETER FUNCTIONS       */
/*    ACCELEROMETER/GYRO/MAGNETOMETER FUNCTIONS       */
/*    ACCELEROMETER/GYRO/MAGNETOMETER FUNCTIONS       */
float getCompassHeading() {
  float heading;
  if (lsm9ds1.my == 0)
    heading = (lsm9ds1.mx < 0) ? PI : 0;
  else
    heading = atan2(lsm9ds1.mx, lsm9ds1.my);

  heading -= 11.4833 * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);

  heading *= 180.0 / PI;
  return heading;
}

void getAttitude(float* attitude) {
  float ax = lsm9ds1.ax;
  float ay = lsm9ds1.ay;
  float az = lsm9ds1.az;
  
  attitude[0] = atan2(ay, az) * 180.0 / PI;
  
  
  attitude[1] = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  attitude[2] = getCompassHeading();
}

void getAttitudeRate(float* attitudeRate) {
  attitudeRate[0] = lsm9ds1.calcGyro(lsm9ds1.gx);
  attitudeRate[1] = lsm9ds1.calcGyro(lsm9ds1.gy);
  attitudeRate[2] = lsm9ds1.calcGyro(lsm9ds1.gz);
}

void getAcceleration(float* acceleration) {
  acceleration[0] = lsm9ds1.calcAccel(lsm9ds1.ax);
  acceleration[1] = lsm9ds1.calcAccel(lsm9ds1.ay);
  acceleration[2] = lsm9ds1.calcAccel(lsm9ds1.az);
}

void getNewLSMdata() {
  if (lsm9ds1.gyroAvailable())
    lsm9ds1.readGyro();
  if (lsm9ds1.accelAvailable())
    lsm9ds1.readAccel();
  if (lsm9ds1.magAvailable())
    lsm9ds1.readMag();
}

void readIMUdata(SensorData& data) {
  getNewLSMdata();
  getAttitude(data.attitude);
  getAttitudeRate(data.attitudeRate);
  getAcceleration(data.acceleration);
}
/*     END ACCELEROMETER/GYRO/MAGNETOMETER FUNCTIONS       */
/*     END ACCELEROMETER/GYRO/MAGNETOMETER FUNCTIONS       */
/*     END ACCELEROMETER/GYRO/MAGNETOMETER FUNCTIONS       */


/*    BME280 FUNCTIONS     */
/*    BME280 FUNCTIONS     */
/*    BME280 FUNCTIONS     */
void readBME280Data(SensorData& data) {
  data.extPressure = bme280_ext.readFloatPressure(); // Pressure in Pa
  data.extAltitude = bme280_ext.readFloatAltitudeMeters(); // Altitude in meters
  data.humidity = bme280_int.readFloatHumidity(); // Humidity in %
  data.intTemp = bme280_int.readTempC(); // Internal temperature in Celsius
  data.extTemp = bme280_ext.readTempC(); // External temperature in Celsius
}
/*    END BME280 FUNCTIONS     */
/*    END BME280 FUNCTIONS     */
/*    END BME280 FUNCTIONS     */


/*    GPS FUNCTIONS     */
/*    GPS FUNCTIONS     */
/*    GPS FUNCTIONS     */
void readGPSData(SensorData& data) {
  //data.date = String(gps.date.year()) + String(gps.date.month()) + String(gps.date.day());
  data.gmtTime = String(gps.time.hour()) + String(gps.time.minute()) + String(gps.time.second());
  data.latitude = gps.location.lat();
  data.longitude = gps.location.lng();
  data.gpsAltitude = gps.altitude.meters();
  data.gpsHeading = gps.course.deg();
  data.gpsSpeed = gps.speed.kmph();
}

static void smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (swSerial.available())
      gps.encode(swSerial.read());
  } while (millis() - start < ms);
}
/*    END GPS FUNCTIONS     */
/*    END GPS FUNCTIONS     */
/*    END GPS FUNCTIONS     */


/*    CURRENT SENSOR FUNCTIONS     */
/*    CURRENT SENSOR FUNCTIONS     */
/*    CURRENT SENSOR FUNCTIONS     */
void readBatteryCurrent(SensorData& data) {
  // Number of samples to average the reading over
  // Change this to make the reading smoother... but beware of buffer overflows!
  const int avgSamples = 20;

  int sensorValue = 0;

  float sensitivity = -90.0 / 80.0; //90mA per 80 mV
  float Vref = 2450; // Output voltage with no current: ~ 2500mV or 2.5V

  for (int i = 0; i < avgSamples; i++)
  {
    sensorValue += analogRead(A0);

    // wait 2 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    // DOES DELAY, SO IF OTHER STUFF STOPS WHILE SENSING CURRENT, IT'S BECAUSE OF THIS
    delay(2);

  }

  sensorValue = sensorValue / avgSamples;

  // The on-board ADC is 10-bits -> 2^10 = 1024 -> 5V / 1024 ~= 4.88mV
  // The voltage is in millivolts
  float voltage = 4.88 * sensorValue;

  // This will calculate the actual current (in mA)
  // Using the Vref and sensitivity settings you configure
  data.batteryCurrent = (voltage - Vref) * sensitivity;
}
/*    END CURRENT SENSOR FUNCTIONS     */
/*    END CURRENT SENSOR FUNCTIONS     */
/*    END CURRENT SENSOR FUNCTIONS     */


/*    THERMISTOR SENSOR FUNCTIONS     */
/*    THERMISTOR SENSOR FUNCTIONS     */
/*    THERMISTOR SENSOR FUNCTIONS     */
void readBatteryTemp(SensorData& data) {
  uint8_t i;
  float average;
  float samples[5];

  // take N samples in a row, with a slight delay
  for (i=0; i < 5; i++) {
   samples[i] = analogRead(A1);
   delay(10);
  }
  
  // average all the samples out
  average = 0;
  for (i=0; i< 5; i++) {
     average += samples[i];
  }
  average /= 5;
  
  // convert the value to resistance
  average = 1023 / average - 1;
  average = 100000 / average; // 100 kOhm resistor is used

  float steinhart;
  steinhart = average / 12200;     // (R/Ro) Ro is resistance at a reference temp.
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= 3950;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (20.5556 + 273.15); // + (1/To) To is lab temperature 20.5556C
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C
  
  data.batteryTemp = steinhart;

}
/*    END THERMISTOR SENSOR FUNCTIONS     */
/*    END THERMISTOR SENSOR FUNCTIONS     */
/*    END THERMISTOR SENSOR FUNCTIONS     */


SensorData measureAllSensors() {
  SensorData data;
  
  data.timestamp = millis();
  
  data.sampleCount = sampleNumber++;
  
  readGPSData(data);

  readBME280Data(data);

  readIMUdata(data);

  readBatteryCurrent(data);
  readBatteryTemp(data);

  return data;
}


#endif