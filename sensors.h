#ifndef SENSORS_H
#define SENSORS_H

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
  attitude[0] = atan2(lsm9ds1.ay, lsm9ds1.az) * 180.0 / PI;
  attitude[1] = atan2(-lsm9ds1.ax, sqrt(lsm9ds1.ay * lsm9ds1.ay + lsm9ds1.az * lsm9ds1.az)) * 180.0 / PI;
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

  float sensitivity = -90.0 / 770.0; //90mA per 770mV. Inverted
  float Vref = 1860; // Output voltage with no current: ~ 1950mV

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