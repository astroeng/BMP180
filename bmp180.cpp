#include <bmp180.h>
#include <Arduino.h>
#include <software_i2c.h>

#define SAMPLE_DELAY 8 /* ms */

/* Test macro uncomment the 'x' to run the code in test mode. It will use the 
   default values from the datasheet instead of the ones from the device. This
   can also be used to see how this code works if you do not have an BMP180 or
   compatable device.
 */
#define TEST(x) //x


unsigned char calibrationAddress[] = 
{
  AC1_MSB,
  AC1_LSB,
  AC2_MSB,
  AC2_LSB,
  AC3_MSB,
  AC3_LSB,
  AC4_MSB,
  AC4_LSB,
  AC5_MSB,
  AC5_LSB,
  AC6_MSB,
  AC6_LSB,
  B1_MSB,
  B1_LSB,
  B2_MSB,
  B2_LSB,
  MB_MSB,
  MB_LSB,
  MC_MSB,
  MC_LSB,
  MD_MSB,
  MD_LSB
};

BMP180::BMP180(Software_I2C* i2c_bus)
{
  _i2c_bus = i2c_bus;
  
  temperatureTime = 0;
  pressureTime = 0;
  
  rawTemperature = 0;
  rawPressure = 0;
  
  cal_AC1 = 1;
  cal_AC2 = 1;
  cal_AC3 = 1;
  cal_AC4 = 1;
  cal_AC5 = 1;
  cal_AC6 = 1;
  cal_B1  = 1;
  cal_B2  = 1;
  cal_MB  = 1;
  cal_MC  = 1;
  cal_MD  = 1;

}

char BMP180::begin()
{
  int looper = 0;
  char error = 0;
  unsigned int temp[22];

  /* Initiate the transfer of the calibration values. */
  _i2c_bus->start_i2c();
  _i2c_bus->write(ADDRESS);
  _i2c_bus->write(AC1_MSB);
  _i2c_bus->stop_i2c();
  
  /* Start the read of the calibration values. */
  _i2c_bus->start_i2c();
  _i2c_bus->write(ADDRESS | 0x01);
  
  /* Loop through the calibration values and save off the bytes. */
  for (looper = 0; looper < 21; looper++)
  {
    temp[looper] = _i2c_bus->read(&error);
    if (error != NO_ERROR)
    {
      return error;
    }
  }
  
  /* For the last one send the NACK signal so the device knows to stop
     sending.
   */
  temp[looper] = _i2c_bus->read(&error, I2C_HIGH);
  
  _i2c_bus->stop_i2c();
  
  /* Save the calibration values. The int cast is needed to correctly save the
     signed values into the longer types.
   */
  cal_AC1 = (int)((temp[0]  << 8) | temp[1]);
  cal_AC2 = (int)((temp[2]  << 8) | temp[3]);
  cal_AC3 = (int)((temp[4]  << 8) | temp[5]);
  cal_AC4 = (temp[6]  << 8) | temp[7];
  cal_AC5 = (temp[8]  << 8) | temp[9];
  cal_AC6 = (temp[10] << 8) | temp[11];
  cal_B1  = (int)((temp[12] << 8) | temp[13]);
  cal_B2  = (int)((temp[14] << 8) | temp[15]);
  cal_MB  = (int)((temp[16] << 8) | temp[17]);
  cal_MC  = (int)((temp[18] << 8) | temp[19]);
  cal_MD  = (int)((temp[20] << 8) | temp[21]);
 
  /* Test code for validating the math in this unit. */
  TEST(cal_AC1 =    408);
  TEST(cal_AC2 =    -72);
  TEST(cal_AC3 = -14383);
  TEST(cal_AC4 =  32741);
  TEST(cal_AC5 =  32757);
  TEST(cal_AC6 =  23153);
  TEST(cal_B1  =   6190);
  TEST(cal_B2  =      4);
  TEST(cal_MB  = -32768);
  TEST(cal_MC  =  -8711);
  TEST(cal_MD  =   2868);
  
  TEST(Serial.println("+CAL VALUES"));
  TEST(Serial.println(cal_AC1));
  TEST(Serial.println(cal_AC2));
  TEST(Serial.println(cal_AC3));
  TEST(Serial.println(cal_AC4));
  TEST(Serial.println(cal_AC5));
  TEST(Serial.println(cal_AC6));
  TEST(Serial.println(cal_B1));
  TEST(Serial.println(cal_B2));
  TEST(Serial.println(cal_MB));
  TEST(Serial.println(cal_MC));
  TEST(Serial.println(cal_MD));
  TEST(Serial.println("-CAL VALUES"));
  
  return NO_ERROR;
}

char BMP180::run()
{
  char error;
  unsigned int temp[2];
  
  _i2c_bus->start_i2c();
  _i2c_bus->write(ADDRESS);
  _i2c_bus->write(CONTROL_REGISTER);
  _i2c_bus->write(TEMPERATURE_MEASURE);
  _i2c_bus->stop_i2c();
  
  delay(SAMPLE_DELAY);
  
  _i2c_bus->start_i2c();
  _i2c_bus->write(ADDRESS);
  _i2c_bus->write(REGISTER_MSB);
  _i2c_bus->stop_i2c();
  
  _i2c_bus->start_i2c();
  _i2c_bus->write(ADDRESS | 0x01);
  temp[0] = _i2c_bus->read(&error);
  temp[1] = _i2c_bus->read(&error, I2C_HIGH);
  _i2c_bus->stop_i2c();
  
  rawTemperature = (temp[0] << 8) | temp[1];
  temperatureTime = millis();
  
  /************ PRESSURE ************/
  
  _i2c_bus->start_i2c();
  _i2c_bus->write(ADDRESS);
  _i2c_bus->write(CONTROL_REGISTER);
  _i2c_bus->write(PRESSURE_MEASURE);
  _i2c_bus->stop_i2c();
  
  delay(SAMPLE_DELAY);
  
  _i2c_bus->start_i2c();
  _i2c_bus->write(ADDRESS);
  _i2c_bus->write(REGISTER_MSB);
  _i2c_bus->stop_i2c();
  
  _i2c_bus->start_i2c();
  _i2c_bus->write(ADDRESS | 0x01);
  temp[0] = _i2c_bus->read(&error);
  temp[1] = _i2c_bus->read(&error, I2C_HIGH);
  _i2c_bus->stop_i2c();
  
  rawPressure = (temp[0] << 8) | temp[1];
  rawPressure = millis();
  
}

int BMP180::getTemperature()
{
  long temperature;

  TEST(Serial.println("TEMPERATURE"));
  TEST(rawTemperature = 27898);
  
  long val_X1 = (rawTemperature - cal_AC6) * cal_AC5 / 32768;
  long val_X2 = (cal_MC * 2048) / (val_X1 + cal_MD);
  long val_B5 = val_X1 + val_X2;
  
  temperature = (val_B5 + 8) / 16;
  
  return temperature;
}

int BMP180::getTemperatureAge()
{
  return 0;
}

long BMP180::getPressure()
{
  int oss_amt = 0;
  long pressure;
  
  long val_X1, val_X2, val_X3;
  long val_B3, val_B5, val_B6;
  unsigned long val_B4, val_B7;
  
  TEST(Serial.println("PRESSURE"));
  TEST(rawPressure = 23843);
  TEST(Serial.println(rawPressure));
  TEST(Serial.println(rawTemperature));
  
  val_X1 = ((rawTemperature - cal_AC6) * cal_AC5) / 32768;
  TEST(Serial.println(val_X1));
  val_X2 = (cal_MC * 2048) / (val_X1 + cal_MD);
  TEST(Serial.println(val_X2));
  val_B5 = val_X1 + val_X2;
  TEST(Serial.println(val_B5));
  val_B6 = val_B5 - 4000;
  TEST(Serial.println(val_B6));
  
  val_X1 = (cal_B2 * ((val_B6 * val_B6) / 4096)) / 2048;
  TEST(Serial.println(val_X1));
  val_X2 = (cal_AC2 * val_B6) / 2048;
  TEST(Serial.println(val_X2));
  val_X3 = val_X1 + val_X2;
  TEST(Serial.println(val_X3));
  val_B3 = (((cal_AC1 * 4) + val_X3) + 2) / 4; 
  TEST(Serial.println(val_B3));
  
  val_X1 = cal_AC3 * val_B6 / 8192;
  TEST(Serial.println(val_X1));
  val_X2 = (cal_B1 * ((val_B6 * val_B6) / 4096)) / 65536;
  TEST(Serial.println(val_X2));
  val_X3 = (val_X1 + val_X2 + 2) / 4;
  TEST(Serial.println(val_X3));
  val_B4 = (cal_AC4 * (val_X3 + 32768)) / 32768;
  TEST(Serial.println(val_B4));
  val_B7 = (rawPressure - val_B3) * (50000);
  TEST(Serial.println(val_B7));
  
  if (val_B7 < 0x80000000)
  {
    pressure = (val_B7 * 2) / val_B4;
  }
  else 
  {
    pressure = (val_B7 / val_B4) * 2;
  }
  
  TEST(Serial.println(pressure));
  
  val_X1 = (pressure / 256) * (pressure / 256);
  TEST(Serial.println(val_X1));
  val_X1 = (val_X1 * 3038) / 65536;
  TEST(Serial.println(val_X1));
  val_X2 = (-7357 * pressure) / 65536;
  TEST(Serial.println(val_X2));
  pressure = pressure + ((val_X1 + val_X2 + 3791) / 16);
  
  return pressure;
}

int BMP180::getPressureAge()
{
  return 0;
}