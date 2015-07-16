
#ifndef BMP180_H
#define BMP180_H

#include <software_i2c.h>

#define ADDRESS 0xEE

#define AC1_MSB 0xAA
#define AC1_LSB 0xAB
#define AC2_MSB 0xAC
#define AC2_LSB 0xAD
#define AC3_MSB 0xAE
#define AC3_LSB 0xAF
#define AC4_MSB 0xB0
#define AC4_LSB 0xB1
#define AC5_MSB 0xB2
#define AC5_LSB 0xB3
#define AC6_MSB 0xB4
#define AC6_LSB 0xB5
#define B1_MSB 0xB6
#define B1_LSB 0xB7
#define B2_MSB 0xB8
#define B2_LSB 0xB9
#define MB_MSB 0xBA
#define MB_LSB 0xBB
#define MC_MSB 0xBC
#define MC_LSB 0xBD
#define MD_MSB 0xBE
#define MD_LSB 0xBF

#define CONTROL_REGISTER 0xF4

#define TEMPERATURE_MEASURE  0x2E
#define PRESSURE_MEASURE 0x34

#define REGISTER_MSB  0xF6
#define REGISTER_LSB  0xF7
#define REGISTER_XLSB 0xF8


class BMP180
{
public:
  BMP180(Software_I2C* i2c_bus);
  char getCalibrationData();
  char getData();
  
  int getTemperature();
  int getTemperatureAge();
  long getPressure();
  int getPressureAge();

private:

  Software_I2C* _i2c_bus;
  
  unsigned long temperatureTime;
  unsigned long pressureTime;
  
  unsigned long rawTemperature;
  unsigned long rawPressure;
  
  long cal_AC1;
  long cal_AC2;
  long cal_AC3;
  unsigned long cal_AC4;
  unsigned long cal_AC5;
  unsigned long cal_AC6;
  long cal_B1;
  long cal_B2;
  long cal_MB;
  long cal_MC;
  long cal_MD;
  
};


#endif