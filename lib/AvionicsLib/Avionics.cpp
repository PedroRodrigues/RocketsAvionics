#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include "Avionics.h"
#include "AvionicsConsts.h"

Avionics::Avionics()  {};

Adafruit_BMP280 bmp280;
Adafruit_INA219 ina219;

void Avionics::init()
{
  Serial.begin(9600);
  initBMP180();
  initBMP280();
  initIMU();
  initINA();

}

void Avionics::update()
{

}

void Avionics::initBMP180()
{

}
///////////////////////////////DIEGO///////////////////////////////
float Avionics::getBMP180()
{

}

void Avionics::initBMP280()
{
  if (!bmp280.begin(0x76))
  {
    Serial.println(F("Could not find a BMP280 sensor."));
  }
  
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
///////////////////////////////DIEGO///////////////////////////////
float Avionics::getBMP280()
{
  char bmp280Data[2];
  float bmp280Temperature, bmp280Pressure;

  bmp280Data[0] = bmp280.readTemperature();
  bmp280Pressure = bmp280.readPressure();



  return *bmp280Data;
}

void Avionics::initIMU()
{
  Wire2.begin();
  Wire2.beginTransmission(MPU_ADDRESS);
  Wire2.write(0x6B);

  Wire2.write(0); 
  Wire2.endTransmission(true);

}
///////////////////////////////DIEGO///////////////////////////////
float Avionics::getIMU()
{
  int accelX,accelY,accelZ,internalTemp,gyroX,gyroY,gyroZ;
  float returnAccelArray[3], returnGyroArray[3];

  Wire2.beginTransmission(MPU_ADDRESS);
  Wire2.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire2.endTransmission(false);
  
  //Solicita os dados do sensor
  Wire2.requestFrom(MPU_ADDRESS,14,true);  
  
  //Armazena o valor dos sensores nas variaveis correspondentes
  accelX=Wire2.read()<<8|Wire2.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  accelY=Wire2.read()<<8|Wire2.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accelZ=Wire2.read()<<8|Wire2.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  internalTemp=Wire2.read()<<8|Wire2.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX=Wire2.read()<<8|Wire2.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY=Wire2.read()<<8|Wire2.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ=Wire2.read()<<8|Wire2.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  returnAccelArray[0] = accelX;
  returnAccelArray[1] = accelY;
  returnAccelArray[2] = accelZ;

  returnGyroArray[0] = gyroX;
  returnGyroArray[1] = gyroY;
  returnGyroArray[2] = gyroZ;

  /*
  //Mostra os valores na serial
  Serial.print("Acel. X = "); Serial.print(AcX);
  Serial.print(" | Y = "); Serial.print(AcY);
  Serial.print(" | Z = "); Serial.print(AcZ);
  Serial.print(" | Gir. X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.print(GyZ);
  Serial.print(" | Temp = "); Serial.println(Tmp/340.00+36.53);
  */

  return returnAccelArray, returnGyroArray;

}
///////////////////////////////DIEGO///////////////////////////////
float Avionics::getSensors()
{
  float bmp180[3], bmp280[3], IMU[6], returnSensors[4][3];
  

  bmp180[] = getBMP180();
}

void Avionics::initINA()
{
  uint32_t currentFrequency;

  if (!(ina219.begin()))
  {
    Serial.println(F("Could not find a INA219 sensor."));
  };

}

float Avionics::getINA()
{
  float shuntVoltage;
  float busVoltage;
  float current_mA;
  float loadVoltage;
  float power_mW;

  shuntVoltage = ina219.getShuntVoltage_mV();
  busVoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadVoltage = busVoltage + (shuntVoltage / 1000);
  
  
}

void Avionics::updateAltitudes(float actState[20])
{

}

void Avionics::filterAltitudes(float actState[20])
{

}

void Avionics::finiteDifferences(float actState[20])
{

}

void Avionics::detectApogge(float actState[20])
{

}

