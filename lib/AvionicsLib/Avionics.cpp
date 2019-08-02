#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "Avionics.h"
#include "AvionicsConsts.h"




Avionics::Avionics()  {};

Adafruit_BMP280 bmp280;
Adafruit_INA219 ina219;                

int accelX,accelY,accelZ,internalTemp,gyroX,gyroY,gyroZ;

//const int chipSelect = BUILTIN_SDCARD;       // Teensy 3.5 & 3.6 on-board

void Avionics::init()
{
  Serial.begin(9600);
  initBMP280();
  initIMU();
}

void Avionics::update()
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
void Avionics::getBMP280(IMU_s *imu)
{
  char bmp280Data[2];
  float bmp280Temperature, bmp280Pressure;

  bmp280Data[0] = bmp280.readTemperature();
  bmp280Pressure = bmp280.readPressure();

  imu->barometro[0] =  bmp280Data[0];
  imu->barometro[1] =  bmp280Pressure;
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
void Avionics::getIMU(IMU_s *imu)
{
  float accelX,accelY,accelZ,internalTemp,gyroX,gyroY,gyroZ;
  //float returnAccelArray[3], returnGyroArray[3];

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
   
  
  //Mostra os valores na serial
  /*Serial.print("Acel. X = "); Serial.print(accelX);
  Serial.print(" | Y = "); Serial.print(accelY);
  Serial.print(" | Z = "); Serial.print(accelZ);
  Serial.print(" | Gir. X = "); Serial.print(gyroX);
  Serial.print(" | Y = "); Serial.print(gyroY);
  Serial.print(" | Z = "); Serial.print(gyroZ);
  Serial.print(" | Temp = "); Serial.println(internalTemp/340.00+36.53);
*/
  //Serial.print("AccelX in fuction :");Serial.println(accelX);
  //Serial.print("AccelY in fuction :");Serial.println(accelY);
  imu->acelerometro[0] = accelX;
  imu->acelerometro[1] = accelY;
  imu->acelerometro[2] = accelZ;

  imu->giroscopio[0] = gyroX;
  imu->giroscopio[1] = gyroY;  
  imu->giroscopio[2] = gyroZ;;
}
