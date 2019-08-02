#include <Arduino.h>
#include <Wire.h>
#include "Avionics.h"

IMU_s imu_struct;
IMU_s *imu_pstruct = &imu_struct;


Avionics rocket;

void setup()
{
  
  rocket.init();

}

void loop()
{

  rocket.update();
  rocket.getIMU(imu_pstruct);
  rocket.getBMP280(imu_pstruct);

  //Serial.print("AccelX in struct :");Serial.println(imu_struct.acelerometro[0]);
  //Serial.print("AccelY in struct :");Serial.println(imu_struct.acelerometro[1]);
  Serial.print("Pressure in struct :");Serial.println(imu_struct.barometro[1]);
}