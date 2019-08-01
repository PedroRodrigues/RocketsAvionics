#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_BMP280.h"
#include "Avionics.h"

Avionics::Avionics()  {};

Adafruit_BMP280 bmp280;


void Avionics::init()
{
  Serial.begin(9600);
  initBMP180();
  initBMP280();
  initIMU();

}

void Avionics::update()
{

}

void Avionics::initBMP180()
{

}

char Avionics::getBMP180()
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

char Avionics::getBMP280()
{
  char bmp280Data;
  float bmp280Temperature, bmp280Pressure;

  bmp280Temperature = bmp280.readTemperature();
  bmp280Pressure = bmp280.readPressure();
  
  return bmp280Data;
}

void Avionics::initIMU()
{

}

char Avionics::getIMU()
{

}

char Avionics::getSensors()
{

}

void Avionics::updateAltitudes()
{

}

void Avionics::filterAltitudes()
{

}

void Avionics::finiteDifferences()
{

}

void Avionics::detectApogge()
{

}

