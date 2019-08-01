#include <Arduino.h>
#include <Wire.h>
#include "Avionics.h"

Avionics rocket;

void setup()
{
  
  rocket.init();

}

void loop()
{
  
  rocket.update();
  
}