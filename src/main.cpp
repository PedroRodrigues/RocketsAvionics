#include <Arduino.h>
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