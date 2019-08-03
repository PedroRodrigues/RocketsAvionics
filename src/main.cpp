#include <Arduino.h>
#include "Avionics.h"

Avionics rocket;

void setup()
{
  pinMode(DROGUE_PIN, OUTPUT);
  digitalWrite(DROGUE_PIN, HIGH);

  pinMode(MAIN_PIN, OUTPUT);
  digitalWrite(MAIN_PIN, HIGH);
  
  rocket.init();

}

void loop()
{
  
  rocket.update();

}