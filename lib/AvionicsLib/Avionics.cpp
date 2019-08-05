#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <RH_RF95.h>
#include "Avionics.h"
#include "AvionicsConsts.h"



Avionics::Avionics(){};

StateStruct sActState;
StateStruct *spActState = &sActState;

Adafruit_BMP280 bmp280;
Adafruit_INA219 ina219;
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void Avionics::init()
{
  Serial.begin(9600);
  initBMP280();
  initIMU();
  initINA();
  initFlight();
}

void Avionics::update()
{
  if (millis() - sActState.stateTime > UPDATE_DELAY_STATE)
  {
    sActState.stateTime = millis();

    updateState();
    filterAltitudes();
    finiteDifferences();
    updateFlightState();
    activateServos();

    if (!DEBUG_BOARD)
    {
      boardDebug();
    }
    
    if(!DEBUG_SERIAL)
    {
      serialDebug();
    }
    
  }
  
}

void Avionics::serialDebug()
{

  Serial.print("flightState = "); Serial.print(sActState.flightState);
  Serial.print(" | AGL_Ref = "); Serial.print(sActState.AGL);
  Serial.print(" | AGL_Var = "); Serial.print(sActState.AGL_Variance);

  Serial.print(" | Alt_AGL = "); Serial.print(sActState.barometer[3]);
  Serial.print(" | Drogue = "); Serial.print(sActState.drogue);
  Serial.print(" | Main = "); Serial.print(sActState.main);


  Serial.print(" || Temperature = "); Serial.print(sActState.barometer[0]);
  Serial.print(" | Pressure = "); Serial.println(sActState.barometer[1]);
  Serial.print(" | Altitude_MSL = "); Serial.println(sActState.barometer[2]);
  //delay(500);
  Serial.print("Acel. X = "); Serial.print(sActState.accelerometer[0]);
  Serial.print(" | Y = "); Serial.print(sActState.accelerometer[1]);
  Serial.print(" | Z = "); Serial.print(sActState.accelerometer[2]);
  Serial.print(" | Gir. X = "); Serial.print(sActState.gyroscope[0]);
  Serial.print(" | Y = "); Serial.print(sActState.gyroscope[1]);
  Serial.print(" | Z = "); Serial.println(sActState.gyroscope[2]);
  //delay(250);
}

void Avionics::boardDebug()
{
  if (sActState.flightState == 0)
  {
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(LED_Y_PIN, LOW);
  digitalWrite(LED_G_PIN, HIGH);
  }
  else if (sActState.flightState == 1)
  {
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(LED_Y_PIN, HIGH);
  digitalWrite(LED_G_PIN, HIGH);
  }
  else if (sActState.flightState == 2)
  {
  digitalWrite(LED_R_PIN, HIGH);
  digitalWrite(LED_Y_PIN, HIGH);
  digitalWrite(LED_G_PIN, HIGH);
  }
  else if (sActState.flightState == 3)
  {
  digitalWrite(LED_R_PIN, HIGH);
  digitalWrite(LED_Y_PIN, HIGH);
  digitalWrite(LED_G_PIN, LOW);
  }
  else if (sActState.flightState == 4)
  {
  digitalWrite(LED_R_PIN, HIGH);
  digitalWrite(LED_Y_PIN, LOW);
  digitalWrite(LED_G_PIN, LOW);
  }
}

void Avionics::error()
{
  pinMode(LED_R_PIN,OUTPUT);
  pinMode(LED_Y_PIN,OUTPUT);
  pinMode(LED_G_PIN,OUTPUT);

  ////FATAL ERROR/////
  if(initBMP280() == 0 || initIMU()_ == 0 || initLora() == 0)
}

void Avionics::initBMP280()
{
  if (!bmp280.begin(0x76))
  {
    Serial.println(F("Could not find a BMP280 sensor."));
  }

  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X2,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X4,      /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void Avionics::getBMP280(StateStruct *sBarometer)
{
  float bmp280Temperature, bmp280Pressure, bmp280Altitude;

  bmp280Temperature = bmp280.readTemperature();
  bmp280Pressure = bmp280.readPressure();
  bmp280Altitude = bmp280.readAltitude();

  sBarometer->barometer[0] = bmp280Temperature; // temperatura celcius.
  sBarometer->barometer[1] = bmp280Pressure; // pressao hPa.
  sBarometer->barometer[2] = bmp280Altitude; // altitude MSL.
  sBarometer->barometer[3] = bmp280Altitude - sActState.AGL; // altitude AGL.
}

void Avionics::initIMU()
{
  Wire2.begin();
  Wire2.beginTransmission(IMU_ADDRESS);
  Wire2.write(0x6B);

  Wire2.write(0); 
  Wire2.endTransmission(true);
}

void Avionics::getIMU(StateStruct *sImu)
{
  float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
  float  internalTemp;




  Wire2.beginTransmission(IMU_ADDRESS);
  Wire2.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire2.endTransmission(false);
  
  //Solicita os dados do sensor
  Wire2.requestFrom(IMU_ADDRESS,14,true);  
  
  //Armazena o valor dos sensores nas variaveis correspondentes
  accelX=Wire2.read()<<8|Wire2.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  accelY=Wire2.read()<<8|Wire2.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accelZ=Wire2.read()<<8|Wire2.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  //internalTemp=Wire2.read()<<8|Wire2.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
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

  sImu->accelerometer[0] = accelX;
  sImu->accelerometer[1] = accelY;
  sImu->accelerometer[2] = accelZ;

  sImu->gyroscope[0] = gyroX;
  sImu->gyroscope[1] = gyroY;  
  sImu->gyroscope[2] = gyroZ;;
}

void Avionics::initINA()
{
  ina219.begin();
}

void Avionics::getINA(StateStruct *sIna)
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current = 0;
  float loadVoltage = 0;
  float power = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current = ina219.getCurrent_mA();
  power = ina219.getPower_mW();
  loadVoltage = busvoltage + (shuntvoltage / 1000);

  sIna->ina[0] = loadVoltage;
  sIna->ina[1] = current;
  sIna->ina[2] = power;
}

void Avionics::getSensors()
{
  getBMP280(spActState);
  getIMU(spActState);
  getINA(spActState);
}

void initLora()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
  if (!rf95.init()) 
  {
    Serial.println("LoRa radio init failed");
  }

  rf95.setTxPower(23, false);
}

void loraSend()
{
  rf95.send((uint8_t *)sActState.radioPacket, PACKET_SIZE*4);

  rf95.waitPacketSent();
}

void Avionics::initFlight()
{
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);

  pinMode(LED_R_PIN, OUTPUT);
  digitalWrite(LED_R_PIN, HIGH);

  pinMode(LED_Y_PIN, OUTPUT);
  digitalWrite(LED_Y_PIN, HIGH);

  pinMode(LED_G_PIN, OUTPUT);
  digitalWrite(LED_G_PIN, HIGH);

  delay(DEBUG_INIT_TIME);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(LED_Y_PIN, LOW);
  digitalWrite(LED_G_PIN, LOW);

  sActState.flightState = 0; // initial ground state;

  sActState.drogue = 0; // initial drogue parachute not deployed;
  sActState.main = 0; // initial main parachute not deployed;


  float AGL_Buffer[AGL_CALIBRATION_SAMPLES];

  for (int i=0; i<AGL_CALIBRATION_SAMPLES; i++)
  {
    AGL_Buffer[i] = bmp280.readAltitude();
    Serial.print("Calibrating altitude: ");
    Serial.println(AGL_Buffer[i]);
    delay(AGL_CALIBRATION_TIME * 1000 / AGL_CALIBRATION_SAMPLES);
  }

  float sum = 0;
  for (int i=0; i<AGL_CALIBRATION_SAMPLES; i++)
  {
    sum += AGL_Buffer[i];
  }
  float AGL = sum / AGL_CALIBRATION_SAMPLES;

  float variance = 0;
  for (int i=0; i<AGL_CALIBRATION_SAMPLES; i++)
  {
    variance += pow((AGL_Buffer[i] - AGL), 2) / AGL_CALIBRATION_SAMPLES;
  }

  sActState.AGL = AGL;
  sActState.AGL_Variance = variance;

  if (variance < AGL_VARIANCE_THRESHOLD)
  {
    // Soft start on ground, saving AGL do EEPROM and nominal operation.

    // COLOCAR AQUI O CODIGO PRA SALVAR O AGL NA EEPROM!! @DIEGO

  }
  else
  {
    // Hard start mid flight (hard reset), retrieving AGL from EEPROM.

    // COLOCAR AQUI O CODIGO PRA >>RECUPERAR<< O AGL NA EEPROM!! @DIEGO

  }
  sActState.stateTime = millis();
}


void Avionics::updateState()
{
  // Ler os sensores e atualizar 'sActState'.
  getSensors();

  for (int i=0; i<MEMORY_SIZE; i++)
  {
    if (i < MEMORY_SIZE - 1)
    {
      sActState.altitudeHistory[i] = sActState.altitudeHistory[i+1];
    }
    else
    {
      sActState.altitudeHistory[i] = sActState.barometer[3];
    }
  }
}

void Avionics::filterAltitudes()
{
  for (int i = 0; i < MEMORY_SIZE - FILTER_SIZE + 1; i++)
  {
    float sum = 0;
    
    for (int k = 0; k < FILTER_SIZE; k++)
    {
      sum += sActState.altitudeHistory[i+k];
    }
    sActState.filteredHistory[i] = sum / FILTER_SIZE;
  }
}

void Avionics::finiteDifferences()
{
  for (int i = 0; i < MEMORY_SIZE - FILTER_SIZE; i++)
  {
    sActState.finiteDifferences[i] = sActState.filteredHistory[i+1] - sActState.filteredHistory[i];
  }
}

void Avionics::updateFlightState()
{
  Serial.println("DEBUG:");
  Serial.println(abs(sActState.filteredHistory[MEMORY_SIZE-FILTER_SIZE]));
  //### GROUNDED AVIONICS ###
  if (sActState.flightState == 0) // ground in launch rod.
  {
    
    if (abs(sActState.filteredHistory[MEMORY_SIZE-FILTER_SIZE]) > THRESHOLD_LIFTOFF)
    {
      sActState.flightState = 1;
    }
  }

  //### IN FLIGHT ###
  else if (sActState.flightState == 1) // in flight.
  {
    float count = 0;
    for (int i = 0; i < MEMORY_SIZE - FILTER_SIZE; i++)
    {
      if (sActState.finiteDifferences[i] < 0)
      {
        count++;
      }
    }
    if (count/(MEMORY_SIZE-FILTER_SIZE) > THRESHOLD_PARACHUTE)
    {
      sActState.flightState = 2;
    }
  }

  //### APOGEE DETECTED ###
  else if (sActState.flightState == 2) // drogue deployment.
  {
    float count = 0;
    for (int i = 0; i < MEMORY_SIZE - FILTER_SIZE; i++)
    {
      if (sActState.filteredHistory[i] < THRESHOLD_MAIN)
      {
        count++;
      }
    }
    if (count/(MEMORY_SIZE-FILTER_SIZE) > THRESHOLD_PARACHUTE)
    {
      sActState.flightState = 3;
    }
  }

  //### APPROACHING GROUND ###
  else if (sActState.flightState == 3) // main deployment.
  {
    if (sActState.filteredHistory[MEMORY_SIZE - FILTER_SIZE] < THRESHOLD_TOUCHDOWN)
    {
      sActState.flightState = 4;
    }
  }

  //### TOUCHDOWN ###
  else if (sActState.flightState == 4)
  {

  }
}

void Avionics::activateServos()
{
  if (sActState.flightState == 0)
  {
  sActState.drogue = 0;
  sActState.main = 0;
  digitalWrite(DROGUE_PIN, HIGH);
  digitalWrite(MAIN_PIN, HIGH);
  }

  else if (sActState.flightState == 1)
  {
  sActState.drogue = 0;
  sActState.main = 0;
  digitalWrite(DROGUE_PIN, HIGH);
  digitalWrite(MAIN_PIN, HIGH);
  }

  else if (sActState.flightState == 2)
  {
      // AQUI FALTA O CODIGO PRA USAR DROGUE_TIME_MS PRA GERAR O PULSO

  sActState.drogue = 1;
  sActState.main = 0;
  digitalWrite(DROGUE_PIN, LOW);
  digitalWrite(MAIN_PIN, HIGH);
  }

  else if (sActState.flightState == 3)
  {
      // AQUI FALTA O CODIGO PRA USAR MAIN_TIME_MS PRA GERAR O PULSO

  sActState.drogue = 1;
  sActState.main = 1;
  digitalWrite(DROGUE_PIN, HIGH);
  digitalWrite(MAIN_PIN, LOW);
  }


  else if (sActState.flightState == 4)
  {
  sActState.drogue = 1;
  sActState.main = 1;
  digitalWrite(DROGUE_PIN, HIGH);
  digitalWrite(MAIN_PIN, HIGH);
  }
}