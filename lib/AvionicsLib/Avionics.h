#ifndef Avionics_h
#define Avionics_h

#include "AvionicsConsts.h"

typedef struct
{
  // atualizado pelo getSensors();
  float accelerometer[3] = {0}; //Posicoes 1,2 e 3, respectivamente sao as Aceleracoes em x,y,z
  float gyroscope[3] = {0}; //Posicoes 1, 2 e 3, respectivamente sao a velocidade angular em x,y,z
  float barometer[4] = {0}; //respectivamente sao temperatura, pressao, altitude MSL e AGL.
  float ina[3] = {0}; //Posicoes 1, 2 e 3 respectivamente são a Tensão, Corrente e Potencia.
  // fim do getSensors();

  float radioPacket[PACKET_SIZE] = {0};

  float altitudeHistory[MEMORY_SIZE] = {0}; //Memoria das ultimas altitudes.
  float filteredHistory[MEMORY_SIZE - FILTER_SIZE + 1] = {0}; //Altitudes filtradas.
  float finiteDifferences[MEMORY_SIZE - FILTER_SIZE] = {0}; //Velocidade vertical.

  float flightState; // 0 chao, 1 voo, 2 drogue, 3 main, 4 chao;

  float AGL; // Above Ground Level - altitude reference for 'ground';
  float AGL_Variance;

  float main;
  unsigned long long mainTime;

  float drogue;
  unsigned long long drogueTime;

  unsigned long long stateTime;

} StateStruct; //State Structure

class Avionics
{
  public:
    Avionics();

    void init();
    void update();
    void serialDebug();

    void boardDebug();
    void error();

    void initBMP280();
    void getBMP280(StateStruct *sBarometer);
    void initIMU();
    void getIMU(StateStruct *sImu);
    void initINA();
    void getINA(StateStruct *sIna);
    void getSensors();
    void initLora();
    void loraSend(StateStruct *sRadioPackage);

    void initFlight();
    void updateState();
    void filterAltitudes();
    void finiteDifferences();
    void updateFlightState();
    void activateServos(); 
};

#endif