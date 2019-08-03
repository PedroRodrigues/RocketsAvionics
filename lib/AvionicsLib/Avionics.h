#ifndef Avionics_h
#define Avionics_h

#include "AvionicsConsts.h"

typedef struct
{
    float accelerometer[3]; //Posicoes 1,2 e 3, respectivamente sao as Aceleracoes em x,y,z
    float gyroscope[3]; //Posicoes 1, 2 e 3, respectivamente sao a velocidade angular em x,y,z
    float barometer[2]; //Posicoes 1 e 2 respectivamente sao Pressao e Temperatura
    float ina[3]; //Posicoes 1, 2 e 3 respectivamente são a Tensão, Corrente e Potencia.


    float altitudeHistory[MEMORY_SIZE]; //Memoria das ultimas altitudes.
    float filteredHistory[MEMORY_SIZE - FILTER_SIZE + 1]; //Altitudes filtradas.
    float finiteDifferences[MEMORY_SIZE - FILTER_SIZE]; //Velocidade vertical.

    float flightState; // 0 chao, 1 voo, 2 drogue, 3 main, 4 chao;

    float altitude;
    float main;
    float drogue;
} StateStruct; //State Structure

class Avionics
{
  public:
    Avionics();

    void init();
    void update();
    void debug();

    void initBMP280();
    void getBMP280(StateStruct *sBarometer);
    void initIMU();
    void getIMU(StateStruct *sImu);
    void initINA();
    void getINA(StateStruct *sIna);
    void getSensors();
    void getAltitude(StateStruct *sAltitude);

    void initFlight();
    void filterAltitudes(StateStruct *sAltitude);
    void updateAltitudes();
    void filterStates(StateStruct *sFlightState);
    void finiteDifferences();
    void detectApogge(); 
};

#endif