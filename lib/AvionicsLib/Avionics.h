#ifndef Avionics_h
#define Avionics_h

typedef struct
{
    float accelerometer[3]; //Posicoes 1,2 e 3, respectivamente sao as Aceleracoes em x,y,z
    float gyroscope[3]; //Posicoes 1, 2 e 3, respectivamente sao a velocidade angular em x,y,z
    float barometer[2]; //Posicoes 1 e 2 respectivamente sao Pressao e Temperatura
    float ina[3]; //Posicoes 1, 2 e 3 respectivamente são a Tensão, Corrente e Potencia.


    float state;
    float altitude;

} StateStruct; //State Structure

class Avionics
{
  public:
    Avionics();

    void init();
    void update();

    void initBMP280();
    void getBMP280(StateStruct *sBarometer);
    void initIMU();
    void getIMU(StateStruct *sImu);
    void initINA();
    void getINA(StateStruct *sIna);
    void getSensors();

    void filterAltitudes(StateStruct *sAltitude);
    void updateAltitudes();
    void filterStates(StateStruct *sState);
    void finiteDifferences();
    void detectApogge(); 
    
    

};

#endif