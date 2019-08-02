#ifndef Avionics_h
#define Avionics_h

typedef struct
{
    float acelerometro[3]; //Posicoes 1,2,3, respectivamente sao as Aceleracoes em x,y,z
    float magnetometro[3]; //Posicoes 1,2,3, respectivamente sao as Campos Magneticos em x,y,z
    float giroscopio[3]; //Posicoes 1, 2, 3, respectivamente sao a velocidade angular em x,y,z
    float barometro[3]; //Posicoes 1,2,3 respectivamente sao Pressao, Altura e Temperatura
}IMU_s; //IMU Structure

class Avionics
{
  public:
    Avionics();

    void  init();
    void  update();

    void  initBMP280();
    void getBMP280(IMU_s *imu);
    void  initIMU();
    void  getIMU(IMU_s *imu);
    void  initINA();
    float getINA();
    float getSensors();
    float ReturnAccel[3];
    float ReturnGyro[3];

    //void  updateAltitudes(float State[][]);
    //void  filterAltitudes(float State[][]);
    //void  finiteDifferences(float State[][]);
    //void  detectApogge(float State[][]); 
    
    

};

class DataFlight
{
  public:
    DataFlight();

    float prevState();
    float actState();
};



#endif