#ifndef Avionics_h
#define Avionics_h

class Avionics
{
  public:
    Avionics();

    void  init();
    void  update();

    void  initBMP180();
    float getBMP180();
    void  initBMP280();
    float getBMP280();
    void  initIMU();
    float getIMU();
    void  initINA();
    float getINA();
    float getSensors();

    void  updateAltitudes(float State[][]);
    void  filterAltitudes(float State[][]);
    void  finiteDifferences(float State[][]);
    void  detectApogge(float State[][]);    
};

class DataFlight
{
  public:
    DataFlight();

    float prevState();
    float actState();
};



#endif