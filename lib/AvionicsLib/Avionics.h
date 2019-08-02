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

    /*void  updateAltitudes(float actState[][]);
    void  filterAltitudes(float actState[][]);
    void  finiteDifferences(float actState[][]);
    void  detectApogge(float actState[][]);    */
};

class DataFlight
{
  public:
    DataFlight();

    float prevState();
    float actState();
};



#endif