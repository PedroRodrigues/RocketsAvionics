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

<<<<<<< HEAD
    /*void  updateAltitudes(float actState[][]);
    void  filterAltitudes(float actState[][]);
    void  finiteDifferences(float actState[][]);
    void  detectApogge(float actState[][]);    */
=======
    void  updateAltitudes(float State[][]);
    void  filterAltitudes(float State[][]);
    void  finiteDifferences(float State[][]);
    void  detectApogge(float State[][]);    
>>>>>>> 7f78603fac84adc8e9569a17826488292e070402
};

class DataFlight
{
  public:
    DataFlight();

    float prevState();
    float actState();
};



#endif