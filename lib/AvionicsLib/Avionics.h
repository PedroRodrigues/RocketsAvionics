#ifndef Avionics_h
#define Avionics_h

class Avionics
{
  public:
    Avionics();

    void init();
    void update();

    void initBMP180();
    float getBMP180();
    void initBMP280();
    float getBMP280();
    void initIMU();
    float getIMU();
    float getSensors();

    void updateAltitudes(float actState[4][3]);
    void filterAltitudes(float actState[4][3]);
    void finiteDifferences(float actState[4][3]);
    void detectApogge(float actState[4][3]);    
};

class DataFlight
{
  public:
    DataFlight();

    float prevState();
    float actState();
};



#endif