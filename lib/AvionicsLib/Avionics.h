#ifndef Avionics_h
#define Avionics_h

class Avionics
{
  public:
    Avionics();

    void init();
    void update();

    void initBMP180();
    char getBMP180();
    void initBMP280();
    char getBMP280();
    void initIMU();
    char getIMU();
    char getSensors();

    void updateAltitudes();
    void filterAltitudes();
    void finiteDifferences();
    void detectApogge();

};



#endif