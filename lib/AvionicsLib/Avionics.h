#ifndef Avionics_H
#define Avionics_H

class Avionics
{
  public:

    void init();
    void update();

    char getBMP280();
    char getBMP180();
    char getIMU();
    char getSensors();

    void updateAltitudes();
    void filterAltitudes();
    void finiteDifferences();
    void detectApogge();

};



#endif