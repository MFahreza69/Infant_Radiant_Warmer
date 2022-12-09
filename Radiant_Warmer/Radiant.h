#ifndef Radiant_h
#define Radiant_h

class Radiant{
   public:
    Radiant();
    uint8_t get_value_heat(int set_temp, int mode, float skinTemp, float power);
    float get_value_sensor(int adcSense);
    float get_error_data();
   
   private:
     int heater;
     float all_temp;
};

#endif