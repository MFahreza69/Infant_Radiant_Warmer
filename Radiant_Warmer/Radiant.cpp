#include <Arduino.h>
#include <EEPROM.h>

#include "Radiant.h"

Radiant::Radiant(){
    heater = 0;
    all_temp = 0;
}

uint8_t Radiant::get_value_heat(int set_temp, int mode, float skinTemp, float power){
    float err_skin = (set_temp*10) - (skinTemp*10);
    if(mode == 1){
        if(skinTemp == 0){
            return 0;
        }
        else{
            if(skinTemp != 0){
                if(err_skin < -0.1 && err_skin > -999){
                    return 10;
                }
                if(err_skin < 150 && err_skin > 70.5){
                    return 220;
                }
                if(err_skin < 70.5 && err_skin > 50.5){
                    return 200;
                }
                if(err_skin < 50.5 && err_skin > 40.5){
                    return 170;
                }
                if(err_skin < 40.5 && err_skin > 20.5){
                    return 150;
                }
                if(err_skin < 20.5 && err_skin > 10.5){
                    return 120;
                }
                if(err_skin < 10.5 && err_skin > 5.5){
                    return 110;
                }
                if(err_skin < 5.5 && err_skin > 1.5){
                    return 100;
                }
                if(err_skin < 1.5 && err_skin > -0.1){
                    return 92;
                }                                                                                                            
            }   
        }
    }
    if(mode == 2){
       uint8_t heatPower = map(power, 0, 10, 0, 255);
        if(power == 0){
            return 0;
        }
        if(power != 0){
            return heatPower;
        }
    }
    if(mode == 3){}
    else{return 0;}

}

float Radiant::get_value_sensor(int adcSense){
    float avr_temp = 0;
    avr_temp = adcSense;
    all_temp = ((-0.001*(pow(avr_temp, 2))) + (18*avr_temp) + 24125)/1000; 
        if(avr_temp <= 20){
          return all_temp = 0;
        }
        else{
          return all_temp;
        }
}

float Radiant::get_error_data(){
    
}