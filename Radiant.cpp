#include <Arduino.h>
#include <EEPROM.h>

#include "Radiant.h"

Radiant::Radiant(){
    heater = 0;
    all_temp = 0;
}

float Radiant::get_value_heat(int set_temp, int mode, float skinTemp, float power){
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
                if(err_skin < 150 && err_skin >= 70.5){
                    return 255;
                }
                if(err_skin < 70.5 && err_skin >= 50.5){
                    return 218;
                }
                if(err_skin < 50.5 && err_skin >= 40.5){
                    return 200;
                }
                if(err_skin < 40.5 && err_skin >= 20.5){
                    return 190;
                }
                if(err_skin < 20.5 && err_skin >= 10.5){
                    return 180;
                }
                if(err_skin < 10.5 && err_skin >= 5.5){
                    return 170;
                }
                if(err_skin < 5.5 && err_skin >= 1.5){
                    return 160;
                }
                if(err_skin < 1.5 && err_skin >= -0.1){
                    return 150;
                }                                                                                                            
            }   
        }
    }
    if(mode == 2){
        if(power ==  0){return 0;}
        if(power ==  1){return 11.25;}
        if(power ==  2){return 30.6;}
        if(power ==  3){return 58.6;}
        if(power ==  4){return 81.6;}
        if(power ==  5){return 103.9;}
        if(power ==  6){return 127.1;}
        if(power ==  7){return 149.52;}
        if(power ==  8){return 183.56;}
        if(power ==  9){return 218.2;}
        if(power == 10){return 255;}                                        
    }
    if(mode == 3){
        if(skinTemp < 29){
            return 183.56;
        }
        if(skinTemp < 33.9 && skinTemp >= 29){
            return 149.52;
        }
        if(skinTemp >= 33.9){
            return 127.1;
        }
    }
    else{
        return 0;
        }

}

float Radiant::get_value_sensor(int adcSense, int whichSensor){
   float avr_temp = 0;
   if(whichSensor == 0){ 
     avr_temp = adcSense;
     all_temp = ((-0.001*(pow(avr_temp, 2))) + (18*avr_temp) + 24125)/1000; 
     if(avr_temp <= 20){
        return all_temp = 0;
     }
     else{
        return all_temp;
     }
   }

   if(whichSensor == 1){
     avr_temp = adcSense;
     all_temp = (0.0269*avr_temp) + 18.426;
    //  all_temp = ((0.007*(pow(avr_temp, 2)))+(13.9*avr_temp) + 19801)/1000;
     if(avr_temp <= 20){
       return all_temp = 0;
     }
     else{
       return all_temp;
     }
   }
}

float Radiant::get_error_data(){
    
}