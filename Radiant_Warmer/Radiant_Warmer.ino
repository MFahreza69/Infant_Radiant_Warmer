/*Library Include*/
#include "Radiant.h"
#include <RTClib.h>
#include <PID_v1.h>
#include <ArduinoJson.h>
#include "src/SimpleTimer/SimpleTimer.h"
#include "LedControl.h"
/* Library Include*/

/*Define Pins*/          /*Select Pin*/
#define clkOut                52 /*PF7*/
#define dataIn                51 /*PF6*/
#define speakerPin            8
#define buzzerPin             7 /*PE7*/
#define heaterPin             4 /*PE4*/
#define relayPin              12 /*PB4*/
#define incTemp               3 /*PE3*/
#define decTemp               2 /*PE2*/
#define preheatLed            29 /*PC1*/
#define startTimer            24
#define resetTimer            11
#define microLed1             26
#define microLed2             27
#define autoLed               28
#define manualLed             25
#define modeBtn               30
#define settempBtn            45
#define silenceAlarm          48
#define resetAlarm            42
#define silenceLed            47
#define devtempLed            49
#define powerfailLed          50
#define overtempLed           44
#define probefailLed          43
#define batteryPin            46
// LedControl(DIN, CLK, CS, NUMBER OF IC USED)
LedControl lc = LedControl(13, 5, 6, 2);
// int graphpin[]={31, 32, 33, 34, 35, 37, 38, 39, 40, 41};
int graphpin[]={41, 40, 39, 38, 37, 35, 34, 33, 32, 31};
/*End Define Pins*/

/*Define Variable*/
//display variable
float displaysetTemp = 34.0;
float displayskinTemp = 00.0;
uint8_t displayTimer1; //Hour
uint8_t displayTimer2; //Minute
uint8_t displayTimer3; //Second
float skinTemp;
bool segmentblink;

//digit display variable
uint8_t digit1 = 0;
uint8_t digit2 = 0;
uint8_t digit3 = 0;
uint8_t digit4 = 0;
uint8_t digit5 = 0;
uint8_t digit6 = 0;
uint8_t digit7 = 0;
uint8_t digit8 = 0;
uint8_t digit9 = 0;
uint8_t digit10 = 0;
uint8_t digit11 = 0;
uint8_t digit12 = 0;

//error variable
bool error0 = 0;
bool error1 = 0;
bool error2 = 0;
bool error3 = 0;

//button variable
bool lastPower0;
bool lastPower1;
bool lastPower2;
bool lastPower3;
bool lastPower4;
bool lastPower5;
bool lastPower6;
bool lastPower7;
bool lastPower8;
bool lastPower9;
bool lastPower10;
bool currentPower0;
bool currentPower1;
bool currentPower2;
bool currentPower3;
bool currentPower4;
bool currentPower5;
bool currentPower6;
bool currentPower7;
bool currentPower8;
bool currentPower9;
bool currentPower10;
uint8_t setTemp;
uint8_t holdTimer;
unsigned long tcal;
uint8_t timeMode;
int holdTimer1;
int holdTimer2;
int holdTimer3;
bool rstTimer;
bool rstAlarm;
uint8_t modeSelect = 0;
bool preheatMode;
bool autoMode;
bool manualMode;
uint8_t modeControl = 0;
uint16_t vbattery;

//sensor and control variable
float babyskinTemp;
double outHeater;
uint8_t heaterPwm;
uint8_t heatedPower;
double setPoint;
double input;
uint8_t settimerMode;
unsigned long lastTime1;
unsigned long timerReset;
unsigned long startup;
uint8_t loopTimer;
uint8_t manualPower = 1;
uint32_t adcValue;
float convertSkin;
float errorSkin;
uint8_t steadytime;
uint8_t timeSecond;
uint8_t timeMinute;
uint8_t timeHours;
int intrCount;
unsigned long incskinTime;
unsigned long decskinTime;
unsigned long lastTime3;
float holdSkin;
bool debugMode = 0;
float sumSkin  = 0;
unsigned long ledBlink1;
uint8_t q = 0;
float read_skin_temp;

//Isolation communication Value
uint8_t pulsa = 0;
unsigned long timePulse;
uint8_t highState = 0;
unsigned long generateData;
bool sampleState = 0;
uint16_t dataSensor;
uint8_t halfBit;
int dataArray[20];
bool start = 0; //1
unsigned long k;
unsigned long l;
unsigned long i;
uint8_t o;
unsigned long p;
uint8_t alarm[5] = {0,0,0,0,0};
uint8_t sAlarm[5] = {0,0,0,0,0};
bool sunyiValue = 0;
bool last_sunyi_value = 0;
int alarmValue = 0;
bool alarmValue2 = 0;
bool sirenAlarm = 0;
unsigned long debugTimer;
bool alarmRst = 0;
bool loopAlarm = 0;
bool whichSensor;
int fullBit;

Radiant Radiant_Warmer;
PID myPID(&input, &outHeater, &setPoint, 40, 0.23, 0, DIRECT);
RTC_DS3231 rtc;
SimpleTimer timer0;
SimpleTimer timer1;
SimpleTimer timer2;
SimpleTimer timer3;
SimpleTimer timer4;

/*Serial Debug*/
void generate_json(){
  StaticJsonDocument<512> outDbg;
  //SensorVariable
  outDbg["s"][0] = skinTemp;
  outDbg["s"][1] = holdSkin;
  outDbg["s"][2] = sumSkin;
  outDbg["s"][3] = manualPower;
  outDbg["s"][4] = setTemp; 
  outDbg["s"][5] = setPoint;
  outDbg["s"][6] = round(outHeater);
  outDbg["s"][7] = heaterPwm;
  outDbg["s"][8] = errorSkin;
  outDbg["t"][0] = timeSecond;
  outDbg["t"][1] = timeMinute;
  outDbg["t"][2] = timeHours;
  outDbg["t"][3] = timeMode;
  // outDbg["t"][3] = holdTimer1;
  // outDbg["t"][4] = holdTimer2;
  // outDbg["t"][5] = holdTimer3;
  outDbg["m"][0] = modeControl;
  // outDbg["m"][1] = modeControl2;
  outDbg["m"][2] = modeSelect;
  outDbg["m"][1] = debugMode;
  outDbg["a"][0] = error0;
  outDbg["a"][1] = error1;
  outDbg["a"][2] = error2;
  outDbg["a"][3] = error3;
  outDbg["a"][4] = sirenAlarm;
  outDbg["a"][5] = alarmValue2;
  outDbg["a"][6] = last_sunyi_value;
  outDbg["a"][7] = sunyiValue;
  // serializeJson(outDbg, Serial);
  // Serial.println();
}


void set_ctrl_btn(){
  lastPower2 = currentPower2;
  currentPower2 = digitalRead(settempBtn);
    //SET CONTROL BUTTON
    if(lastPower2 == HIGH && currentPower2 == LOW){
        setTemp++;
        if(modeSelect == 1){  
           if(setTemp == 1){
              modeControl = 1;
              setPoint = displaysetTemp;
           }
           if(setTemp == 0 || setTemp > 1){
              modeControl = 0;
           }           
        }

        if(modeSelect == 2){
          if(setTemp == 1){
            modeControl = 2;
          }
          if(setTemp == 0 || setTemp > 1){
            modeControl = 0;
          }
        }
        if(modeSelect == 3){
          if(setTemp == 1){
            modeControl = 3;
          }
          if(setTemp == 0 || setTemp > 1){
            modeControl = 0;
          }
        }
      if(setTemp > 1){
          setTemp = 0;
      }
      }
  //END SET CONTROL BUTTON 
}

/*Function of each Button session*/
void btn_menu(){
    lastPower0 = currentPower0;
    lastPower1 = currentPower1;
    lastPower3 = currentPower3;
    lastPower4 = currentPower4;
    lastPower5 = currentPower5;
    lastPower6 = currentPower6;
    currentPower0 = digitalRead(incTemp);
    currentPower1 = digitalRead(decTemp);
    currentPower3 = digitalRead(startTimer);
    currentPower4 = digitalRead(resetTimer);
    currentPower5 = digitalRead(modeBtn);
    currentPower6 = digitalRead(silenceAlarm);

  if(debugMode == 0){
  //MODE SELECT BUTTON  
    if(lastPower5 == HIGH && currentPower5 == LOW){
        modeSelect++;
        modeControl = 0;
        if(modeSelect == 1){
        }
        if(modeSelect == 2){        
        }
        if(modeSelect == 3){
        }
        if(modeSelect == 4){
          modeSelect = 1;
        }
    }  
  //END MODE SELECT BUTTON

  //INCREMENT AND DECREMENT BUTTON
    //AUTO
    if(lastPower0 == HIGH && currentPower0 == LOW){
        if(modeSelect == 1 && modeControl == 0){
           displaysetTemp = displaysetTemp + 0.1;
           if(displaysetTemp >= 38){
              displaysetTemp = 38;
            }
        }
        //manual
        if(modeSelect == 2 && modeControl == 0){
            manualPower = manualPower + 1;
            if(manualPower > 10){
              manualPower = 10;
            }
        }  
      }
    if(currentPower0 == LOW){
        if(modeSelect == 1 && modeControl == 0){
          if(millis() - incskinTime > 2000){
            displaysetTemp = displaysetTemp + 0.02;
            if(displaysetTemp >= 38){
              displaysetTemp = 38;
            }
          }
        }
    }
    if(currentPower0 == HIGH){
        incskinTime = millis();
      } 

    if(lastPower1 == HIGH && currentPower1 == LOW){
        if(modeSelect == 1 && modeControl == 0){ 
          displaysetTemp = displaysetTemp - 0.1;
            if(displaysetTemp <= 34){
                displaysetTemp = 34;
            }
          }
        if(modeSelect == 2 && modeControl == 0){
          manualPower = manualPower - 1;  
        }
        if(manualPower <= 0){
          manualPower = 0;
        }
        if(manualPower <= 255 && manualPower > 11){
          manualPower = 0;
        }               
    }
    if(currentPower1 == LOW){
        if(modeSelect == 1 && modeControl == 0){
          if(millis() - decskinTime > 2000){  
            displaysetTemp = displaysetTemp - 0.02;
            if(displaysetTemp <= 34){
              displaysetTemp = 34;
            }
          }
        }  
      }
    if(currentPower1 == HIGH){
        decskinTime = millis();
    }
  //END INCREMENT AND DECREMENT BUTTON   
  }

  if(debugMode == 1){
  //INCREMENT & DECREMENT DEBUG MODE  
    if(lastPower0 == HIGH && currentPower0 == LOW){
      holdSkin = holdSkin + 0.1;
      if(holdSkin >= 38.5){
        holdSkin = 38.5;
      }
    }
    if(lastPower1 == HIGH && currentPower1 == LOW){
      holdSkin = holdSkin - 0.1;
      if(holdSkin <= 25){
        holdSkin = 25;
      }
    }
  }

  //SET TIMER MODE
    if(lastPower3 == HIGH && currentPower3 == LOW){
        settimerMode++;
        if(settimerMode == 1){
           timeMode = 1;
        }
        if(settimerMode == 2){
            holdTimer1 = displayTimer1;
            holdTimer2 = displayTimer2;
            holdTimer3 = displayTimer3;
            settimerMode = 3;
        }
        if(settimerMode == 3){
            timeMode = 3;
        }
        if(settimerMode > 3){
            settimerMode = 1;
            timeMode = 1;
        }        
    }
  //END SET TIMER MODE

  //RESET TIMER BUTTON
    if(lastPower4 == HIGH && currentPower4 == LOW){
       settimerMode = 0;
       timeMode = 0;
    }
  //END RESET TIMER BUTTON

  //DEBUG MODE BUTTON 
    if(currentPower4 == LOW){
      if(millis() - debugTimer > 7000 && debugMode == 0){
        holdSkin = skinTemp;
        debugMode = 1;
        timeMode = 0;
        debugTimer = millis();
      }
      else if(millis() - debugTimer > 7000 && debugMode == 1){
        sumSkin = holdSkin - skinTemp;
        debugMode = 0;
        debugTimer = millis();
      }
    }
    if(currentPower4 == HIGH){
      debugTimer = millis();
    }
  //END DEBUG MODE BUTTON

  //SILENCE ALARM BUTTON
    if(lastPower6 == HIGH && currentPower6 == LOW){
      k = millis();
      // l = millis();
      if(millis() - k < 60000){
        sunyiValue = 1;
        digitalWrite(silenceLed, LOW);
      }
      if(millis() - l > 1050 && i == 0){
        last_sunyi_value = 1;
        i = 1;
        l = millis();
      }
    }
      if(millis() - l > 1050 && i == 1){
        last_sunyi_value = 0;
        i = 0;
        l = millis();
    }
      if(millis() - k > 60000){
        sunyiValue = 0;
        // digitalWrite(silenceLed, HIGH);
        k = millis();
    } 
  //END SILENCE ALARM BUTTON
}
/*end fucntion of each button session*/

void reset_alarm_btn(){
  //RESET ALARM FUNCTION
  lastPower7 = currentPower7;
  currentPower7 = digitalRead(resetAlarm);
  if(lastPower7 == HIGH && currentPower7 == LOW){
    p = millis();
    if(millis() - p < 20000){
      alarmRst = 1;
    }
  }
  if(millis() - p > 20000){
    alarmRst = 0;
  }
  //END RESET ALARM FUNCTION
}


/*value of  each digits seven segment*/
void digit_value(){
  if(debugMode == 0){
    displayskinTemp = skinTemp;
  }
  if(debugMode == 1){
    displayskinTemp = holdSkin;
    sumSkin = 0;
  }
    digit1 = displayskinTemp / 10;
    digit2 = displayskinTemp - (digit1 * 10);
    digit3 = (displayskinTemp * 10 - (digit1 * 100) - (digit2 * 10));
    digit4 = displaysetTemp / 10;
    digit5 = displaysetTemp - (digit4 * 10);
    digit6 = (displaysetTemp * 10 - (digit4 * 100) - (digit5 * 10));
    digit7 = displayTimer1 / 10;
    digit9 = displayTimer2 / 10;
    digit11 = displayTimer3 / 10;

    if(displayTimer1 < 10){
        digit8 = displayTimer1;
    }
    if(displayTimer1 >= 10){
        digit8 = (displayTimer1)-((displayTimer1/10)* 10);
    }
    if(displayTimer2 < 10){
        digit10 = displayTimer2;
    }
    if(displayTimer2 >= 10){
        digit10 = (displayTimer2)-((displayTimer2/10)* 10);
    }
    if(displayTimer3 < 10){
        digit12 = displayTimer3;
    }
    if(displayTimer3 >= 10){
        digit12 = (displayTimer3)-((displayTimer3/10)* 10);
    } 
  /*end value of each digits*/   
}

/*Displaying value to each seven segment digits*/ 
void digit_display(){ 
  digit_value();
  if(debugMode == 0){  
    //display skin Temp
    lc.setDigit(1, 1, digit1, false);    
    lc.setDigit(1, 5, digit2, true);     
    lc.setDigit(1, 7, digit3, false);

  //displayTimer
    lc.setDigit(0, 1, digit7, false);    
    lc.setDigit(0, 5, digit8, false);     
    lc.setDigit(0, 7, digit9, false);
    lc.setDigit(0, 3, digit10, false);    
    lc.setDigit(0, 2, digit11, false);     
    lc.setDigit(0, 6, digit12, false);

    if(modeSelect == 1){ //AutoMode
      digitalWrite(autoLed, HIGH);
      digitalWrite(manualLed, LOW);
      digitalWrite(preheatLed, LOW);
    //display set skin temp
      if(modeControl == 0){  
        if(millis() -  ledBlink1 > 250 && q == 0){
          lc.setDigit(1, 6, digit4, false);    
          lc.setDigit(1, 4, digit5, true);     
          lc.setDigit(1, 0, digit6, false);
          q = 1;
          ledBlink1 = millis();
        }
        if(millis() - ledBlink1 > 250 && q == 1){
          // lc.setChar(1, 6, 'blank', false);    
          // lc.setChar(1, 4, 'blank', false);     
          // lc.setChar(1, 0, 'blank', false);
          lc.setDigit(1, 6, digit4, false);    
          lc.setDigit(1, 4, digit5, false);     
          lc.setDigit(1, 0, digit6, false);          
          q = 0;
          ledBlink1 = millis();        
        }
      }
      if(modeControl == 1){
          lc.setDigit(1, 6, digit4, false);    
          lc.setDigit(1, 4, digit5, true);     
          lc.setDigit(1, 0, digit6, false);        
      }
        heatedPower = map(heaterPwm, 0, 255, 0, 10);
        for(int thisled = 0; thisled < 10; thisled++){
          if(thisled < heatedPower){
            digitalWrite(graphpin[thisled], LOW);
          }
          else{
            digitalWrite(graphpin[thisled], HIGH);
          }
        }      
    }
    
    
    //IN MODE CONTROL = 0 LEDBAR BLINKING
    if(modeSelect == 2){
       digitalWrite(autoLed, LOW);
       digitalWrite(manualLed, HIGH);
       digitalWrite(preheatLed, LOW);
      //10 segment bar blinking
       heatedPower = manualPower;
       lc.setChar(1, 6, '-', false);
       lc.setChar(1, 4, '-', false);
       lc.setChar(1, 0, '-', false);
       if(modeControl == 0){
        //  if(millis() - ledBlink1 > 500 && q == 0){
           for(int thisled = 0; thisled < 10; thisled++){
             if(thisled < heatedPower){
               digitalWrite(graphpin[thisled], LOW);
             }
             else{
               digitalWrite(graphpin[thisled], HIGH);
             }
           }
          //  q = 1;
          //  ledBlink1 = millis();
        //  }
        //  if(millis() - ledBlink1 > 500 && q == 1){
        //     for(int thisled = 0; thisled < 10; thisled++){
        //       digitalWrite(graphpin[thisled], HIGH);
        //     }
        //    q = 0;
        //    ledBlink1 = millis();       
        //  }
       }

      if(modeControl == 2){
       for(int thisled = 0; thisled < 10; thisled++){
        if(thisled < heatedPower){
          digitalWrite(graphpin[thisled], LOW);
        }
        else{
          digitalWrite(graphpin[thisled], HIGH);
        }
       }
      }
    }

    if(modeSelect == 3){ //PreheatMode
       digitalWrite(autoLed, LOW);
       digitalWrite(manualLed, LOW);
       digitalWrite(preheatLed, HIGH);

      heatedPower = map(heaterPwm, 0, 255, 0, 10);
        for(int thisled = 0; thisled < 10; thisled++){
          if(thisled < heatedPower){
            digitalWrite(graphpin[thisled], LOW);
          }
          else{
            digitalWrite(graphpin[thisled], HIGH);
          }
        }


      // lc.setChar(1, 6, 'p', false);
      // lc.setChar(1, 4, 'r', true);
      // lc.setChar(1, 0, 'e', false);
    }   
 } 

 if(debugMode == 1){
  //display skin temp debug Mode
    lc.setDigit(1, 1, digit1, false);    
    lc.setDigit(1, 5, digit2, true);     
    lc.setDigit(1, 7, digit3, false);

  //Display timer debug Mode
    lc.setChar(0, 1, 'c', false);    
    lc.setChar(0, 5, 'a', false);     
    lc.setChar(0, 7, 'l', false);
    lc.setChar(0, 3, '-', false);    
    lc.setChar(0, 2, '-', false);     
    lc.setChar(0, 6, '-', false);  
 } 
}
/*End Displaying value of each 7 segment digits*/


/*Read Skin Temperature from atTiny1616*/
void read_skin_temperature(){
  // read_skin_temp = dataSensor;
  read_skin_temp = Radiant_Warmer.get_value_sensor(dataSensor, whichSensor);
  if(read_skin_temp > 21){
    // babyskinTemp = read_skin_temp/100;
    babyskinTemp = read_skin_temp + sumSkin;
    if(millis()-tcal > 1000){
      skinTemp = babyskinTemp;
      tcal = millis();
    }
  }
  if(read_skin_temp <= 21){
    skinTemp = 0;
    babyskinTemp = 0;
  }
/*end Read skin temperature*/  
}


/*Control Function*/
void run_control(){
//Non PID Control each delta temperature has a value of PWM
  heaterPwm = Radiant_Warmer.get_value_heat(setPoint, modeControl, babyskinTemp, heatedPower);
  set_pwm(heaterPwm);
}

void set_pwm(uint8_t heat){
  Serial.println("controled");
  analogWrite(heaterPin, heat);
}

//PID Control only used when temperature rise from 0 to setpoint
void PID_control(){
  errorSkin = (setPoint*10) - (babyskinTemp*10);
  if(error0 == 0 && error1 == 0 && error2 == 0 && error3 == 0){
    if(modeControl == 1 && errorSkin > 4){
      digitalWrite(relayPin, HIGH);
      input = babyskinTemp;
      Serial.println("mode0");
      myPID.Compute();
      heaterPwm = outHeater;
      set_pwm(outHeater);
    }
    else if(modeControl == 1 && errorSkin <= 4){
      digitalWrite(relayPin, HIGH);
      run_control();
      Serial.println("mode1");
      outHeater = heaterPwm;
      input = babyskinTemp;
    }
    else if(modeControl == 2){
      digitalWrite(relayPin, HIGH);
      Serial.println("mode2");
      run_control();
    }
    else if(modeControl == 0){
      digitalWrite(relayPin, LOW);
      outHeater = 0;
      heaterPwm = 0;
      // manualPower = 0;
      // setPoint = 0;
    }
  }else{
    digitalWrite(relayPin, LOW);
    outHeater = 0;
    heaterPwm = 0;
    // manualPower = 0;
    // setPoint = 0;  
  }
}

//Alarm trigger
void alarm_control(){
  if(sirenAlarm == 1){
    if(alarmValue2 == 0){
      noTone(speakerPin);
      if(millis() - lastTime3 > 1000 && loopAlarm == 0){
        lastTime3 = millis();
        loopAlarm = 1;
        // Serial.println("buzz");
        // tone(buzzerPin, 2200);
        digitalWrite(buzzerPin, HIGH);
      }
      if(millis() - lastTime3 > 1000 && loopAlarm == 1){
        lastTime3 = millis();
        loopAlarm = 0;
        // Serial.println("didnt buzz");
        // noTone(buzzerPin);
        digitalWrite(buzzerPin, LOW);
      }
    }
    if(alarmValue2 == 1){
      digitalWrite(buzzerPin, LOW);
      if(millis() - lastTime3 > 1000 && loopAlarm == 0){
        lastTime3 = millis();
        loopAlarm = 1;
        tone(speakerPin, 2300);
      }
      if(millis() - lastTime3 > 1000 && loopAlarm == 1){
        lastTime3 = millis();
        loopAlarm = 0;
        tone(speakerPin, 1800);
      }
    }
  }else{
    noTone(speakerPin);
    digitalWrite(buzzerPin, LOW);
 }
}
/*End Control and Alarm Function*/

/*atTiny1616 Communication*/
//Generating 12 pulse with 2ms time each high and low condition
void generate_pulse(){
  if(start == 0 && pulsa <= 19){ //9
    if(millis() - timePulse > 2 && highState == 0){ //2
      digitalWrite(clkOut, HIGH);     
      timePulse = millis();
      highState = 1;
    }
    if(millis() - timePulse > 2 && highState == 1){
      dataArray[pulsa] = digitalRead(dataIn);
      pulsa++;
      timePulse = millis();
      highState = 2;
    } 
    if(millis() - timePulse > 2 && highState == 2){
      digitalWrite(clkOut, LOW);
      timePulse = millis();
      highState = 3;
    }
    if(millis() - timePulse > 2 && highState == 3){
      timePulse = millis();
      highState = 0;
    }
  }  
}

//Sampling data each high pulse condition
void sample_data(){
    if(pulsa > 19){ //9
      digitalWrite(clkOut, LOW);
        if(millis() - generateData > 100 && sampleState == 0){
          for(halfBit = 0; halfBit < 10; halfBit++){
            bitWrite(whichSensor, 9- halfBit , dataArray[halfBit]);
            Serial.print(dataArray[halfBit]);
          }
          for(fullBit = 10; fullBit < 20; fullBit++){
            bitWrite(dataSensor, 19- fullBit, dataArray[fullBit]);
            Serial.print(dataArray[fullBit]);
          }
            Serial.print("-");
            Serial.print(dataSensor);
            Serial.print("-");
            Serial.print(whichSensor);
            Serial.print("-");
            Serial.println(babyskinTemp);
            start = 1;
            sampleState = 1;
            generateData = millis();
            read_skin_temperature();
        }
        if(millis() - generateData > 100 && sampleState == 1){
            pulsa = 0;
            start = 0;
            sampleState = 0;
            generateData = millis();            
        }
    }  
}
/*End atTiny1616 Communication*/

/*Error and Alarm Triggering Session*/
void read_error(){         
  errorSkin = (setPoint*10) - (babyskinTemp*10);
  if(alarmRst == 0){
    //vbat sense
    vbattery = analogRead(batteryPin);

    //Power Failure
    // bool acDetect = digitalWrite(acdetectPin);
    // if(acDetect == HIGH){
    //   error4 == 0;
    // }
    // if(acDetect == LOW){
    //   error4 == 0;
    // }

    // powerIn = digitalRead();
    if(millis() - startup > 5000){
      if(rstAlarm == 0){
      //Probe Missing
        if(dataSensor == 0){
        error0 = 1;
        }
        if(dataSensor != 0){
          error0 = 0;
        }

    //Temperature Deviation
      if(modeControl == 1){
        steadytime = 0;
        if(errorSkin < 1 && errorSkin >= -1){
          steadytime++;
          error1 = 0;
          if(steadytime >= 60){
            steadytime = 60;
          }
        }
        if(steadytime == 60){
          if(babyskinTemp >= setPoint+1 || babyskinTemp <= setPoint-1){
            error1 = 1;
          }
          else{error1 = 0;}
        }
      }

      //High Temperature
      if(babyskinTemp >= 38.5){
        error2 = 1;
      }
      if(babyskinTemp < 38.5){
        error2 = 0;
      }
    }
   }
  }
  if(alarmRst == 1){
    error0 = 0;
    error1 = 0;
    error2 = 0;
    error3 = 0;
  }

  // LED Triggered 
  //probefail
  if(error0 == 1){
    digitalWrite(probefailLed, HIGH);
    }
  if(error0 == 0){
    digitalWrite(probefailLed, LOW);
    sAlarm[0] = 1;
  }
  //tempdev
  if(error1 == 1){
    digitalWrite(devtempLed, HIGH);
  }
  if(error1 == 0){
    digitalWrite(devtempLed, LOW);
    sAlarm[1] = 1;
  }
  //high temp
  if(error2 == 1){
    digitalWrite(overtempLed, HIGH);
  }
  if(error2 == 0){
    digitalWrite(overtempLed, LOW);
    sAlarm[2] = 0;
  }

  /*Silent Alarm function*/
  if(error0 == 1 && sAlarm[0] == 1){
    alarm[0] = 1;
  }
  if(error1 == 1 && sAlarm[1] == 1){
    alarm[1] = 1;
  }
  if(error2 == 1 && sAlarm[2] == 1){
    alarm[2] = 1;
  }
  if(error3 == 1 && sAlarm[3] == 1){
    alarm[3] = 1;
  }
  //SIlent button triggered
  if(alarm[0] == 1 && last_sunyi_value == 1){
    sAlarm[0] = 0;
    alarm[0] = 0;
  }
  if(alarm[1] == 1 && last_sunyi_value == 1){
    sAlarm[1] = 0;
    alarm[1] = 0;
  }  
  if(alarm[2] == 1 && last_sunyi_value == 1){
    sAlarm[2] = 0;
    alarm[2] = 0;
  }
  if(alarm[3] == 1 && last_sunyi_value == 1){
    sAlarm[3] = 0;
    alarm[3] = 0;
  }  
  // if(alarm[4] == 1 && last_sunyi_value == 1){
  //   sAlarm[4] = 0;
  //   alarm[4] = 0;
  // }
  // check error again after silent btn triggered
  for(o=0; o<4;o++){
    if(alarm[o] == 1){
      sirenAlarm = 1;
      alarmValue++;
      if(alarmValue >= 6000){
        alarmValue = 6000;
        alarmValue2 = 1;
      }
    }
    if(last_sunyi_value == 1){
      sirenAlarm = 0;
      alarmValue = 0;
      alarmValue2 = 0;
    }
    if(sunyiValue == 0){
      digitalWrite(silenceLed, HIGH);
      for(uint8_t j=0; j<4; j++){
        sAlarm[j] = 1;
      }
    }
    if(error0 == 0 && error1 == 0 && error2 == 0 && error3 == 0){
      alarmValue = 0;
      alarmValue2 = 0;
      sirenAlarm = 0;
    }
  }
}
/*end of error and alarm triggering session*/

/*Real time clock session*/
void generate_rtc(uint8_t nowSecond, uint8_t nowMinute, uint8_t nowHours){
    // DateTime now  = rtc.now();
    if(timeMode == 0){
      displayTimer1 = 00;
      displayTimer2 = 00;
      displayTimer3 = 00;
    }
    else if(timeMode == 1){
            displayTimer1 = nowHours;
            displayTimer2 = nowMinute;
            displayTimer3 = nowSecond;
    }
    else if(timeMode == 3){
            displayTimer1 = holdTimer1;
            displayTimer2 = holdTimer2;
            displayTimer3 = holdTimer3;
    }
}

void time_start(){
  if(timeMode == 1){
      timeSecond++;
      if(timeSecond >= 60){
        timeSecond = 0;
        timeMinute++;
      }
      if(timeMinute >= 60){
        timeMinute = 0;
        timeHours++;
      }
      if(timeHours >= 99){
        timeHours = 0;
        timeSecond = 0;
        timeMinute = 0;
      }
  }
    // else intrCount++;
  if(timeMode == 0){
    timeSecond = 0;
    timeMinute = 0;
    timeHours = 0;
  }
  if(timeMode == 3){
    timeSecond = holdTimer3;
    timeMinute = holdTimer2;
    timeHours = holdTimer1;
  }
}

/*end RTC session*/


void setup(){
  analogReference(EXTERNAL);
  Serial.begin(9600);
  // rtc.begin();  
  timer0.setInterval(1000, read_error);
  timer2.setInterval(1000, generate_json);
  timer3.setInterval(1000, PID_control);
  timer4.setInterval(1000, time_start);
  lc.setIntensity(0, 2);
  lc.setIntensity(1, 2);
  lc.clearDisplay(0);
  lc.clearDisplay(1);
  lc.shutdown(0, false);
  lc.shutdown(1, false);

  pinMode(incTemp, INPUT);
  pinMode(decTemp, INPUT);
  pinMode(startTimer, INPUT);
  pinMode(resetTimer, INPUT);
  pinMode(modeBtn, INPUT);
  pinMode(settempBtn, INPUT);
  pinMode(silenceAlarm, INPUT);
  pinMode(resetAlarm, INPUT);
  pinMode(dataIn, INPUT);
  pinMode(clkOut, OUTPUT);

  pinMode(microLed1, OUTPUT);
  pinMode(preheatLed, OUTPUT);
  pinMode(autoLed, OUTPUT);
  pinMode(manualLed, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(silenceLed, OUTPUT);
  pinMode(devtempLed, OUTPUT);
  pinMode(powerfailLed, OUTPUT);
  pinMode(overtempLed, OUTPUT);
  pinMode(probefailLed, OUTPUT);
  pinMode(relayPin, OUTPUT);

  digitalWrite(microLed1, LOW);
  digitalWrite(relayPin, LOW);
  digitalWrite(silenceLed, HIGH);
  digitalWrite(speakerPin, LOW);
  digitalWrite(buzzerPin, LOW);
  
  for(uint8_t thisled = 0; thisled < 10; thisled++){
    pinMode(graphpin[thisled], OUTPUT);
    digitalWrite(graphpin[thisled], HIGH);
  }
  myPID.SetOutputLimits(0, 180);
  myPID.SetMode(AUTOMATIC);
  pulsa = 0;
  start = 0;

/*initialize timer register for generating RTC*/
  // cli();
  // TCCR0 = 0;        //stop timer 0
  // TIMSK = 0;        //disable timer 0 interrupt
  // ASSR = (1 << AS0); //select asynchronous operation timer0
  // TCNT0 = 0;        //clear timer 0 interrupt 
  // TCCR0 = (1 << WGM01)|(1 << CS02)|(1 << CS01)|(0 << CS00); //prescaler 256
  // while(ASSR & ((1 << TCN0UB)|(1 << OCR0UB)|(1 << TCR0UB))); //wait tcn and tcr cleared
  // OCR0 = 1.28; //antara 128/1.28 pres 256 (128 = 1hz, 1.28 = 100hz)
  // TIFR = (1 << OCF0); //clear interrupt flag
  // TIMSK = (1 << OCIE0);
  // sei();
/*end initialize timer register*/  
  delay(2000);
}

void loop(){
  generate_pulse();
  sample_data();
  digit_display();
  generate_rtc(timeSecond, timeMinute, timeHours);
  set_ctrl_btn();
  reset_alarm_btn();
  btn_menu();
  alarm_control();
  timer0.run(); //read error
  // timer2.run(); //generate_json
  timer3.run(); //PID Control
  timer4.run(); //timestart
}

 //interrupt routine
ISR(TIMER0_COMP_vect){
  // generate_pulse();
  // sample_data();
  // digit_display();
  // generate_rtc(timeSecond, timeMinute, timeHours);
  // set_ctrl_btn();
  // btn_menu();
  // read_skin_temperature();
  // generate_json();
  // timer0.run(); //read error
  // timer2.run(); //generate_json
  // timer3.run(); //PID Control
  // Serial.println("a");
  // Serial.println(timeSecond);
  // if(intrCount == 250){
    // timeSecond++;
//   if(timeMode == 1){
//     if(intrCount == 100){ 
//       if(timeMode == 1){
//         timeSecond++;
//         if(timeSecond >= 60){
//           timeSecond = 0;
//           timeMinute++;
//         }
//         if(timeMinute >= 60){
//           timeMinute = 0;
//           timeHours++;
//         }
//         if(timeHours >= 99){
//           timeHours = 0;
//           timeSecond = 0;
//           timeMinute = 0;
//         }
//       }
//       intrCount = 0;
//     }
    // else intrCount++;
//   }
//   if(timeMode == 0){
//     timeSecond = 0;
//     timeMinute = 0;
//     timeHours = 0;
//   }
//   if(timeMode == 3){
//     timeSecond = holdTimer3;
//     timeMinute = holdTimer2;
//     timeHours = holdTimer1;
//   }
}
