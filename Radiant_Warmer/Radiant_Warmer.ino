/*Library Include*/
#include "Radiant.h"
#include <RTClib.h>
#include <PID_v1.h>
#include "src/SimpleTimer/SimpleTimer.h"
#include "LedControl.h"
/* Library Include*/

/*Define Pins*/          /*Select Pin*/
#define clkOut                52 /*PF7*/
#define dataIn                51 /*PF6*/
#define buzzerPin             7 /*PE7*/
#define heaterPin             4 /*PE4*/
#define relayPin              12 /*PB4*/
#define incTemp               3 /*PE3*/
#define decTemp               2 /*PE2*/
#define preheatLed            29 /*PC1*/
#define startTimer            24
#define resetTimer            42
#define autoLed               28
#define manualLed             25
#define modeBtn               30
#define settempBtn            45
#define silenceAlarm          48
#define resetAlarm            11
#define silenceLed            47
#define devtempLed            49
#define powerfailLed          50
#define overtempLed           44
#define probefailLed          43
LedControl lc = LedControl(13, 5, 6, 2);
int graphpin[]={31, 32, 33, 34, 35, 37, 38, 39, 40, 41};
/*End Define Pins*/

/*Define Variable*/
//display variable
float displaysetTemp = 27.0;
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
bool error0;
bool error1;
bool error2;
bool error3;

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
uint8_t modeSelect;
bool preheatMode;
bool autoMode;
bool manualMode;
uint8_t modeControl;

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
uint8_t manualPower;
uint32_t adcValue;
int convertSkin;
float errorSkin;
uint8_t steadytime;
uint8_t timeSecond;
uint8_t timeMinute;
uint8_t timeHours;
int intrCount;

//Isolation communication Value
uint8_t pulsa = 0;
unsigned long timePulse;
uint8_t highState = 0;
unsigned long generateData;
bool sampleState = 0;
uint16_t dataSensor;
uint8_t halfBit;
int dataArray[10];
bool start = 1;



Radiant Radiant_Warmer;
PID myPID(&input, &outHeater, &setPoint, 40, 0.23, 0, DIRECT);
RTC_DS3231 rtc;
SimpleTimer timer0;
SimpleTimer timer1;
SimpleTimer timer2;
SimpleTimer timer3;

void btn_menu(){
    //mode select 
    lastPower5 = currentPower5;
    currentPower5 = digitalRead(modeBtn);
    if(lastPower5 == HIGH && currentPower5 == LOW){
        modeSelect++;
        modeControl = 0;
        if(modeSelect == 1){
          digitalWrite(autoLed, LOW);
          digitalWrite(manualLed, HIGH);
          digitalWrite(preheatLed, HIGH);
        }
        if(modeSelect == 2){
          digitalWrite(autoLed, HIGH);
          digitalWrite(manualLed, LOW);
          digitalWrite(preheatLed, HIGH);        
        }
        if(modeSelect == 3){
          digitalWrite(autoLed, HIGH);
          digitalWrite(manualLed, HIGH);
          digitalWrite(preheatLed, LOW);
        }
        if(modeSelect > 3){
          modeSelect = 1;
        }
    }  
    //end mode select
    lastPower0 = currentPower0;
    currentPower0 = digitalRead(incTemp);
      if(lastPower0 == HIGH && currentPower0 == LOW){
        if(modeSelect == 1 && modeControl == 0){
           displaysetTemp = displaysetTemp + 0.1;
           if(displaysetTemp >= 38){
              displaysetTemp = 38;
            }
        }
        if(modeSelect == 2 && modeControl == 0){
            manualPower = manualPower + 1;
            if(manualPower > 10){
              manualPower = 10;
            }
          }  
      }

    lastPower1 = currentPower1;
    currentPower1 = digitalRead(decTemp);
      if(lastPower1 == HIGH && currentPower1 == LOW){
        if(modeSelect == 1 && modeControl == 0){ 
          displaysetTemp = displaysetTemp - 0.1;
            if(displaysetTemp <= 31){
                displaysetTemp = 31;
            }
          }
        if(modeSelect == 2 && modeControl == 0){
          manualPower = manualPower - 1;
            if(manualPower < 0){
                manualPower = 0;
              }
        }               
      }

    lastPower2 = currentPower2;
    currentPower2 = digitalRead(settempBtn);
      if(lastPower2 == HIGH && currentPower2 == LOW){
        if(modeSelect == 1){  
          setTemp = setTemp + 1;
              if(setTemp == 1){
                modeControl = 1;
                setPoint = displaysetTemp;
              }
              if(setTemp == 0){
                modeControl = 0;
                segmentblink = 1;
              }
              if(setTemp > 1){
                setTemp = 0;
              }            
        }
        if(modeSelect == 2){
          if(setTemp == 1){
            modeControl = 1;
            heatedPower = manualPower;
          }
          if(setTemp == 0){
            modeControl = 0;
          }
        }
      }

    lastPower3 = currentPower3;
    currentPower3 = digitalRead(startTimer);
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
        }        
    }
    if(currentPower3 == LOW){
      if(millis() - timerReset >= 3000){
        settimerMode = 0;
        timeMode = 0;
      }
    }

    lastPower4 = currentPower4;
    currentPower4 = digitalRead(resetTimer);
    if(lastPower4 == HIGH && currentPower4 == LOW){
       settimerMode = 0;
       timeMode = 0;
    }
}

void digit_value(){
    displayskinTemp = skinTemp;
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
}

void digit_display(){
  digit_value();
  //display skin Temp  
  lc.setDigit(1, 1, digit1, false);    
  lc.setDigit(1, 5, digit2, true);     
  lc.setDigit(1, 7, digit3, false);

  //display set skin temp
  lc.setDigit(1, 6, digit4, false);    
  lc.setDigit(1, 4, digit5, true);     
  lc.setDigit(1, 0, digit6, false);

  //displayTimer
  lc.setDigit(0, 1, digit7, false);    
  lc.setDigit(0, 5, digit8, true);     
  lc.setDigit(0, 7, digit9, false);
  lc.setDigit(0, 3, digit10, false);    
  lc.setDigit(0, 2, digit11, true);     
  lc.setDigit(0, 6, digit12, false);  
  
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

void read_skin_temperature(){
  float read_skin_temp0 = Radiant_Warmer.get_value_sensor(adcValue);
  if(read_skin_temp0 > 21){
    convertSkin = (read_skin_temp0*100);
    babyskinTemp = (float(convertSkin)/100);
    if(millis()-tcal > 1000){
      skinTemp = babyskinTemp;
    }
  }
  if(read_skin_temp0 <= 21){
    skinTemp = 0;
  }
}

void run_control(){
  heaterPwm = Radiant_Warmer.get_value_heat(setPoint, modeControl, babyskinTemp, heatedPower);
  set_pwm(heaterPwm);
}

void set_pwm(uint8_t heat){
  analogWrite(heaterPin, heat);
}

void PID_control(){
  errorSkin = (setPoint*10) - (babyskinTemp*10);
  if(error0 == 0 && error1 == 0 && error2 == 0 && error3 == 0){
    if(modeControl == 1 && errorSkin > 2){
      digitalWrite(relayPin, HIGH);
      input = babyskinTemp;
      myPID.Compute();
      set_pwm(outHeater);
    }
    else if(modeControl == 1 && errorSkin <= 2){
      digitalWrite(relayPin, HIGH);
      run_control();
      outHeater = heaterPwm;
      input = babyskinTemp;
    }
    else if(modeControl == 0){
      digitalWrite(relayPin, LOW);
    }
  }
  else{
    digitalWrite(relayPin, LOW);
  }
}

void generate_pulse(){
  if(start == 0 && pulsa <= 9){ //20
    if(millis() - timePulse > 2 && highState == 0){
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
    if(millis() - timePulse > 4 && highState == 2){
      digitalWrite(clkOut, LOW);
      timePulse = millis();
      highState = 3;
    }
    if(millis() - timePulse > 1 && highState == 3){
      timePulse = millis();
      highState = 0;
    }
  }  
}

void sample_data(){
    if(pulsa > 9){ //20
      digitalWrite(clkOut, LOW);
        if(millis() - generateData > 100 && sampleState == 0){
          for(halfBit = 0; halfBit < 10; halfBit++){
            bitWrite(dataSensor, 9- halfBit , dataArray[halfBit]);
            Serial.print(dataArray[halfBit]);
          }
            Serial.print("-");
            Serial.print(dataSensor);
            Serial.print("-");
            start = 1;
            sampleState = 1;
            generateData = millis();
        }
        if(millis() - generateData > 100 && sampleState == 1){
            pulsa = 0;
            start = 0;
            sampleState = 0;
            generateData = millis();            
        }
    }  
}

void read_error(){
  errorSkin = (setPoint*10) - (babyskinTemp*10);
  //Power Failure
  // powerIn = digitalRead();
  if(millis() - startup > 5000){
    if(rstAlarm == 0){
    //Probe Missing
      if(babyskinTemp == 0){
        error0 = 1;
      }
      if(babyskinTemp != 0){
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


void setup(){
  // rtc.begin();  
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

  pinMode(preheatLed, OUTPUT);
  pinMode(autoLed, OUTPUT);
  pinMode(manualLed, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(silenceLed, OUTPUT);
  pinMode(devtempLed, OUTPUT);
  pinMode(powerfailLed, OUTPUT);
  pinMode(overtempLed, OUTPUT);
  pinMode(probefailLed, OUTPUT);

  for(uint8_t thisled = 0; thisled < 10; thisled++){
    pinMode(graphpin[thisled], OUTPUT);
  }
  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);

/*initialize timer register*/
  cli();
  TCCR0 = 0;        //stop timer 0
  TIMSK = 0;        //disable timer 0 interrupt
  ASSR |= 1 << AS0; //select asynchronous operation timer0
  TCNT0 = 0;        //clear timer 0 interrupt 
  TCCR0 = (1 << WGM01)|(1 << CS02)|(1 << CS01)|(0 << CS00); //prescaler 256
  while(ASSR & ((1 << TCN0UB)|(1 << TCR0UB))); //wait tcn and tcr cleared
  OCR0 = 1.28;
  TIFR = (1 << OCF0); //clear interrupt flag
  TIMSK = (1 << OCIE0);
  sei();
  delay(2000);
}


void loop(){
  generate_rtc(timeSecond, timeMinute, timeHours);
}

 //interrupt routine
ISR(TIMER0_COMPA_vect){
  if(timeMode == 1){
    if(intrCount == 100){ 
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
      intrCount = 0;
    }
    else intrCount++;
  }
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
