#define dataOut 4 //PB5
#define clkIn 5  //PB4
#define highSensor 16 //PA3
#define lowSensor 0 //PA4

//test
int a;
int dat;
int dat2;
unsigned long low;
int lowvar;
int b = 0;
unsigned long lowtime;
int dataSensor;
uint8_t current = 0;
uint8_t last = 0;
uint16_t sense1;
uint16_t sense2;
String dataArray3[13];
int r = random(0, 1023);


/*convertion ADC value to Binary 12bit*/
void convert_tobinary(int input){
  dat = 0;
  for(a=2048; a>=1; a=a/2){ 
    if((input-a)>=0){
      dataArray3[dat]= '1';
      input-=a; 
    }
    else{
      dataArray3[dat]='0';
    } 
    dat++;  
  }
}
/*end convertion adc to binary*/

/*Send sensor's data 12bit to optocoupler*/
void send_data(){
  last = current;
  current = digitalRead(clkIn);
  if(last == 0 && current == 1){
     lowvar = 0;
     digitalWrite(dataOut, dataArray3[b].toInt());
     b++;
     Serial.print(b);
     Serial.println("-");
  }
  if(b > 13){ //20
     b = 0;
     lowvar = 1;
     current = 0;
     last = 0;
     convert_tobinary(dataSensor);
  }
   if(current == LOW){
    if(millis() - lowtime > 1000){ 
     b = 0;
     }
   }
   if(current == HIGH){
      lowtime = millis();
   }
  else{}
}
/*end send data*/

/*Select probe sensor 17Kohm / 2K2ohm*/
void which_sensor(){
  sense1 = analogRead(highSensor); //17kohm
  sense2 = analogRead(lowSensor); // 2,2kohm
  uint16_t avr_temp;
  float all_temp;
  float sendTemp;
  int convertTemp;
  if(sense1 >= 1023 && sense2 != 0){
    // dataSensor = sense2;
    avr_temp = sense2;
    all_temp = ((-0.001*(pow(avr_temp, 2))) + (18*avr_temp) + 24125)/1000;
    sendTemp = all_temp*100;
    convertTemp = sendTemp;  
      if(avr_temp <= 20){
        dataSensor = 0;
      }
      else{
        dataSensor = convertTemp;
      }
  }
  else if(sense1 < 1023 && sense2 != 0){
    // dataSensor = sense1;
    avr_temp = sense1;
    all_temp = /*pers 17k ohm*/;
    sendTemp = all_temp*100;
    convertTemp = sendTemp;
      if(avr_temp <= 20){
        dataSensor = 0;
      }
      else{
        dataSensor = convertTemp;
      }
  }
  else if(sense1 == 0 && sense2 == 0){
    dataSensor = 0;
  }
}
/*end select probe*/


void setup(){
  analogReference(EXTERNAL);
  Serial.begin(9600);
  pinMode(dataOut, OUTPUT);
  pinMode(clkIn, INPUT);
  pinMode(highSensor, INPUT);
  pinMode(lowSensor, INPUT);
  which_sensor();
  convert_tobinary(dataSensor);
//    delay(1000);
    b = 0;
}

void loop(){
  which_sensor();
  send_data();
}
