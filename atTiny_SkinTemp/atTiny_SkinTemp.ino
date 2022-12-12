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
String dataArray3[10];
int r = random(0, 1023);


/*konversi ADC ke Biner 10bit*/
void convert_tobinary(int input){
  dat = 0;
  for(a=512; a>=1; a=a/2){ 
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
/*end konversi adc to biner*/

/*Send data sensor 10bit biner ke optocoupler*/
void send_data(){
//  mean();
  last = current;
  current = digitalRead(clkIn);
  if(last == 0 && current == 1){
     lowvar = 0;
     digitalWrite(dataOut, dataArray3[b].toInt());
     b++;
     Serial.print(b);
     Serial.println("-");
  }
  if(b > 10){ //20
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
  sense1 = analogRead(highSensor);
  sense2 = analogRead(lowSensor);
  if(sense1 >= 1023 && sense2 != 0){
    dataSensor = sense2;
  }
  else if(sense1 < 1023 && sense2 != 0){
    dataSensor = sense1;
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
//    delay(1000);
    b = 0;
}

void loop(){
  which_sensor();
  send_data();
}
