#define dataOut 4 //PB5
#define clkIn 5  //PB4
#define highSensor 16 //2
#define lowSensor 0 //3



int a;
int a2;
int dat;
int dat2;
unsigned long low;
int lowvar;
int b = 0;
unsigned long lowtime;
unsigned long averageTime;
uint16_t sumdata1;
uint16_t sumdata2;
int dataSensor1;
int dataSensor2;
int sense1;
int sense2;
int total = 0;
int total2 = 0;
int mdata = 0;
int ndata = 0;
int sampleData[10];
int sampleData2[10];
uint8_t current = 0;
uint8_t last = 0;
String dataArray3[20];
int r = random(0, 1023);
//SoftwareSerial mySerial(5, 4);

void convert_tobinary(int input, int input2){
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
    for(a2=512; a2>=1; a2=a2/2){
    if((input2-a2)>=0){
      dataArray3[dat]= '1';
      input2-=a2; 
    }
    else{
      dataArray3[dat]='0';
    } 
    dat++;  
  }
}


void send_data(){
//  mean();
  last = current;
  current = digitalRead(clkIn);
  if(last == 0 && current == 1){
     lowvar = 0;
     digitalWrite(dataOut, dataArray3[b].toInt());
     b++;
   }

  if(b > 20){
     b = 0;
     lowvar = 1;
     current = 0;
     last = 0;
     convert_tobinary(dataSensor2, dataSensor1);
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

void mean(){
 if(millis() - averageTime > 200){  
  total = total - sampleData2[ndata];
  sampleData2[ndata] = analogRead(highSensor);
  total = total + sampleData2[ndata];
  ndata = ndata + 1;
  if(ndata >= 10){
    ndata = 0;
  }
  sumdata2 = total/10;

  total2 = total2 - sampleData[mdata];
  sampleData[mdata] = analogRead(lowSensor);
  total2 = total2 + sampleData[mdata];
  mdata = mdata + 1;
  if(mdata >= 10){
    mdata = 0;
  }
  sumdata1 = total2/10;

  if(sumdata1 != 0 && sumdata2 > 1020){
    dataSensor2 = 0;
    dataSensor1 = sumdata1;
  }
  if(sumdata1 == 0 && sumdata2 != 0){
    dataSensor2 = 1;
    dataSensor1 = sumdata2;
  }
  if(sumdata2 == 0 && sumdata1 == 0){
    dataSensor2 = 0;
    dataSensor1 = 0;
  }
 }

}

void setup(){
  analogReference(INTERNAL4V3);
  Serial.begin(9600);
//  mySerial.begin(9600);
    pinMode(dataOut, OUTPUT);
    pinMode(clkIn, INPUT);
    pinMode(lowSensor, INPUT);
    pinMode(highSensor, INPUT);
//    delay(1000);
    b = 0;
}


void loop(){ 
//  if(millis() - averageTime < 200){  
//   sense1 = analogRead(lowSensor);
//   sense2 = analogRead(highSensor);
// //  dataSensor1 = sense1;
// //  dataSensor2 = 1; 
//   if(sense1 != 0 && sense2 > 1020){
//     dataSensor2 = 0;
//     dataSensor1 = sense2;
//     // dataSensor1 = sumdata1;
//   }
//   if(sense1 != 0 && sense2 == 0){
//     dataSensor2 = 1;
//     dataSensor1 = sense1;
//     // dataSensor1 = sumdata2;
//   }
//   if(sense1 == 0 && sense2 == 0){
//     dataSensor2 = 0;
//     dataSensor1 = 0;
//   }
//  }
  mean();
  send_data();
}
