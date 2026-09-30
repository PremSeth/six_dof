#include <Arduino.h>
#include <Wire.h>
#include <IntervalTimer.h>
#include <stdlib.h>
#include <string.h>

// Encoder-positive axis order: base, J1, J2, wrist A, wrist B.
constexpr uint8_t STEP[5]={2,23,0,4,6}, DIR[5]={3,22,1,5,7};
constexpr uint8_t PORT[5]={0,1,3,5,4}, POS_DIR[5]={1,0,1,1,1};
// Floor(speed_deg_s * 3200 * ratio / 360): never exceed configured caps.
constexpr int MAX_HZ[5]={88,666,1333,888,888};
IntervalTimer timer;
volatile int rate[5]={};
volatile uint32_t phase[5]={},steps[5]={};
volatile bool settle[5]={};
volatile uint32_t lastCommand=0,lastGood[5]={};
volatile bool moving=false;
volatile uint8_t fault=0;
char line[128];size_t used=0;bool overflow=false;
void halt(uint8_t code){
  moving=false;fault=code;
  for(unsigned i=0;i<5;++i){rate[i]=0;phase[i]=0;digitalWrite(STEP[i],LOW);}
}
void tick(){
  if(!moving)return;
  if(millis()-lastCommand>300){halt(1);return;}
  for(unsigned i=0;i<5;++i)if(millis()-lastGood[i]>500){halt(2);return;}
  bool fire[5]={};bool any=false;
  for(unsigned i=0;i<5;++i){
    if(settle[i]){settle[i]=false;continue;}
    phase[i]+=abs(rate[i]);
    if(phase[i]>=20000){phase[i]-=20000;fire[i]=true;any=true;digitalWrite(STEP[i],HIGH);}
  }
  if(any)delayMicroseconds(5);
  for(unsigned i=0;i<5;++i)if(fire[i]){digitalWrite(STEP[i],LOW);steps[i]+=rate[i]<0?UINT32_MAX:1u;}
}
bool encoder(uint8_t port,uint16_t &raw){
  Wire.beginTransmission(0x70);Wire.write((uint8_t)(1u<<port));
  bool ok=Wire.endTransmission()==0;delayMicroseconds(100);
  if(ok){ok=Wire.requestFrom((uint8_t)0x70,(uint8_t)1)==1;if(ok)ok=Wire.read()==(1<<port);}
  if(ok){Wire.beginTransmission(0x36);Wire.write((uint8_t)0x0C);ok=Wire.endTransmission(false)==0;}
  if(ok)ok=Wire.requestFrom((uint8_t)0x36,(uint8_t)2)==2;
  if(ok){raw=(Wire.read()&15)<<8;raw|=Wire.read();}
  Wire.beginTransmission(0x70);Wire.write((uint8_t)0);return Wire.endTransmission()==0&&ok;
}
void command(){
  if(!strcmp(line,"PING"))Serial.println("READY FIVE_AXIS_V2 STEP=2,23,0,4,6 DIR=3,22,1,5,7 ENC=0,1,3,5,4 POS=1,0,1,1,1 HZ=88,666,1333,888,888");
  else if(!strcmp(line,"STOP")){noInterrupts();halt(0);interrupts();Serial.println("OK STOP");}
  else if(!strcmp(line,"READ")){
    uint16_t raw[5]={};bool valid[5]={};uint32_t counts[5];
    for(unsigned i=0;i<5;++i){
      valid[i]=encoder(PORT[i],raw[i]);
      noInterrupts();counts[i]=steps[i];if(valid[i])lastGood[i]=millis();interrupts();
    }
    Serial.printf("FRAME fault=%u moving=%u",fault,moving);
    for(unsigned i=0;i<5;++i)Serial.printf(" v%u=%u r%u=%u s%u=%lu",i,valid[i],i,raw[i],i,(unsigned long)counts[i]);
    Serial.println();
  }else if(!strncmp(line,"VEL ",4)){
    if(fault){Serial.println("ERROR latched_fault_STOP_required");return;}
    int next[5];char* save;strtok_r(line," ",&save);bool ok=true;
    for(unsigned i=0;i<5;++i){
      char* token=strtok_r(nullptr," ",&save);if(!token){ok=false;break;}
      char* end;long n=strtol(token,&end,10);
      if(end==token||*end||n<-MAX_HZ[i]||n>MAX_HZ[i]){ok=false;break;}next[i]=n;
    }
    if(!ok||strtok_r(nullptr," ",&save)){noInterrupts();halt(3);interrupts();Serial.println("ERROR five_rates_or_axis_speed_limit");return;}
    for(unsigned i=0;i<5;++i)if(millis()-lastGood[i]>500){Serial.println("ERROR stale_feedback");return;}
    noInterrupts();moving=false;
    for(unsigned i=0;i<5;++i){
      if((rate[i]<0)!=(next[i]<0)||(!rate[i]&&next[i])){settle[i]=true;phase[i]=0;}
      uint8_t level=next[i]>=0?POS_DIR[i]:1-POS_DIR[i];digitalWrite(DIR[i],level?HIGH:LOW);
      rate[i]=next[i];moving=moving||next[i]!=0;
    }
    lastCommand=millis();interrupts();Serial.println("OK VEL");
  }else Serial.println("ERROR unknown_command");
}
void setup(){
  for(unsigned i=0;i<5;++i){digitalWrite(STEP[i],LOW);pinMode(STEP[i],OUTPUT);digitalWrite(DIR[i],LOW);pinMode(DIR[i],OUTPUT);}
  Serial.begin(115200);Wire.begin();Wire.setClock(100000);timer.begin(tick,50);
}
void loop(){
  while(Serial.available()){
    char c=Serial.read();if(c=='\r')continue;
    if(c=='\n'){if(overflow){noInterrupts();halt(3);interrupts();Serial.println("ERROR long_line");}else{line[used]=0;command();}used=0;overflow=false;}
    else if(used<sizeof(line)-1&&!overflow)line[used++]=c;else overflow=true;
  }
}
