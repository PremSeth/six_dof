#include <Arduino.h>
#include <Wire.h>
#include <IntervalTimer.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t STEPS[2]={4,6}, DIRS[2]={5,7};
IntervalTimer timer;
volatile bool moving=false;
volatile uint32_t pulses=0,target=0;
volatile uint8_t motor=0,state=0;
uint32_t lastContact=0;
char line[96];size_t used=0;bool overflow=false;
void stop(uint8_t reason){
  noInterrupts();timer.end();moving=false;
  digitalWriteFast(4,LOW);digitalWriteFast(6,LOW);state=reason;interrupts();
}
void tick(){
  if(!moving)return;
  if(motor==0)digitalWriteFast(4,HIGH);else digitalWriteFast(6,HIGH);
  delayMicroseconds(5);
  digitalWriteFast(4,LOW);digitalWriteFast(6,LOW);
  if(++pulses>=target){timer.end();moving=false;state=2;}
}
bool number(char* s,uint32_t &v){
  if(!s||!*s)return false;
  for(char* p=s;*p;++p)if(*p<'0'||*p>'9')return false;
  char* end;unsigned long n=strtoul(s,&end,10);if(*end)return false;v=n;return true;
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
  lastContact=millis();
  if(!strcmp(line,"PING"))Serial.println("READY WRIST_MAP_V1 A=4,5 B=6,7 ENC=4,5 MAX=2667");
  else if(!strcmp(line,"STOP")){stop(3);Serial.println("OK STOP");}
  else if(!strcmp(line,"STATUS"))Serial.printf("STATUS state=%u motor=%u pulses=%lu target=%lu\n",state,motor,(unsigned long)pulses,(unsigned long)target);
  else if(!strcmp(line,"ENCODERS")){
    if(moving){Serial.println("ERROR busy");return;}
    uint16_t a=0,b=0;bool va=encoder(4,a),vb=encoder(5,b);
    Serial.printf("ENC valid4=%u raw4=%u valid5=%u raw5=%u\n",va,a,vb,b);
  }else if(!strncmp(line,"MOVE ",5)){
    if(moving){Serial.println("ERROR busy");return;}
    char *save;strtok_r(line," ",&save);uint32_t m,n,period,dir;
    bool ok=number(strtok_r(nullptr," ",&save),m)&&number(strtok_r(nullptr," ",&save),n)&&
      number(strtok_r(nullptr," ",&save),period)&&number(strtok_r(nullptr," ",&save),dir)&&!strtok_r(nullptr," ",&save);
    if(!ok||m>1||n<1||n>2667||period<5000||period>20000||dir>1){Serial.println("ERROR MOVE_motor0or1_count1to2667_period5000to20000_dir0or1");return;}
    digitalWrite(DIRS[m],dir?HIGH:LOW);delayMicroseconds(100);
    noInterrupts();motor=m;target=n;pulses=0;state=1;moving=true;interrupts();
    if(!timer.begin(tick,period)){stop(3);Serial.println("ERROR timer");}else Serial.println("OK MOVE");
  }else Serial.println("ERROR unknown_command");
}
void setup(){
  // Leave other known stepper STEP outputs inactive. Never pulse them here.
  for(uint8_t p:{0,2,23,4,5,6,7}){digitalWrite(p,LOW);pinMode(p,OUTPUT);}
  Serial.begin(115200);Wire.begin();Wire.setClock(100000);lastContact=millis();
}
void loop(){
  if(moving&&millis()-lastContact>1500)stop(4);
  while(Serial.available()){
    char c=Serial.read();if(c=='\r')continue;
    if(c=='\n'){
      if(overflow){stop(3);Serial.println("ERROR line_too_long");}
      else {line[used]=0;command();}used=0;overflow=false;
    }else if(used<sizeof(line)-1&&!overflow)line[used++]=c;else overflow=true;
  }
}
