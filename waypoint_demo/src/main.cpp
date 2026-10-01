#include <Arduino.h>
#include <Wire.h>
#include <IntervalTimer.h>
#include <Servo.h>
#include <stdlib.h>
#include <string.h>

// Encoder-positive axis order: base, J1, J2, wrist A, wrist B.
constexpr uint8_t STEP[5]={2,23,0,4,6}, DIR[5]={3,22,1,5,7};
constexpr uint8_t PORT[5]={0,1,3,5,4}, POS_DIR[5]={1,0,1,1,1};
// Floor(speed_deg_s * 3200 * ratio / 360): never exceed configured caps.
constexpr int MAX_HZ[5]={355,933,2000,888,888};
constexpr uint8_t SERVO_PIN[2]={15,14}; // sixth axis, claw
Servo servos[2];
int servoWidth[2]={};
uint32_t lastServoContact=0;
IntervalTimer timer;
volatile int rate[5]={};
volatile uint32_t phase[5]={},steps[5]={};
volatile bool settle[5]={};
volatile uint32_t lastCommand=0,lastGood[5]={};
volatile bool moving=false;
volatile uint8_t fault=0;
char line[128];size_t used=0;bool overflow=false;
void servosOff(){
  noInterrupts();
  for(unsigned i=0;i<2;++i){
    if(servos[i].attached())servos[i].detach();
    digitalWrite(SERVO_PIN[i],LOW);servoWidth[i]=0;
  }
  interrupts();
}
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
bool encoderAttempt(uint8_t port,uint16_t &raw){
  Wire.beginTransmission(0x70);Wire.write((uint8_t)(1u<<port));
  bool ok=Wire.endTransmission()==0;delayMicroseconds(100);
  if(ok){ok=Wire.requestFrom((uint8_t)0x70,(uint8_t)1)==1;if(ok)ok=Wire.read()==(1<<port);}
  if(ok){Wire.beginTransmission(0x36);Wire.write((uint8_t)0x0C);ok=Wire.endTransmission(false)==0;}
  if(ok)ok=Wire.requestFrom((uint8_t)0x36,(uint8_t)2)==2;
  if(ok){raw=(Wire.read()&15)<<8;raw|=Wire.read();}
  Wire.beginTransmission(0x70);Wire.write((uint8_t)0);return Wire.endTransmission()==0&&ok;
}
bool encoder(uint8_t port,uint16_t &raw){
  const uint32_t begun=millis();
  for(unsigned attempt=0;attempt<3;++attempt){
    if(millis()-begun>=100)return false;
    bool ok=encoderAttempt(port,raw);
    if(millis()-begun>=100)return false;
    if(ok)return true;
    if(attempt<2)delay(1);
  }
  return false;
}
void command(){
  if(!strcmp(line,"PING"))Serial.println("READY WAYPOINT_V1 STEP=2,23,0,4,6 DIR=3,22,1,5,7 ENC=0,1,3,5,4 POS=1,0,1,1,1 HZ=355,933,2000,888,888 SERVO=15,14");
  else if(!strcmp(line,"STOP")){noInterrupts();halt(0);interrupts();lastServoContact=millis();Serial.println("OK STOP");}
  else if(!strcmp(line,"OFF")){noInterrupts();halt(0);interrupts();servosOff();Serial.println("OK OFF");}
  else if(!strcmp(line,"READ")){
    lastServoContact=millis();
    uint16_t raw[5]={};bool valid[5]={};uint32_t counts[5];
    for(unsigned i=0;i<5;++i){
      valid[i]=encoder(PORT[i],raw[i]);
      noInterrupts();counts[i]=steps[i];if(valid[i])lastGood[i]=millis();interrupts();
    }
    Serial.printf("FRAME fault=%u moving=%u",fault,moving);
    for(unsigned i=0;i<5;++i)Serial.printf(" v%u=%u r%u=%u s%u=%lu",i,valid[i],i,raw[i],i,(unsigned long)counts[i]);
    Serial.printf(" u0=%d u1=%d",servoWidth[0],servoWidth[1]);
    Serial.println();
  }else if(!strncmp(line,"SET ",4)){
    if(fault){Serial.println("ERROR latched_fault_STOP_required");return;}
    int next[5];char* save;strtok_r(line," ",&save);bool ok=true;
    for(unsigned i=0;i<5;++i){
      char* token=strtok_r(nullptr," ",&save);if(!token){ok=false;break;}
      char* end;long n=strtol(token,&end,10);
      if(end==token||*end||n<-MAX_HZ[i]||n>MAX_HZ[i]){ok=false;break;}next[i]=n;
    }
    int pulse[2]={};
    for(unsigned i=0;i<2&&ok;++i){
      char* token=strtok_r(nullptr," ",&save);if(!token){ok=false;break;}
      char* end;long n=strtol(token,&end,10);
      // Zero means leave this servo untouched, NOT move to angle zero.
      if(end==token||*end||(n!=0&&(n<500||n>2500))){ok=false;break;}pulse[i]=n;
    }
    if(!ok||strtok_r(nullptr," ",&save)){noInterrupts();halt(3);interrupts();Serial.println("ERROR five_rates_two_pulses_or_limits");return;}
    bool needsFeedback=false;
    for(unsigned i=0;i<5;++i)needsFeedback=needsFeedback||next[i]!=0;
    if(needsFeedback)for(unsigned i=0;i<5;++i)if(millis()-lastGood[i]>500){
      noInterrupts();halt(2);interrupts();Serial.println("ERROR stale_feedback");return;
    }
    noInterrupts();moving=false;
    for(unsigned i=0;i<5;++i){
      if((rate[i]<0)!=(next[i]<0)||(!rate[i]&&next[i])){settle[i]=true;phase[i]=0;}
      uint8_t level=next[i]>=0?POS_DIR[i]:1-POS_DIR[i];digitalWrite(DIR[i],level?HIGH:LOW);
      rate[i]=next[i];moving=moving||next[i]!=0;
    }
    for(unsigned i=0;i<2;++i)if(pulse[i]){
      if(!servos[i].attached())servos[i].attach(SERVO_PIN[i],500,2500);
      servos[i].writeMicroseconds(pulse[i]);servoWidth[i]=pulse[i];
    }
    lastCommand=lastServoContact=millis();interrupts();Serial.println("OK SET");
  }else Serial.println("ERROR unknown_command");
}
void setup(){
  for(unsigned i=0;i<5;++i){digitalWrite(STEP[i],LOW);pinMode(STEP[i],OUTPUT);digitalWrite(DIR[i],LOW);pinMode(DIR[i],OUTPUT);lastGood[i]=millis()-501;}
  for(unsigned i=0;i<2;++i){digitalWrite(SERVO_PIN[i],LOW);pinMode(SERVO_PIN[i],OUTPUT);}
  Serial.begin(115200);Wire.begin();Wire.setClock(100000);timer.begin(tick,50);
}
void loop(){
  if(millis()-lastServoContact>1500&&(servos[0].attached()||servos[1].attached()))servosOff();
  for(unsigned budget=0;budget<64&&Serial.available();++budget){
    char c=Serial.read();if(c=='\r')continue;
    if(c=='\n'){if(overflow){noInterrupts();halt(3);interrupts();Serial.println("ERROR long_line");}else{line[used]=0;command();}used=0;overflow=false;}
    else if(used<sizeof(line)-1&&!overflow)line[used++]=c;else overflow=true;
  }
}
