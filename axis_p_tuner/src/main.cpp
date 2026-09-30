#include <Arduino.h>
#include <IntervalTimer.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>

// ONLY base, J1, J2. No wrist or servo pins are configured or commanded.
constexpr uint8_t STEP[3]={2,23,0}, DIR[3]={3,22,1};
constexpr uint8_t PORT[3]={0,1,3}, POS[3]={1,0,1};
constexpr int MAX_HZ[3]={88,666,1333}; // 5,5,10 output degrees/s
IntervalTimer timer;
volatile int selected=-1,rate=0;
volatile uint32_t phase=0,count=0,lastCommand=0,lastGood=0;
volatile uint8_t fault=0;
volatile bool settling=false;
char line[96];unsigned used=0;bool overflow=false;

void halt(uint8_t reason){
  rate=0;phase=0;fault=reason;
  for(unsigned i=0;i<3;++i)digitalWrite(STEP[i],LOW);
}
void tick(){
  if(!rate||selected<0)return;
  if(millis()-lastCommand>300){halt(1);return;}
  if(millis()-lastGood>300){halt(2);return;}
  if(settling){settling=false;return;}
  phase+=abs(rate);
  if(phase>=20000){
    phase-=20000;digitalWrite(STEP[selected],HIGH);delayMicroseconds(5);
    digitalWrite(STEP[selected],LOW);count+=rate<0?UINT32_MAX:1u;
  }
}
bool integer(const char* text,long &value){
  if(!text||!*text)return false;
  char* end;value=strtol(text,&end,10);return end!=text&&!*end;
}
bool encoder(uint16_t &raw,const char* &stage,int &code,unsigned &attempts){
  uint32_t begun=millis();
  auto check=[&](const char* name,int actual,int expected){
    if(actual==expected)return true;
    stage=name;code=actual;return false;
  };
  for(attempts=0;attempts<3;){
    if(millis()-begun>=100)break;
    ++attempts;
    Wire.beginTransmission(0x70);Wire.write(uint8_t(1u<<PORT[selected]));
    bool ok=check("mux_select",Wire.endTransmission(),0);
    if(ok)delayMicroseconds(100);
    if(ok)ok=check("mux_bytes",Wire.requestFrom(uint8_t(0x70),uint8_t(1)),1);
    if(ok)ok=check("mux_mask",Wire.read(),1u<<PORT[selected]);
    if(ok){
      Wire.beginTransmission(0x36);Wire.write(uint8_t(0x0C));
      ok=check("encoder_register",Wire.endTransmission(false),0);
    }
    if(ok)ok=check("encoder_bytes",Wire.requestFrom(uint8_t(0x36),uint8_t(2)),2);
    if(ok){raw=(Wire.read()&15)<<8;raw|=Wire.read();}
    Wire.beginTransmission(0x70);Wire.write(uint8_t(0));
    int deselect=Wire.endTransmission();
    if(ok)ok=check("mux_deselect",deselect,0);
    if(millis()-begun>=100){stage="read_budget";return false;}
    if(ok)return true;
    if(attempts<3)delay(1);
  }
  return false;
}
void command(){
  if(!strcmp(line,"PING")){
    Serial.println("READY AXIS_P_V1 STEP=2,23,0 DIR=3,22,1 ENC=0,1,3 POS=1,0,1 HZ=88,666,1333");
  }else if(!strcmp(line,"STOP")){
    noInterrupts();halt(0);interrupts();Serial.println("OK STOP");
  }else if(!strncmp(line,"SELECT ",7)){
    long n;
    if(!integer(line+7,n)||n<0||n>2){Serial.println("ERROR axis_0_to_2");return;}
    if(rate){Serial.println("ERROR moving_STOP_first");return;}
    noInterrupts();halt(0);selected=n;count=0;lastGood=millis()-301;interrupts();
    Serial.printf("OK SELECT %ld\n",n);
  }else if(!strcmp(line,"READ")){
    if(selected<0){Serial.println("ERROR select_axis_first");return;}
    uint16_t raw=0;const char* stage="none";int code=0;unsigned attempts=0;
    uint32_t started=millis();bool ok=encoder(raw,stage,code,attempts);
    if(!ok){
      noInterrupts();halt(2);interrupts();
      Serial.printf("ERROR encoder_read_failed port=%u attempts=%u elapsed_ms=%lu stage=%s code=%d\n",
        PORT[selected],attempts,(unsigned long)(millis()-started),stage,code);return;
    }
    noInterrupts();lastGood=millis();uint32_t steps=count;int hz=rate;uint8_t f=fault;interrupts();
    Serial.printf("FRAME raw=%u steps=%lu hz=%d fault=%u\n",raw,(unsigned long)steps,hz,f);
  }else if(!strncmp(line,"VEL ",4)){
    long n;
    if(selected<0||!integer(line+4,n)||n<-MAX_HZ[selected]||n>MAX_HZ[selected]){
      noInterrupts();halt(3);interrupts();Serial.println("ERROR axis_or_rate_limit");return;
    }
    if(fault){Serial.println("ERROR latched_fault_STOP_required");return;}
    if(millis()-lastGood>300){Serial.println("ERROR stale_encoder");return;}
    noInterrupts();
    if((rate<0)!=(n<0)||(!rate&&n)){settling=true;phase=0;}
    digitalWrite(DIR[selected],n>=0?POS[selected]:1-POS[selected]);
    rate=n;lastCommand=millis();interrupts();Serial.println("OK VEL");
  }else Serial.println("ERROR unknown_command");
}
void setup(){
  for(unsigned i=0;i<3;++i){
    digitalWrite(STEP[i],LOW);pinMode(STEP[i],OUTPUT);
    digitalWrite(DIR[i],LOW);pinMode(DIR[i],OUTPUT);
  }
  Serial.begin(115200);Wire.begin();Wire.setClock(100000);timer.begin(tick,50);
}
void loop(){
  for(unsigned budget=0;budget<64&&Serial.available();++budget){
    char c=Serial.read();if(c=='\r')continue;
    if(c=='\n'){
      if(overflow){noInterrupts();halt(3);interrupts();Serial.println("ERROR long_line");}
      else{line[used]=0;command();}
      used=0;overflow=false;
    }else if(!overflow){if(used<sizeof(line)-1)line[used++]=c;else overflow=true;}
  }
}
