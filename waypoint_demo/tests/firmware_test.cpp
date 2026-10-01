#include <cassert>
#include <cstring>
#include "../src/main.cpp"

void call(const char* text){Serial.output.clear();strncpy(line,text,sizeof(line));command();}
int main(){
  setup();
  for(unsigned i=0;i<5;++i){assert(rate[i]==0);assert(pinLevels[STEP[i]]==LOW);}
  assert(!servos[0].attached()&&!servos[1].attached());
  call("SET 1 0 0 0 0 0 0");assert(!moving&&fault==2);
  call("STOP");
  call("PING");assert(Serial.output.find("HZ=355,933,2000,1777,1777 SERVO=15,14")!=std::string::npos);
  call("READ");assert(!moving);assert(Serial.output.find("u0=0 u1=0")!=std::string::npos);
  call("SET 0 0 0 0 0 1500 1167");assert(Serial.output=="OK SET\n");
  assert(servos[0].pin==15&&servos[1].pin==14);
  assert(servoWidth[0]==1500&&servoWidth[1]==1167);
  call("SET 0 0 0 0 0 0 0");assert(servoWidth[0]==1500&&servoWidth[1]==1167);
  call("STOP");assert(servos[0].attached()); // STOP holds; OFF releases.
  call("SET 356 0 0 0 0 2000 2000");assert(fault==3&&!moving);assert(servoWidth[0]==1500);
  call("STOP");call("SET 0 0 0 0 0 499 1500");assert(fault==3);assert(servoWidth[0]==1500);
  call("STOP");call("READ");call("SET 0 -933 0 0 0 0 0");assert(rate[1]==-933);assert(pinLevels[22]==HIGH);
  for(int i=0;i<200;++i)tick();
  assert(steps[1]!=0);for(int i:{0,2,3,4})assert(steps[i]==0);
  fakeMillis+=301;tick();assert(!moving&&fault==1);
  call("SET 0 0 1 0 0 0 0");assert(Serial.output.find("latched_fault")!=std::string::npos);
  call("STOP");call("READ");call("SET 355 933 2000 1777 1777 1500 1167");assert(moving);
  fakeMillis+=501;lastCommand=fakeMillis;tick();assert(!moving&&fault==2);
  call("STOP");call("READ");call("SET 0 0 0 0 0 1500 1500");
  fakeMillis+=1501;loop();assert(!servos[0].attached()&&!servos[1].attached());
  call("OFF");assert(servoWidth[0]==0&&servoWidth[1]==0&&!moving);
  call("SET 0 0 0 1778 0 0 0");assert(fault==3&&!moving);
  call("STOP");
  Wire.failing=true;call("READ");assert(Serial.output.find("v0=0")!=std::string::npos);
  call("SET 1 0 0 0 0 0 0");assert(!moving&&Serial.output.find("stale_feedback")!=std::string::npos);
  return 0;
}
