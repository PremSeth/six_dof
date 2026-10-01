#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <string>
#include <sstream>
constexpr int LOW=0,HIGH=1,OUTPUT=1;
inline uint32_t fakeMillis=1000;
inline int pinLevels[40]={};
inline uint32_t millis(){return fakeMillis;}
inline void delay(unsigned ms){fakeMillis+=ms;}
inline void delayMicroseconds(unsigned){}
inline void digitalWrite(unsigned pin,int value){pinLevels[pin]=value;}
inline void pinMode(unsigned,int){}
inline void noInterrupts(){}
inline void interrupts(){}
struct SerialMock {
  std::string output;
  void begin(unsigned){}
  int available(){return 0;}
  char read(){return 0;}
  template<class T>void println(T value){std::ostringstream os;os<<value;output+=os.str()+"\n";}
  void println(){output+='\n';}
  void printf(const char* format,...){char buffer[2048];va_list args;va_start(args,format);vsnprintf(buffer,sizeof(buffer),format,args);va_end(args);output+=buffer;}
};
inline SerialMock Serial;
