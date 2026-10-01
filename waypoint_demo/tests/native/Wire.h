#pragma once
#include <cstdint>
struct WireMock{
  int address=0,last=0,mask=0,index=0;bool failing=false;
  void begin(){} void setClock(unsigned){}
  void beginTransmission(int a){address=a;}
  void write(uint8_t v){last=v;}
  int endTransmission(bool=true){if(failing)return 2;if(address==0x70)mask=last;return 0;}
  int requestFrom(uint8_t a,uint8_t n){address=a;index=0;return failing?0:n;}
  int read(){return address==0x70?mask:(index++==0?3:232);}
};
inline WireMock Wire;
