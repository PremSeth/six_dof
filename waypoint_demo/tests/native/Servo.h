#pragma once
struct Servo{
  bool on=false;int pin=-1,width=0;
  bool attached(){return on;}
  void attach(int p,int,int){pin=p;on=true;}
  void detach(){on=false;}
  void writeMicroseconds(int us){width=us;}
};
