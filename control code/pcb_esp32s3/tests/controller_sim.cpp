// Test endpoint only. No GPIO, BLE, USB peripheral or real motor access.
#include "controller.h"
#include <chrono>
#include <iostream>
#include <poll.h>
#include <unistd.h>
struct Sim : Hardware {
 bool on=false;std::array<int,4> rates_{};
 bool external()const override{return true;} unsigned faults()const override{return 0;}
 bool enable()override{on=true;return true;}void disable()override{on=false;}
 bool rates(const std::array<int,4>&x)override{rates_=x;return true;}
 void current(unsigned,unsigned)override{}
};
int main(){
 Sim h;Controller c(h);LineReader reader;
 auto start=std::chrono::steady_clock::now();
 while(true){
  uint32_t now=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();
  c.tick(now);pollfd p{STDIN_FILENO,POLLIN,0};int ready=poll(&p,1,5);
  if(ready<0)return 1;
  if(ready>0 && (p.revents&POLLIN)){
   char bytes[128];int n=read(STDIN_FILENO,bytes,sizeof(bytes));if(n<=0)return 0;
   for(int i=0;i<n;i++)if(int r=reader.feed(bytes[i])){
    char reply[300];if(r>0)c.command(reader.data,Source::USB,now,reply,sizeof(reply));else snprintf(reply,sizeof(reply),"ERR LINE");
    std::cout<<reply<<'\n'<<std::flush;
   }
  }
 }
}
