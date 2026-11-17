#include <cassert>
#include <iostream>
#include <string>
#include "controller.h"
#include "pulse_pattern.h"
struct Fake : Hardware {
 bool power=true,on=false,fail=false;unsigned fault=0;std::array<int,4> rate{};
 bool external()const override{return power;}
 unsigned faults()const override{return fault;}
 bool enable()override{on=power&&!fault;return on;}
 void disable()override{on=false;rate.fill(0);}
 bool rates(const std::array<int,4>&r)override{rate=r;return !fail;}
 void current(unsigned,unsigned)override{}
};
std::string cmd(Controller&c,const char*s,Source src=Source::USB,uint32_t now=0){char out[300];c.command(s,src,now,out,sizeof(out));return out;}
int main(){
 Fake h;Controller c(h);
 assert(cmd(c,"V400")=="ERR ARM_REQUIRED");assert(!h.on);
 h.power=false;assert(cmd(c,"A1")=="ERR NO_EXTERNAL_POWER");h.power=true;
 assert(cmd(c,"A1")=="OK A1");assert(h.on);
 assert(cmd(c,"V400")=="OK V400");assert((h.rate==std::array<int,4>({400,400,400,400}))); 
 assert(cmd(c,"A1",Source::BLE)=="ERR BUSY");assert(cmd(c,"M2,800",Source::BLE)=="ERR ARM_REQUIRED");
 assert(cmd(c,"S",Source::BLE)=="OK V0");assert(h.on);assert(c.owner==Source::USB);
 assert(cmd(c,"M2,-800")=="OK AXES");assert((h.rate==std::array<int,4>({0,-800,0,0}))); 
 assert(cmd(c,"?")=="ERR INDEPENDENT_USE_STATUS");
 assert(cmd(c,"I1,400")=="ERR STOP_FIRST");
 assert(cmd(c,"W1,2,3,99999")=="ERR RANGE");assert(c.speed[1]==-800);
 assert(cmd(c,"W1,2,3,")=="ERR FORMAT");assert(c.speed[1]==-800);
 assert(cmd(c,"W-100,200,300,-400")=="OK AXES");
 assert(cmd(c,"S")=="OK V0");assert(cmd(c,"I4,450")=="OK I");assert(c.currents[3]==450);
 assert(cmd(c,"I4,800")=="ERR RANGE");assert(cmd(c,"V16001")=="ERR RANGE");
 for(const char*s:{"V","V+4","V--4","V4x","M0,2","M5,2","V999999999999999999999999999","","A1x"})assert(cmd(c,s).find("ERR")==0);
 cmd(c,"V400",Source::USB,100);c.tick(1099);assert(c.speed[0]==400);c.tick(1100);assert(c.speed[0]==0&&h.on&&c.owner==Source::None);
 assert(cmd(c,"H",Source::USB,1100)=="ERR NOT_OWNER");assert(cmd(c,"V400",Source::USB,1100)=="ERR ARM_REQUIRED");
 cmd(c,"A1",Source::BLE,1100);cmd(c,"V400",Source::BLE,1100);c.disconnected(Source::USB);assert(c.speed[0]==400);
 c.disconnected(Source::BLE);assert(c.speed[0]==0&&h.on&&c.owner==Source::None);
 cmd(c,"A1",Source::USB,1200);cmd(c,"V400",Source::USB,1200);h.power=false;c.tick(1201);assert(!c.armed&&!h.on);h.power=true;
 cmd(c,"A1",Source::USB,1202);h.fault=4;c.tick(1203);assert(c.latched&&!h.on);assert(cmd(c,"A1")=="ERR LATCHED");assert(cmd(c,"CLEAR")=="ERR FAULT_ACTIVE_OR_ARMED");h.fault=0;
 assert(cmd(c,"CLEAR")=="OK CLEAR");cmd(c,"A1");cmd(c,"E");assert(c.latched&&!h.on);cmd(c,"A0");assert(c.latched);cmd(c,"CLEAR");cmd(c,"A1");h.fail=true;assert(cmd(c,"V400")=="ERR PULSE");assert(!h.on&&c.latched);
 Fake wrap;Controller cw(wrap);cmd(cw,"A1",Source::USB,UINT32_MAX-400);cmd(cw,"V400",Source::USB,UINT32_MAX-400);cw.tick(100);assert(cw.speed[0]==400);cw.tick(700);assert(cw.speed[0]==0);
 LineReader lines;for(int i=0;i<100;i++)assert(lines.feed('x')==0);for(char ch:std::string("V800"))lines.feed(ch);assert(lines.feed('\n')==-1);assert(lines.feed('S')==0);assert(lines.feed('\n')==1);assert(std::string(lines.data)=="S");
 // Exhaust the entire accepted speed range: no zero halfword/end marker, exact period.
 for(unsigned speed=1;speed<=16000;speed++){
   PulseHalf halves[64];auto n=pulsePattern(speed,halves);assert(n>0&&n<=64&&n%2==0);unsigned total=0,high=0;
   for(unsigned i=0;i<n;i++){assert(halves[i].ticks>0&&halves[i].ticks<=32767);total+=halves[i].ticks;if(halves[i].high)high+=halves[i].ticks;}
   assert(high==6);assert(total==(2000000U+speed/2)/speed);
 }
 std::cout<<"PASS: protocol, 4-axis updates, ownership, power, faults, stop/hold, timeout/wrap, overflow, all16000 pulse periods\n";
}
