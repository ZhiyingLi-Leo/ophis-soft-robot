#pragma once
#include <array>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include "board_config.h"

enum class Source : uint8_t {None, USB, BLE};
struct Hardware {
  virtual ~Hardware() = default;
  virtual bool external() const = 0;
  virtual unsigned faults() const = 0;
  virtual bool enable() = 0;
  virtual void disable() = 0;
  virtual bool rates(const std::array<int,4>& speed) = 0;
  virtual void current(unsigned axis, unsigned ma) = 0;
};

class Controller {
 public:
  explicit Controller(Hardware& h) : hw(h) { currents.fill(board::DEFAULT_CURRENT_MA); }
  std::array<int,4> speed{};
  std::array<unsigned,4> currents{};
  Source owner=Source::None;
  bool armed=false, latched=false;
  const char* event="BOOT";
  uint32_t lastCommand=0,uptime=0;
  void halt(const char* why, bool release=false) {
    speed.fill(0); hw.rates(speed); event=why;
    if(release) owner=Source::None;
  }
  void off(const char* why, bool latch=false) {
    halt(why,true); hw.disable(); armed=false; latched=latch;
  }
  void disconnected(Source s) { if(owner==s) halt("DISCONNECTED",true); }
  void tick(uint32_t now) {
    uptime=now;
    if(armed && !hw.external()) { off("POWER_LOST"); return; }
    if(armed && hw.faults()) { off("DRIVER_FAULT",true); return; }
    if(owner!=Source::None && uint32_t(now-lastCommand)>=board::WATCHDOG_MS)
      halt("TIMEOUT",true); // Hold electrically; require a new A1 to resume.
  }
  static bool integer(const char*& p,int& out) {
    if(!(*p=='-' || (*p>='0'&&*p<='9'))) return false;
    errno=0; char* end=nullptr; long v=strtol(p,&end,10);
    if(end==p || errno==ERANGE || v<INT_MIN || v>INT_MAX) return false;
    p=end;out=int(v);return true;
  }
  bool active(Source source) const {return armed && owner==source && source!=Source::None;}
  void status(char* out,size_t n) const {
    snprintf(out,n,"STATUS {\"board\":\"PCB1_S3\",\"uptime_ms\":%lu,\"armed\":%d,\"owner\":%d,\"external\":%d,\"fault\":%u,\"latched\":%d,\"speed\":[%d,%d,%d,%d],\"current_ma\":[%u,%u,%u,%u],\"event\":\"%s\"}",static_cast<unsigned long>(uptime),armed,int(owner),hw.external(),hw.faults(),latched,speed[0],speed[1],speed[2],speed[3],currents[0],currents[1],currents[2],currents[3],event);
  }
  void command(const char* cmd,Source source,uint32_t now,char* out,size_t n) {
    tick(now);
    auto reply=[&](const char* x){snprintf(out,n,"%s",x);};
    if(!strcmp(cmd,"?")) {
      // Legacy V reply only represents a common speed; use STATUS for independent axes.
      if(speed[0]==speed[1]&&speed[0]==speed[2]&&speed[0]==speed[3]) snprintf(out,n,"V%d",speed[0]);
      else reply("ERR INDEPENDENT_USE_STATUS"); return;
    }
    if(!strcmp(cmd,"STATUS")) {status(out,n);return;}
    if(!strcmp(cmd,"INFO")) {reply("INFO PCB1_S3 1.0 microsteps=64 max_sps=16000 watchdog_ms=1000");return;}
    if(!strcmp(cmd,"S") || !strcmp(cmd,"V0")) {halt("STOP");reply("OK V0");return;}
    if(!strcmp(cmd,"E")) {off("ESTOP",true);reply("OK ESTOP");return;}
    if(!strcmp(cmd,"A0")) {off("DISARM",latched);reply("OK A0");return;}
    if(!strcmp(cmd,"CLEAR")) {
      if(armed || hw.faults()) {reply("ERR FAULT_ACTIVE_OR_ARMED");return;}
      latched=false;event="CLEARED";reply("OK CLEAR");return;
    }
    if(!strcmp(cmd,"A1")) {
      if(owner!=Source::None && owner!=source) {reply("ERR BUSY");return;}
      if(latched) {reply("ERR LATCHED");return;}
      if(!hw.external()) {reply("ERR NO_EXTERNAL_POWER");return;}
      if(!armed && !hw.enable()) {off("ARM_FAILED",true);reply("ERR DRIVER_FAULT");return;}
      armed=true;owner=source;lastCommand=now;event="ARMED";reply("OK A1");return;
    }
    if(!strcmp(cmd,"H")) {
      if(!active(source)) {reply("ERR NOT_OWNER");return;}
      lastCommand=now;reply("OK H");return;
    }
    std::array<int,4> next=speed;const char* p=cmd+1;int v=0,axis=0;
    if(cmd[0]=='V') {
      if(!integer(p,v)||*p) {reply("ERR FORMAT");return;}
      if(v < -board::MAX_SPEED || v>board::MAX_SPEED) {reply("ERR RANGE");return;}
      next.fill(v);
    } else if(cmd[0]=='M') {
      if(!integer(p,axis)||*p++!=','||!integer(p,v)||*p) {reply("ERR FORMAT");return;}
      if(axis<1||axis>4||v < -board::MAX_SPEED||v>board::MAX_SPEED) {reply("ERR RANGE");return;}
      next[axis-1]=v;
    } else if(cmd[0]=='W') {
      for(unsigned i=0;i<4;i++) {
        if(!integer(p,v)||(i<3 ? *p++!=',' : *p!='\0')) {reply("ERR FORMAT");return;}
        if(v < -board::MAX_SPEED||v>board::MAX_SPEED) {reply("ERR RANGE");return;}
        next[i]=v;
      }
    } else if(cmd[0]=='I') {
      if(!integer(p,axis)||*p++!=','||!integer(p,v)||*p) {reply("ERR FORMAT");return;}
      if(axis<1||axis>4||v<100||v>int(board::MAX_CURRENT_MA)) {reply("ERR RANGE");return;}
      if(!active(source)) {reply("ERR NOT_OWNER");return;}
      for(int s:speed) if(s) {reply("ERR STOP_FIRST");return;}
      hw.current(axis-1,unsigned(v));currents[axis-1]=unsigned(v);lastCommand=now;reply("OK I");return;
    } else {reply("ERR COMMAND");return;}
    if(!active(source)) {reply("ERR ARM_REQUIRED");return;}
    if(!hw.rates(next)) {off("PULSE_ERROR",true);reply("ERR PULSE");return;}
    speed=next;lastCommand=now;event="COMMAND";
    if(cmd[0]=='V')snprintf(out,n,"OK V%d",v);else reply("OK AXES");
  }
 private: Hardware& hw;
};

struct LineReader {
  char data[96]{};unsigned size=0;bool dropping=false;
  // Return 1 for a complete line, -1 for an overlong/invalid line, 0 for incomplete.
  int feed(char c) {
    if(c=='\r') return 0;
    if(c=='\n') {
      if(dropping){size=0;dropping=false;return -1;}
      data[size]='\0';unsigned old=size;size=0;return old?1:0;
    }
    if(static_cast<unsigned char>(c)<32 || static_cast<unsigned char>(c)>126) dropping=true;
    if(!dropping && size<sizeof(data)-1)data[size++]=c;else dropping=true;
    return 0;
  }
};
