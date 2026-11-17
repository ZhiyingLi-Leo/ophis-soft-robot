#pragma once
#include <stdint.h>
// An entire period fits in 32 RMT words at 2MHz, including a 1Hz period.
struct PulseHalf {uint16_t ticks;bool high;};
inline unsigned pulsePattern(unsigned sps,PulseHalf (&halves)[64]) {
  if(!sps || sps>16000)return 0;
  const uint32_t period=(2000000U+sps/2)/sps;
  unsigned n=0;halves[n++]={6,true}; // 3us high, longer than DRV8434 minimum.
  uint32_t low=period-6;
  while(low){uint16_t part=low>32767?32767:uint16_t(low);halves[n++]={part,false};low-=part;}
  if(n%2){ // Keep both halfwords nonzero; a zero duration is an end marker.
    for(unsigned i=1;i<n;i++)if(halves[i].ticks>1){
      halves[n++]={1,false};--halves[i].ticks;break;
    }
  }
  return n;
}
