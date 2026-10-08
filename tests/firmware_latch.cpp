#include <cassert>
#include <cstring>
#include "latch.h"

void sample(StopLatch& latch, bool safe, bool reset, uint32_t now) {
  latch.query(now);
  latch.update(safe, reset, now);
}

int main() {
  StopLatch l;
  l.update(true, true, 0);  // held reset at power-on cannot arm
  for (uint32_t t=0; t<1000; t+=50) sample(l, true, true, t);
  assert(!l.armed);
  sample(l,true,false,1000);
  sample(l,true,true,1050);
  assert(l.armed);
  sample(l,false,true,1100);  // physical press / open wire
  assert(!l.armed);
  for(uint32_t t=1150;t<1800;t+=50) sample(l,true,true,t);
  assert(!l.armed);  // releasing mushroom with held reset is insufficient
  sample(l,true,false,1800);
  sample(l,true,true,1850);
  assert(l.armed);
  l.update(true,true,2100);  // exactly 250ms without host queries
  assert(!l.armed);
  sample(l,true,true,2150);
  assert(!l.armed);  // restored USB cannot rearm
  sample(l,true,false,2200);
  sample(l,true,true,2250);
  assert(l.armed);
  l.trip();  // malformed/overflowing host input
  sample(l,true,true,2300);
  assert(!l.armed);
  StopLatch short_pulse;
  sample(short_pulse,true,false,0);
  sample(short_pulse,true,true,50);
  for(uint32_t t=100;t<1000;t+=50) sample(short_pulse,true,true,t);
  assert(!short_pulse.armed);  // reset before stable safe circuit not deferred
  assert(wire_crc32("123456789")==0xcbf43926u);
  StopLatch wrap;
  sample(wrap,true,false,0xffffff00u);
  sample(wrap,true,false,0xffffff80u);
  sample(wrap,true,false,0x00000000u);
  sample(wrap,true,false,0x00000080u);
  sample(wrap,true,true,0x00000100u);
  assert(wrap.armed);  // firmware millisecond arithmetic is wrap safe
  return 0;
}
