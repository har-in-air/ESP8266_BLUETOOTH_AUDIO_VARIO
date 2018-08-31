#include <Arduino.h>
#include "cct.h"

static uint32_t cctMarker;


static inline uint32_t cct_ccount(void) {
  uint32_t r;
  asm volatile ("rsr %0, ccount" : "=r"(r));
  return r;
  }

void cct_DelayUs(uint32_t us) {
  volatile uint32_t waitCycleCount = (CCT_TICKS_PER_US * us ) + cct_ccount();
  do  {} while (cct_ccount()  < waitCycleCount);
  }


uint32_t  cct_IntervalUs(uint32_t before, uint32_t after) {
  return  (before <= after ?
    ((after - before)+CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US :
    (after + (0xFFFFFFFF - before) + CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US);
}

float  cct_IntervalSecs(uint32_t before, uint32_t after) {
  return  (before <= after ?
    (float)(after - before)/(float)SYSTEM_CLOCK_HZ :
    (float)(after + (0xFFFFFFFF - before))/(float)SYSTEM_CLOCK_HZ);
}

void cct_SetMarker(void) {
  cctMarker = cct_ccount();
}

uint32_t cct_ElapsedTimeUs(void) {
  uint32_t now = cct_ccount();
  return  (cctMarker <= now ?
    ((now - cctMarker)+CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US :
    (now + (0xFFFFFFFF - cctMarker) + CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US);
}

