#include <Arduino.h>
#include "cct.h"

static uint32_t CCTMarker;

static inline uint32_t cct_ccount(void) {
  uint32_t r;
  asm volatile ("rsr %0, ccount" : "=r"(r));
  return r;
  }

void cct_delayUs(uint32_t us) {
  volatile uint32_t waitCycleCount = (CCT_TICKS_PER_US * us ) + cct_ccount();
  do  {} while (cct_ccount()  < waitCycleCount);
  }


uint32_t  cct_get_intervalUs(uint32_t before, uint32_t after) {
  return  (before <= after ?
    ((after - before)+CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US :
    (after + (0xFFFFFFFF - before) + CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US);
}

float  cct_get_intervalSecs(uint32_t before, uint32_t after) {
  return  (before <= after ?
    (float)(after - before)/(float)SYSTEM_CLOCK_HZ :
    (float)(after + (0xFFFFFFFF - before))/(float)SYSTEM_CLOCK_HZ);
}

void cct_set_marker(void) {
  CCTMarker = cct_ccount();
  }

uint32_t cct_get_elapsedUs(void) {
  uint32_t now = cct_ccount();
  return  (CCTMarker <= now ?
    ((now - CCTMarker)+CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US :
    (now + (0xFFFFFFFF - CCTMarker) + CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US);
}

