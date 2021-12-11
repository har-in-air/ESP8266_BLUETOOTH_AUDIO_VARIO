#include <Arduino.h>
#include "config.h"
#include "btserial.h"

static uint8_t btserial_nmea_checksum(const char *szNMEA){
   const char* sz = &szNMEA[1]; // skip leading '$'
   uint8_t cksum = 0;
   while ((*sz) != 0 && (*sz != '*')) {
      cksum ^= (uint8_t) *sz;
      sz++;
      }
   return cksum;
   }

   
void btserial_transmit_LK8EX1(int32_t altm, int32_t cps, float batVoltage) {
   char szmsg[40];
   sprintf(szmsg, "$LK8EX1,999999,%d,%d,99,%.1f*", altm, cps, batVoltage);
   uint8_t cksum = btserial_nmea_checksum(szmsg);
   char szcksum[5];
   sprintf(szcksum,"%02X\r\n", cksum);
   strcat(szmsg, szcksum);
   Serial1.print(szmsg);
  }
