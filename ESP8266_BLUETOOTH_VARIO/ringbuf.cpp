#include <Arduino.h>
#include "config.h"
#include "ringbuf.h"

static RINGBUF RingBuf;

void ringbuf_Init() {
   memset(RingBuf.buffer,0,RINGBUF_SIZE);
   RingBuf.head = RINGBUF_SIZE-1;
   }

void ringbuf_AddSample(float sample) {
   RingBuf.head++;
   if (RingBuf.head >= RINGBUF_SIZE) RingBuf.head = 0;
   RingBuf.buffer[RingBuf.head] = sample;
   }


float ringbuf_AverageOldestSamples(int numSamples) {
   int index = RingBuf.head+1; // oldest Sample
   float accum = 0.0;
   for (int count = 0; count < numSamples; count++) {
      if (index >= RINGBUF_SIZE) index = 0;
      accum += RingBuf.buffer[index];
      index++;
      }
   return accum/numSamples;
   }   

