#ifndef RINGBUF_H_
#define RINGBUF_H_

#define RINGBUF_SIZE    15

typedef struct RINGBUF_ {
   int head;
   float buffer[RINGBUF_SIZE];
} RINGBUF;

void ringbuf_Init();
void ringbuf_AddSample(float sample);
float ringbuf_AverageOldestSamples(int numSamples);

#endif
