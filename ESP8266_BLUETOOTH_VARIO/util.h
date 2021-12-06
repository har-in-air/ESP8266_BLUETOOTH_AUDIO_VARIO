#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>

typedef union _un16 {
  uint16_t u;
  int16_t i;
  uint8_t b[2];
} un16;

typedef union _un32 {
  uint32_t  u;
  int32_t   i;
  uint16_t  w[2];
  uint8_t   b[4];
} un32;

#ifndef TRUE
#define TRUE    1
#endif
#ifndef FALSE
#define FALSE    0
#endif

#define OUT	1
#define IN	0


#define MIN(x,y)                 ((x) < (y) ? (x) : (y))
#define MAX(x,y)                 ((x) > (y) ? (x) : (y))
#define ABS(x)                 ((x) < 0 ? -(x) : (x))
#define CLAMP(x,mn,mx)       {if (x <= (mn)) x = (mn); else if (x >= (mx)) x = (mx);}
#define CORE(x,t)              {if (ABS(x) <= (t)) x = 0;}
#define MCORE(x,t)              {if (x > (t)) x -= (t); else if (x < -(t)) x += (t); else x = 0;}
#define CORRECT(x,mx,mn)  		(((float)(x-mn)/(float)(mx-mn)) - 0.5f)

//#define M_PI 		  3.1415927f
//#define RAD_TO_DEG(r)   ((r)*57.29577951f)
//#define DEG_TO_RAD(d)   ((d)*0.017453292f)

#define _180_DIV_PI         57.295779f
#define PI_DIV_180          0.017453292f

#define NOP() { asm volatile ("nop"); }


#endif
