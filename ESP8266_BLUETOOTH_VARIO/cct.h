#ifndef CCT_H_
#define CCT_H_

#define  CCT_TICKS_PER_US 	80 // 80MHz clock
#define  SYSTEM_CLOCK_HZ 	80000000

void 		cct_delayUs(uint32_t us);
uint32_t  	cct_get_intervalUs(uint32_t before, uint32_t after);
float  		cct_get_intervalSecs(uint32_t before, uint32_t after);
void 		cct_set_marker(void);
uint32_t 	cct_get_elapsedUs(void);

#endif