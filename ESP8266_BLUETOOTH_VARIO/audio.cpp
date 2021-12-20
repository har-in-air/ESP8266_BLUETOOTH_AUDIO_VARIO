#include <Arduino.h>
#include "config.h"
#include "audio.h"

static int pinPWM_;

void audio_config(int pinPWM) {
#if (CFG_L9110S == true)
   	pinMode(pinL9110Pwr, OUTPUT); // enable/disable power to L9110 push-pull driver
	digitalWrite(pinL9110Pwr, 0); // disable
#endif    
    pinPWM_  = pinPWM;
    }

void audio_set_frequency(int32_t freqHz) {
  if (freqHz > 0) {
#if (CFG_L9110S == true)
    digitalWrite(pinL9110Pwr, 1);
#endif    
    tone(pinPWM_, freqHz, 10000);
    }
  else {
#if (CFG_L9110S == true)
    digitalWrite(pinL9110Pwr, 0);
#endif    
    noTone(pinPWM_);
    }
  }


void audio_generate_tone(int32_t freqHz, int ms) {
#if (CFG_L9110S == true)
    digitalWrite(pinL9110Pwr, 1);
#endif    
    tone(pinPWM_, freqHz, ms);
    delay(ms);
#if (CFG_L9110S == true)
    digitalWrite(pinL9110Pwr, 0);
#endif    
    }
