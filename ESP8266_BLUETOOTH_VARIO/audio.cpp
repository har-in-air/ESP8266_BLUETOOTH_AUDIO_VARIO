#include <Arduino.h>
#include "board.h"
#include "audio.h"

static int pinPWM_;

#if 0
void audio_Config(int pinPWM) {
  pinPWM_       = pinPWM;
  analogWrite(pinPWM_, 0);
  }

void audio_SetFrequency(int32_t freqHz) {
	if (freqHz > 0) {
    digitalWrite(pinAudioEn, 1);
		analogWriteFreq(freqHz);
		analogWrite(pinPWM_, 512); // generate square wave with frequency fHz and 50% duty cycle (512/1023 ~= 0.5)
		}
	else {
    digitalWrite(pinAudioEn, 0);
		analogWrite(pinPWM_, 0); // turn off output
		}
	}


void audio_GenerateTone(int32_t freqHz, int ms) {
    audio_SetFrequency(freqHz);
    delay(ms);
    audio_SetFrequency(0);
    }

#else

void audio_Config(int pinPWM) {
  pinPWM_       = pinPWM;
  }

void audio_SetFrequency(int32_t freqHz) {
  if (freqHz > 0) {
    digitalWrite(pinAudioEn, 1);
    tone(pinPWM_, freqHz, 10000);
    }
  else {
    digitalWrite(pinAudioEn, 0);
    noTone(pinPWM_);
    }
  }


void audio_GenerateTone(int32_t freqHz, int ms) {
    digitalWrite(pinAudioEn, 1);
    tone(pinPWM_, freqHz, ms);
    delay(ms);
    digitalWrite(pinAudioEn, 0);
    }

#endif
