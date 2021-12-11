#ifndef AUDIO_H_
#define AUDIO_H_

void audio_config(int pinPWM);
void audio_set_frequency(int freqHz);
void audio_generate_tone(int freqHz, int ms);

#endif