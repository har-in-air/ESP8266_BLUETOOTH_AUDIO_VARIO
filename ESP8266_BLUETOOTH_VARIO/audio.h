#ifndef AUDIO_H_
#define AUDIO_H_

void audio_Config(int pinPWM);
void audio_SetFrequency(int freqHz);
void audio_GenerateTone(int freqHz, int ms);

#endif