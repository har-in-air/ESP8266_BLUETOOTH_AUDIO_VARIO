#ifndef BOARD_H_
#define BOARD_H_

#define pinPgmConfCalBtn 0
#define pinSDA       5
#define pinSCL       4
#define pinDRDYInt   15
#define pinAudio 	   14
#define pinAudioEn   16
#define pinBTSw      13
#define pinLED       12

#define BTN_PGCC()  (GPIP(pinPgmConfCalBtn) ? 1 : 0)

#endif
