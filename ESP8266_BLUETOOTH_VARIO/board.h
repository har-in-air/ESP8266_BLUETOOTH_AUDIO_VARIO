#ifndef BOARD_H_
#define BOARD_H_

#define pinPGCC      0
#define pinSDA       5
#define pinSCL       4
#define pinDRDYInt   15
#define pinAudio 	 14
#define pinL9110Pwr  16
#define pinHM11Pwr   13
#define pinLED       12

#define BTN_PGCC()  (GPIP(pinPGCC) ? 1 : 0)

#endif
