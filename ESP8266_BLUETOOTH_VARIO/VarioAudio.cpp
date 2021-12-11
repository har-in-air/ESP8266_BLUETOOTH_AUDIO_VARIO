#include <Arduino.h>
#include "config.h"
#include "nvd.h"
#include "audio.h"
#include "util.h"
#include "VarioAudio.h"


void VarioAudio::Config() {
    sinkToneCps_    =  (int32_t)Nvd.par.cfg.vario.sinkThresholdCps;
    climbToneCps_   =  (int32_t)Nvd.par.cfg.vario.climbThresholdCps;
    zeroesToneCps_  =  (int32_t)Nvd.par.cfg.vario.zeroThresholdCps;
    crossoverCps_   =  (int32_t)Nvd.par.cfg.vario.crossoverCps;
    #ifdef VARIO_DEBUG  
    dbg_printf(("climbToneCps = %d\r\n", climbToneCps_));
    dbg_printf(("zeroesToneCps = %d\r\n", zeroesToneCps_));
    dbg_printf(("sinkToneCps = %d\r\n", sinkToneCps_));
    dbg_printf(("crossoverCps = %d\r\n", crossoverCps_));
    #endif  
    varioState_ 		= VARIO_STATE_QUIET;
    beepPeriodTicks_	= 0;
    beepEndTick_ 		= 0;
    varioCps_ 			= 0;
    freqHz_  			= 0;
    }


void VarioAudio::Beep(int32_t nCps) {
  int32_t newFreqHz = 0;
  // generate new beep/tone only if 
  if (
    // current beep/tone has ended, OR
    (beepPeriodTicks_ <= 0)  ||
#ifdef VARIO_INTERRUPT_BEEPS    
    // at least half current beep/tone is over AND there is a significant change in climb/sink, OR
    ((tick_ >= beepPeriodTicks_/2) && (ABS(nCps - varioCps_) > VARIO_DISCRIMINATION_THRESHOLD_CPS)) || 
#endif    
    // climb threshold exceeded
    ((nCps >= climbToneCps_) && (varioCps_ < climbToneCps_)) 
     ) {
    varioCps_ = nCps;
    // if sinking significantly faster than glider sink rate in still air, generate warning sink tone
    if (varioCps_ <= sinkToneCps_) {
      varioState_ = VARIO_STATE_SINK;
			tick_ = 0;
      if (varioCps_ <= -VARIO_MAX_CPS) {
        beepPeriodTicks_ = 8;
        beepEndTick_ = 8;
    		newFreqHz = offScaleLoTone_[0];
    		freqHz_ = newFreqHz;
        audio_SetFrequency(freqHz_);
        }
      else {
        beepPeriodTicks_ = 40; // sink indicated with descending frequency beeps with long on-times
        beepEndTick_  = 30;
        // descending tone starts at higher frequency for higher sink rate
        newFreqHz = VARIO_SPKR_MAX_FREQHZ/2 + ((varioCps_ + VARIO_MAX_CPS)*(VARIO_SPKR_MIN_FREQHZ + 600 - VARIO_SPKR_MAX_FREQHZ/2))/(sinkToneCps_ + VARIO_MAX_CPS);
        CLAMP(newFreqHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ);
        freqHz_ = newFreqHz;
        audio_SetFrequency(freqHz_);
        }
      }
    //if climbing, generate beeps
    else {
      if (varioCps_ >= climbToneCps_) {
        varioState_ = VARIO_STATE_CLIMB;
        tick_ = 0;
        if (varioCps_ >= VARIO_MAX_CPS) {
          beepPeriodTicks_ = 8;
          beepEndTick_ = 8;
          newFreqHz = offScaleHiTone_[0];
          freqHz_ = newFreqHz;
          audio_SetFrequency(freqHz_);
          }
        else {
          int index = varioCps_/100;
          if (index > 9) index = 9;
          beepPeriodTicks_ = beepTbl_[index].periodTicks;
          beepEndTick_ = beepTbl_[index].endTick;
          if (varioCps_ > crossoverCps_) {
            newFreqHz = VARIO_CROSSOVER_FREQHZ + ((varioCps_ - crossoverCps_)*(VARIO_SPKR_MAX_FREQHZ - VARIO_CROSSOVER_FREQHZ))/(VARIO_MAX_CPS - crossoverCps_);
            }
          else {
            newFreqHz = VARIO_SPKR_MIN_FREQHZ + ((varioCps_ - zeroesToneCps_)*(VARIO_CROSSOVER_FREQHZ - VARIO_SPKR_MIN_FREQHZ))/(crossoverCps_ - zeroesToneCps_);
            }
          CLAMP(newFreqHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ);
          freqHz_ = newFreqHz;
          audio_SetFrequency(freqHz_);
          }
        }
      else   // in "zeroes" band, indicate with a short pulse and long interval
      if (varioCps_ >= zeroesToneCps_) {
        varioState_ = VARIO_STATE_ZEROES;
    		tick_ = 0;
    		beepPeriodTicks_ = 30;
    		beepEndTick_ = 2;
    		newFreqHz = VARIO_SPKR_MIN_FREQHZ + ((varioCps_ - zeroesToneCps_)*(VARIO_CROSSOVER_FREQHZ - VARIO_SPKR_MIN_FREQHZ))/(crossoverCps_ - zeroesToneCps_);
        CLAMP(newFreqHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ);
        freqHz_ = newFreqHz;
        audio_SetFrequency(freqHz_);
        }
      // between zeroes threshold and sink threshold, chillout
      else{
        varioState_ = VARIO_STATE_QUIET;
        tick_ = 0;
        beepPeriodTicks_ = 0;
        beepEndTick_  = 0;
        freqHz_ = 0;
        audio_SetFrequency(freqHz_);
        }
      }
    }
  else { // still processing current beep/tone
    tick_++;
    beepPeriodTicks_--;
    if (tick_ >= beepEndTick_){ // shut off climb beep after 'on' time ends
      newFreqHz = 0;
      }
	  else
	  if (varioCps_ >= VARIO_MAX_CPS) { // offscale climbrate (>= +10m/s) indicated with continuous warbling tone
      newFreqHz = offScaleHiTone_[tick_];
      }
    else
	  if (varioCps_ <= -VARIO_MAX_CPS) {  // offscale sink (<= -10m/s) indicated with continuous descending tone
      newFreqHz = offScaleLoTone_[tick_];
      }
    else
	  if (varioState_ == VARIO_STATE_SINK) {  // sink is indicated with a descending frequency beep
      newFreqHz = freqHz_ - 20;
      }
    else {
      newFreqHz = freqHz_; // no change   
      }
    if (newFreqHz != freqHz_) {
      freqHz_ = newFreqHz;
      audio_SetFrequency(freqHz_);
      }
	  }
  }
