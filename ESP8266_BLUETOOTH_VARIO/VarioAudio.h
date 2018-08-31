#ifndef VARIO_AUDIO_H_
#define VARIO_AUDIO_H_


typedef struct BEEP_ {
    int periodTicks;  // on-time + off-time
    int endTick; // on-time
} BEEP;

// clamp climbrate/sinkrate for audio feedback to +/- 10 m/s
#define VARIO_MAX_CPS         1000

#define VARIO_STATE_SINK    	11
#define VARIO_STATE_QUIET   	22
#define VARIO_STATE_ZEROES		33
#define VARIO_STATE_CLIMB   	44

class VarioAudio {
	
public :
VarioAudio(){};

void Config();
void Beep(int32_t cps);

private :
int32_t varioCps_; // internal state : current climb/sink rate
int32_t freqHz_; // internal state : current frequency being generated
// sinktone indicates sinking air, this is a warning tone
int32_t sinkToneCps_; // threshold in cm per second
// climbtone indicates lift is strong enough to try turning to stay in it
int32_t climbToneCps_; // threshold in cm per second
// zeroestone indicates weak lift, possibility of stronger lift/thermal nearby
int32_t zeroesToneCps_; // threshold in cm per second
// allocate roughly 1 decade (10:1) of speaker frequency bandwidth to climbrates below
// crossoverCps, and 1 octave (2:1) of frequency bandwidth to climbrates above
// crossoverCps. So if you are flying in strong conditions, increase crossoverCps.
// If you are flying in weak conditions, decrease crossoverCps.
int32_t crossoverCps_;

int beepPeriodTicks_; // internal state : current beep interval in ticks
int beepEndTick_; // internal state : current beep  on-time in ticks
int tick_; // internal state : current tick ( 1 tick ~= 20mS)
int varioState_; // internal state : climb/zeroes/quiet/sink
// for offscale climbrates above +10m/s generate continuous warbling tone
int offScaleHiTone_[8]  = {400,800,1200,1600,2000,1600,1200,800};
// for offscale sinkrates below -10m/s generate continuous descending tone
int offScaleLoTone_[8]  = {4000,3500,3000,2500,2000,1500,1000,500};
// {beep_period, beep_on_time} based on vertical climbrate in 1m/s intervals
BEEP beepTbl_[10] = {
{16,10}, // 0m/s to +1m/s
{14,9},  
{12,8},
{10,7},
{9,6},
{8,5},
{7,4},
{6,3},
{5,2},
{4,2}, // +9m/s to +10m/s
};	
};


#endif
