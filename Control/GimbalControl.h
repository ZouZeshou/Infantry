#ifndef __GIMBALCONTROL_H
#define __GIMBALCONTROL_H
#include "stdint.h"
#include "pid.h"

typedef struct
{
	int16_t YawBacknow;
	int16_t YawBackold;
	int32_t Yawposition;
	int32_t Yawpositionold;
	int16_t Yawspeed;
	int16_t Yawangle;
	int16_t Yawcirclecounter;
	int16_t Yawinit;
	int16_t YawCurrent;
	float YawTarget;
	int YawMid;
	int YawMax;
	int YawMin;
	
	
	int16_t PitchBacknow;
	int16_t PitchBackold;
	int32_t Pitchposition;
	int32_t Pitchpositionold;
	int16_t Pitchspeed;
	int16_t Pitchangle;
	int32_t Pitchcirclecounter;
	int16_t Pitchinit;
	int16_t PitchCurrent;
	float PitchTarget;
	int PitchMid;
	int PitchMax;
	int PitchMin;
	
	float ImuData;
}GimbalMotor;

extern GimbalMotor GimbalData;
extern PID_AbsoluteType YawInner;
extern PID_AbsoluteType YawOutter;
extern PID_AbsoluteType PitchInner;
extern PID_AbsoluteType PitchOutter;

void GimbalInit (void);
void GimbalCalibration(void);
void GetGimbalTarget(void);
void DealGimbalPosition(void);
void PitchPID(float Target);
void YawPID(float Target);
void GetYawIncrement(void);

#endif
