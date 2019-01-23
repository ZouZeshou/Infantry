#include "GimbalControl.h"
#include "stdint.h"
#include "pid.h"
#include "STMGood.h"
#include "BSP_usart.h"
#include "ChassisControl.h"
#include "ShootControl.h"
#include "DBUS.h"
#include "Keyboard.h"
#include "BSP_can.h"
GimbalMotor GimbalData;
PID_AbsoluteType YawInner;
PID_AbsoluteType YawOutter;
PID_AbsoluteType PitchInner;
PID_AbsoluteType PitchOutter;


int PitchDebug = 0;
int YawDebug = 0;
extern int DBUScounter;

/**
 * @brief initialize the parameter of Gimbal
 * @param None
 * @return None
 * @attention  None
 */
void GimbalInit (void)
{
	PitchOutter.kp = 30;//30
	PitchOutter.ki = 0;
	PitchOutter.kd = 0;	
	PitchOutter.errILim = 0;
	PitchOutter.OutMAX = 400;
	
	PitchInner.kp = 70;//20
	PitchInner.ki = 0;
	PitchInner.kd = 0;
	PitchInner.errILim = 3000;
	PitchInner.OutMAX = 25000;

	YawOutter.kp = 12;//15
	YawOutter.ki = 0;
	YawOutter.kd = 0;	
	YawOutter.errILim = 0;
	YawOutter.OutMAX = 500;
	
	YawInner.kp = 60;//80
	YawInner.ki = 0;
	YawInner.kd = 0;
	YawInner.errILim = 1000;
	YawInner.OutMAX = 15000;
}

/**
 * @brief Calibrate the position of Gimbal Motor
 * @param None
 * @return None
 * @attention  None
 *///1800 2500 2210 
void GimbalCalibration(void)
{

		GimbalData.PitchMax = 7300;
		GimbalData.PitchMid = 7045;
		GimbalData.PitchMin = 5800;

		GimbalData.YawMax = 6750;
		GimbalData.YawMid = 5000;
		GimbalData.YawMin = 4550;

	GimbalData.PitchTarget = GimbalData.PitchMid;
	GimbalData.YawTarget = GimbalData.YawMid;
}
/**
 * @brief get the tagetposition from remote and  keyboard,mouse
 * @param None
 * @return None
 * @attention  None
 */
void GetGimbalTarget(void)
{
	if(DBUScounter>0)
	{
		GimbalData.PitchTarget += (float)(((RC_Ctl.rc.ch3 - 1024)*0.0030f) + RC_Ctl.mouse.y * CLOUD_MOUSEPITCH_CONST);
		if(GimbalData.PitchTarget>GimbalData.PitchMax)
			GimbalData.PitchTarget=GimbalData.PitchMax;
		if(GimbalData.PitchTarget<GimbalData.PitchMin)
			GimbalData.PitchTarget=GimbalData.PitchMin;	
		GimbalData.YawTarget -= (float)(((-RC_Ctl.rc.ch2 + 1024)*0.0045f) + RC_Ctl.mouse.x * MOVE_MOUSEROTATE_CONST);
		if(GimbalData.YawTarget>GimbalData.YawMax)
			GimbalData.YawTarget=GimbalData.YawMax;
		if(GimbalData.YawTarget<GimbalData.YawMin)
			GimbalData.YawTarget=GimbalData.YawMin;	
	}
}

/**
 * @brief transform the single circular position to a continuous data
 * @param backposition from encoder
 * @return A continuous position data(totalposition)
 * @attention  None
 */
void DealGimbalPosition (void)
{
	if(GimbalData.Yawinit)
	{
		if(GimbalData.YawBacknow-GimbalData.YawBackold>5000)
			GimbalData.Yawcirclecounter--;
		if(GimbalData.YawBacknow-GimbalData.YawBackold<-5000)
			GimbalData.Yawcirclecounter++;
	}
	GimbalData.YawBackold = GimbalData.YawBacknow;
	GimbalData.Yawinit = 1;
	
	GimbalData.Yawposition = GimbalData.YawBacknow + GimbalData.Yawcirclecounter*8191;
//	GimbalData.Yawspeed = (int16_t)((GimbalData.Yawposition - GimbalData.Yawpositionold) / 8191.0f * 2 * 3.1415926f / 0.005f);
	GimbalData.Yawpositionold = GimbalData.Yawposition;
	if(ChassisMode == ROTATE )
	YawMidPosi = 4750 + GimbalData.Yawcirclecounter*8191;
	
	if(GimbalData.Pitchinit)
	{ 
		if(GimbalData.PitchBacknow-GimbalData.PitchBackold>5000)
			GimbalData.Pitchcirclecounter--;
		if(GimbalData.PitchBacknow-GimbalData.PitchBackold<-5000)
			GimbalData.Pitchcirclecounter++;
	}
	GimbalData.PitchBackold=GimbalData.PitchBacknow;
	GimbalData.Pitchinit=1;
	
//	GimbalData.Pitchposition=GimbalData.PitchBacknow + GimbalData.Pitchcirclecounter*8192;
//	GimbalData.Pitchspeed = (int16_t)((GimbalData.Pitchposition - GimbalData.Pitchpositionold) / 8191.0f * 2 * 3.1415926f / 0.005f);
//	GimbalData.Pitchpositionold=GimbalData.Pitchposition;
}
/**
 * @brief PID control of pitchmotor
 * @param None
 * @return None
 * @attention  None
 */
void PitchPID (float Target)
{
	if(PitchDebug == 1)
	{
		PitchOutter.kp=P;//25
		PitchOutter.ki=I;
		PitchOutter.kd=D;	
		PitchOutter.errILim=0;
		PitchOutter.OutMAX=V1;
		
		PitchInner.kp=p;//20
		PitchInner.ki=i;
		PitchInner.kd=d;
		PitchInner.errILim=3000;
		PitchInner.OutMAX=V2;
	}
	
	PitchOutter.errNow = (Target - GimbalData.PitchBacknow)*0.0439453125f;//处理成角度值
	PID_AbsoluteMode(&PitchOutter);
	PitchInner.errNow = PitchOutter.ctrOut -  GimbalData.Pitchspeed;//(1/65536*4000)
	PID_AbsoluteMode(&PitchInner);
	GimbalData.PitchCurrent = PitchInner.ctrOut;
}

/**
 * @brief PID control of Yaw motor
 * @param None
 * @return None
 * @attention  None
 */
void YawPID (float Target)
{
	if(YawDebug == 1)
	{
		YawOutter.kp=P;//15
		YawOutter.ki=I;
		YawOutter.kd=D;	
		YawOutter.errILim=0;
		YawOutter.OutMAX=V1;
		
		YawInner.kp=p;//50
		YawInner.ki=i;
		YawInner.kd=d;
		YawInner.errILim=1000;
		YawInner.OutMAX=V2;
	}
	YawOutter.errNow = -(Target - GimbalData.YawBacknow)*0.0439453125f;
	PID_AbsoluteMode(&YawOutter);
	YawInner.errNow = YawOutter.ctrOut - GimbalData.Yawspeed;
	PID_AbsoluteMode(&YawInner);
	GimbalData.YawCurrent = (int16_t)(-YawInner.ctrOut);
}
