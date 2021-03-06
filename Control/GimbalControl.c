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
	PitchOutter.kp = 20;//30
	PitchOutter.ki = 0;
	PitchOutter.kd = 0;	
	PitchOutter.errILim = 0;
	PitchOutter.OutMAX = 250;
	
	PitchInner.kp = 40;//20
	PitchInner.ki = 0.5;
	PitchInner.kd = 0;
	PitchInner.errILim = 3000;
	PitchInner.OutMAX = 5000;

	YawOutter.kp = 10;//15
	YawOutter.ki = 0;
	YawOutter.kd = 0;	
	YawOutter.errILim = 0;
	YawOutter.OutMAX = 500;
	
	YawInner.kp = 120;//80
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
	if(GimbalData.PitchBacknow>4000)
	{
		GimbalData.PitchMax = 2500 + 8192;
		GimbalData.PitchMid = 2210 + 8192;
		GimbalData.PitchMin = 1800 + 8192;
	}
	else
	{
		GimbalData.PitchMax = 2500;
		GimbalData.PitchMid = 2210;
		GimbalData.PitchMin = 1800;
	}
	GimbalData.PitchTarget = GimbalData.PitchMid;
	GimbalData.YawTarget = YAW_MID_ANGLE;
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
		GimbalData.YawTarget += (float)(((-RC_Ctl.rc.ch2 + 1024)*0.0015f) + RC_Ctl.mouse.x * MOVE_MOUSEROTATE_CONST);
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
	
	GimbalData.Yawposition = GimbalData.YawBacknow + GimbalData.Yawcirclecounter*8192;
	GimbalData.Yawpositionold = GimbalData.Yawposition;
	if(ChassisMode == ROTATE )
	YawMidPosi = 4750 + GimbalData.Yawcirclecounter*8192;
	
	if(GimbalData.Pitchinit)
	{ 
		if(GimbalData.PitchBacknow-GimbalData.PitchBackold>5000)
			GimbalData.Pitchcirclecounter--;
		if(GimbalData.PitchBacknow-GimbalData.PitchBackold<-5000)
			GimbalData.Pitchcirclecounter++;
	}
	GimbalData.PitchBackold=GimbalData.PitchBacknow;
	GimbalData.Pitchinit=1;
	
	GimbalData.Pitchposition=GimbalData.PitchBacknow + GimbalData.Pitchcirclecounter*8192;
	GimbalData.Pitchpositionold=GimbalData.Pitchposition;
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
	
	PitchOutter.errNow = (Target - GimbalData.Pitchposition)*0.0439453125f;//处理成角度值
	PID_AbsoluteMode(&PitchOutter);
	PitchInner.errNow = PitchOutter.ctrOut -  GimbalData.Pitchspeed;//(1/65536*4000)
	PID_AbsoluteMode(&PitchInner);
	GimbalData.PitchCurrent = -PitchInner.ctrOut;
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
	YawOutter.errNow = (Target - Gyroscope1.angle);
	PID_AbsoluteMode(&YawOutter);
	YawInner.errNow = YawOutter.ctrOut - GimbalData.Yawspeed;
	PID_AbsoluteMode(&YawInner);
	GimbalData.YawCurrent = (int16_t)(YawInner.ctrOut + StirMotorData.Current*0.08);
}
