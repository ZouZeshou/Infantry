#include "ShootControl.h"
#include "pid.h"
#include "BSP_usart.h"
#include "DBUS.h"
#include "STMGood.h"

#define STIRADDITION 32764 //8191*36/9
int FrictionSpd = 1250;
/***********StirMotorMode********************/
#define DISCRETE 0// shot  bullet one by one
#define CONTINUE 1//continuous shot
int StirMotorMode;
/********************************************/
int16_t ShootFrequncy = 13;//1000/5/15

PID_AbsoluteType StirMotorOutterPID,StirMotorInnerPID;
StirMotor StirMotorData;
int StirMotorMode;
int StirUpdateCounter;
int StirMotorDebug = 0;
/**
 * @brief initialise the data will be used in shoot motor
 * @param None
 * @return None
 * @attention None
 */
void ShootInit (void)
{
	StirMotorOutterPID.kp = 20;
	StirMotorOutterPID.ki = 0;
	StirMotorOutterPID.kd = 0;
	StirMotorOutterPID.errILim = 1000;
	StirMotorOutterPID.OutMAX = 720;
	
	StirMotorInnerPID.kp = 30;
	StirMotorInnerPID.ki = 0;
	StirMotorInnerPID.kd = 0;
	StirMotorInnerPID.errILim = 3000 ;
	StirMotorInnerPID.OutMAX = 5000 ;
	
}
/**
 * @brief initialise the data will be used in shoot motor
 * @param None
 * @return None
 * @attention None
 */
void PWMInit(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // friction wheel
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}
/**
 * @brief choose the mode of shoot bullets 
 * @param None
 * @return None
 * @attention None
 */
void ChooseStirMotorMode(void)
{
	if(RC_Ctl.rc.s2 == 1)
	{
		StirMotorMode = CONTINUE;
	}
	else
	{
		StirMotorMode = DISCRETE;
	}
}
/**
 * @brief calculate the final current of StirMotor by PID controller
 * @param TargetPosition
 * @return None
 * @attention None
 */
void StirPID (int64_t TargetPosition,int16_t BackSpeed,int16_t BackPosition)
{
	if(StirMotorDebug == 1)
	{
		StirMotorOutterPID.kp = P;//0.3(*36)
		StirMotorOutterPID.ki = I;
		StirMotorOutterPID.kd = D;
		StirMotorOutterPID.errILim = 1000;
		StirMotorOutterPID.OutMAX = V1;//160
		
		StirMotorInnerPID.kp = p;//100
		StirMotorInnerPID.ki = i;
		StirMotorInnerPID.kd = d;
		StirMotorInnerPID.errILim = 3000 ;
		StirMotorInnerPID.OutMAX = V2 ;//8000
	}
	if(StirMotorMode == DISCRETE)
	{
		StirMotorOutterPID.kp = 20;
		StirMotorOutterPID.ki = 0;
		StirMotorOutterPID.kd = 0;
		StirMotorOutterPID.errILim = 1000;
		StirMotorOutterPID.OutMAX = 800;
		
		StirMotorInnerPID.kp = 30;
		StirMotorInnerPID.ki = 0;
		StirMotorInnerPID.kd = 0;
		StirMotorInnerPID.errILim = 3000 ;
		StirMotorInnerPID.OutMAX = 5000 ;
	}
	else if(StirMotorMode == CONTINUE)
	{
		StirMotorOutterPID.kp = 20;
		StirMotorOutterPID.ki = 0;
		StirMotorOutterPID.kd = 0;
		StirMotorOutterPID.errILim = 1000;
		StirMotorOutterPID.OutMAX = 800;
	
		StirMotorInnerPID.kp = 30;
		StirMotorInnerPID.ki = 0;
		StirMotorInnerPID.kd = 0;
		StirMotorInnerPID.errILim = 3000 ;
		StirMotorInnerPID.OutMAX = 5000 ;
	}
		
	StirMotorOutterPID.errNow = (TargetPosition - StirMotorData.TotalPosition)*0.001220852f;//angle(¶È)
	PID_AbsoluteMode(&StirMotorOutterPID);
	StirMotorInnerPID.errNow = (StirMotorOutterPID.ctrOut - BackSpeed*0.166666667f);//angle velocity£¨¶È/Ã¿Ãë£©
	PID_AbsoluteMode(&StirMotorInnerPID);
	StirMotorData.Current = (int16_t)(StirMotorInnerPID.ctrOut);
}
/**
 * @brief Transform the discrete position to a continuous position 
 * @param None
 * @return None
 * @attention this function should be operation after the motor had electricity;
 */
void DealStirMotorPosition ()
{
	if(StirMotorData.PositionInit)
	{
		if(StirMotorData.BackPositionNew-StirMotorData.BackPositionOld>5000)
			StirMotorData.CircleCounter--;
		if(StirMotorData.BackPositionNew-StirMotorData.BackPositionOld<-5000)
			StirMotorData.CircleCounter++;
	}
	StirMotorData.BackPositionOld = StirMotorData.BackPositionNew;	
	StirMotorData.TotalPosition = StirMotorData.BackPositionNew + StirMotorData.CircleCounter*8192;
	if(StirMotorData.PositionInit == 0)
		StirMotorData.TargetPosition = StirMotorData.TotalPosition;
	StirMotorData.PositionInit=1;
}
/**
 * @brief update the target position of stirmotor 
 * @param None
 * @return None
 * @attention None
 */
void StirMotorStart (void)
{	
		StirUpdateCounter++;
		if(StirUpdateCounter>=ShootFrequncy)
		{
			StirUpdateCounter=0;
			StirMotorData.TargetPosition += STIRADDITION ;
		}	
}
/**
 * @brief switch for stirmotor and ShootControl
 * @param None
 * @return None
 * @attention for debug
 */
void Switchshoot (void)
{
	static int32_t Friction_ok = 0;
	switch(RC_Ctl.rc.s2)
	{
		case 1:
			FrictionSpd = 1350;
			break;
		case 2:
			FrictionSpd = 1250;
			break;
		case 3:
			FrictionSpd = 1300;
			break;
	}
	if(RC_Ctl.rc.s1 == 2)
	{
		ShootFrequncy = 40;
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
		Turn_on_Fric(FrictionSpd);
		Friction_ok++;
		if(Friction_ok>=50)
		{
			StirMotorStart();
		}
	}
	else
	{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
		Turn_off_Fric();
		Friction_ok=0;
	}
}
/**
 * @brief  turn on the friction 
 * @param None
 * @return None
 * @attention for debug
 */
void Turn_on_Fric(int16_t spd)
{
	LEFT_FRICTION = spd;
	RIGHT_FRICTION = spd;
}	
/**
 * @brief  turn on the friction 
 * @param None
 * @return None
 * @attention for debug
 */
void Turn_off_Fric(void)
{
	LEFT_FRICTION = 1000;
	RIGHT_FRICTION = 1000;
}
