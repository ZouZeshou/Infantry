
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "main.h"


#include "stdio.h"
#include "stdint.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"

#include "can.h"
#include "BSP_can.h"
#include "ChassisControl.h"
#include "delay.h"
#include "BSP_usart.h"
#include "pid.h"
#include "GimbalControl.h"
#include "imu.h"
#include "ShootControl.h"
#include "writtingtask.h"
#include "DBUS.h"
#include "Keyboard.h"
#include "STMGood.h"
int initmark=0 ;

extern uint8_t Remotebuffer[18];


/**
 * @brief TSAK02
 * @param None
 * @return None
 * @attention None
 */
void StartTask02(void const * argument)
{
	//portTickType Current;
	for(;;)
  {
	  if(initmark==0)
	  {
		GimbalCalibration();
		  initmark=1;
	  }
	  taskENTER_CRITICAL();
	  
	  DealKeyMousedata();
		ChooseChassisMode();
	  Chassisfollow();
	  DealRemotedata();
	  MecanumCalculate();
	  ChassisPid();			//chassis
	  
	  GetGimbalTarget();
	  DealGimbalPosition();
	  PitchPID(GimbalData.PitchTarget);
	  YawPID(GimbalData.YawTarget);	
//		Plot_in_UpperMonitor();//Gimbal
	  
		ChooseStirMotorMode();
	  Switchshoot();
	  StirPID (StirMotorData.TargetPosition,StirMotorData.BackSpeed,StirMotorData.BackPositionNew);
		//ShootControl
	  
	  taskEXIT_CRITICAL();
	
    osDelay(5);
  }

}
/**
 * @brief TASK03
 * @param None
 * @return None
 * @attention None
 */
void StartTask03(void const * argument)
{
	//portTickType Current;
	for(;;)
  {
	  
	 //Current = xTaskGetTickCount();  
//		Can1_SendMsg(0x200,Chassisdata.Current[0],Chassisdata.Current[1],Chassisdata.Current[2],Chassisdata.Current[3]);
//		Can1_SendMsg(0x1FF,GimbalData.YawCurrent,GimbalData.PitchCurrent,0,0);
//		Can2_SendMsg(0x1FF,0,0,StirMotorData.Current,0);
     osDelay(5);
  }

}
/**
 * @brief LED Toggle
 * @param None
 * @return None
 * @attention None
 */
void StartTask04(void const * argument)
{
	
	for(;;)
  {
	  static int wait;
	  wait++;
	  if(wait>100)
	  {
		wait=0;
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1);		
		PrintFunction();
	  }
	  imu_cal_update();
		osDelay(2);
  }

}
/**
 * @brief 调试时需用到的打印数据函数
 * @param None
 * @return None
 * @attention None
 */
void PrintFunction(void)
{

/***************************************************** chassisdebug ********************************************/
//	  printf("work\r\n");
//	  printf("targetspeed %d %d %d %d\r\n",Chassisdata.Speed[0],Chassisdata.Speed[1],Chassisdata.Speed[2],Chassisdata.Speed[3]);
//		printf("BackSpeed %d %d %d %d\r\n",Chassisdata.BackSpeed[0],Chassisdata.BackSpeed[1],Chassisdata.BackSpeed[2],Chassisdata.BackSpeed[3]);
////	  printf("current %d %d %d %d\r\n",Chassisdata.Current[0],Chassisdata.Current[1],Chassisdata.Current[2],Chassisdata.Current[3]);
////	  printf("Remotebuffer %d %d %d %d\r\n",Remotebuffer[0],Remotebuffer[1],Remotebuffer[2],Remotebuffer[3]);
//	  printf("channel %d %d\r\n",RC_Ctl.rc.ch0,RC_Ctl.rc.ch1);
//	  printf("Vx %d Vy %d Rotate %d\r\n",Chassisdata.Vx,Chassisdata.Vy,Chassisdata.Rotate);
//	  printf("error %f %f %f %f\r\n",Chassiswheelpid[0].errNow,Chassiswheelpid[1].errNow,Chassiswheelpid[2].errNow,Chassiswheelpid[3].errNow);
//	  printf("ctrout %f %f %f %f\r\n",Chassiswheelpid[0].ctrOut,Chassiswheelpid[1].ctrOut,Chassiswheelpid[2].ctrOut,Chassiswheelpid[3].ctrOut);
//	  printf("currentparam %f\r\n",currentparam);
//	  printf("currentSum %f\r\n",Chassisdata.CurrentSum);
//	  printf("pid %f %f %f %f\r\n",Chassiswheelpid[0].kp,Chassiswheelpid[1].kp,Chassiswheelpid[2].kp,Chassiswheelpid[3].kp);
//			printf("FoOut err%f out%f\r\n ",ChassisfollowOutter.errNow,ChassisfollowOutter.ctrOut);
//			printf("Foinner err%f out%f\r\n",ChassisfollowInner.errNow,ChassisfollowInner.ctrOut);
	
/*************************************************** Gimbaldebug ***********************************************/
//	  printf("backpos Yaw %d Pitch %d\r\n",GimbalData.YawBacknow,GimbalData.PitchBacknow);
//	  printf("backspeed  Yaw %d Pitch %d\r\n",GimbalData.Yawspeed,GimbalData.Pitchspeed);
//	  printf("totalpos Yaw %d Pitch %d\r\n",GimbalData.Yawposition,GimbalData.Pitchposition);
////	  printf("gyro x %d y %d z %d\r\n",imu_data.gx,imu_data.gy,imu_data.gz);
//	  printf("Yawtaget %f Pittaget %f\r\n",GimbalData.YawTarget,GimbalData.PitchTarget);
//			printf("error yaw%f pitch%f\r\n",YawOutter.errNow,PitchOutter.errNow);
//			printf("Yaw outter %f inner %f\r\n",YawOutter.ctrOut,YawInner.ctrOut);
////			printf("Yawcur %d\r\n",GimbalData.YawCurrent);
//			printf("Pitch outter %f inner %f\r\n",PitchOutter.ctrOut,PitchInner.ctrOut);
			printf("Gyro1 gy%d gz%d ang%f\r\n",Gyroscope1.gy,Gyroscope1.gz,Gyroscope1.angle);
			printf("Gyro1State %d\r\n",Gyro1State);
			printf("Gyro2 gy%d gz%d ang%f\r\n",Gyroscope2.gy,Gyroscope2.gz,Gyroscope2.angle);
//		 printf("PitSpd %d YawSpd %d\r\n",GimbalData.Pitchspeed,GimbalData.Yawspeed);
////  printf("FollowctrOut %f\r\n",ChassisfollowOutter.ctrOut);
////	  printf("GimbalImu %f\r\n",GimbalData.ImuData);
/****************************************************** ShootControldebug ***************************************************/
//	  printf("/*******************Stir******************/ \r\n");
//	  printf(" current %d BackSpeed %d Targetpos %lld\r\n",StirMotorData.Current,StirMotorData.BackSpeed,StirMotorData.TargetPosition);
//	  printf("newpos %d oldpos %d totalpos %lld \r\n",StirMotorData.BackPositionNew,StirMotorData.BackPositionOld,StirMotorData.TotalPosition);
//		printf("StirTar %lld\r\n",StirMotorData.TargetPosition);
//			printf(" Stir %d \r\n",StirMotorData.Current);
//	  printf("PIDInner error %f ctrout %f\r\n",StirMotorInnerPID.errNow,StirMotorInnerPID.ctrOut);
/*************************/
//printf("sinf(%f) %f\r\n",P,sinf(P));
//printf("cosf(%f) %f\r\n",I,cosf(I));
//printf("ChassisAngle %f\r\n",ChassisAngle2);
}
