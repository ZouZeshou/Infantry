#ifndef __SHOOTCONTROL_H
#define __SHOOTCONTROL_H
#include "tim.h"
#include "stdint.h"


#define LEFT_FRICTION       TIM1->CCR1
#define RIGHT_FRICTION      TIM4->CCR1
typedef struct
{
	int PositionInit;
	
	int64_t TargetPosition;
	int64_t CircleCounter;
	int64_t TotalPosition;
	int16_t BackPositionOld;
	int16_t BackPositionNew;
	int16_t BackSpeed;
	int16_t Speed;
	int16_t Current;

}StirMotor;

typedef struct
{
	int16_t BackSpeed;
	int16_t TargetSpeed;
	int16_t Current;


}ShootMotor;

extern StirMotor StirMotorData;
extern ShootMotor ShootMotorLeft,ShootMotorRight;

void ChooseStirMotorMode(void);
void ShootInit (void);
void PWMInit(void);
void StirPID (int64_t TargetPosition,int16_t BackSpeed,int16_t BackPosition);
void DealStirMotorPosition (void);
void StirMotorStart (void);
void Switchshoot (void);
void Turn_on_Fric(int16_t spd);
void Turn_off_Fric(void);

#endif
