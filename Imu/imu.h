#ifndef _IMU_H_
#define _IMU_H_

#include "stm32f4xx_hal.h"
#include "MPU6500.h"
#include <math.h>
#include "ChassisControl.h"
#include "BSP_can.h"
#define IMU_TARGET_TEMP 50
#define GYROOFFLINE 1
#define GYROABNORMAL 2
#define GYRONORMAL 3

void init_quaternion(void);
void imu_AHRS_update(void);
void InfantryYawUpdate(void);
void PID_Temp_Init(void);
void Temp_keep(void);
void imu_cal_update(void);
int JudgeGyro(GyroData * Gyro,int RT_fps);

extern int imu_init_ok;
extern float q0,q1,q2,q3;
extern float imu_pitch,imu_roll;

#endif



