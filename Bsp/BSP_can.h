#ifndef __BSPCAN__H
#define __BSPCAN__H
#include "can.h"
#include "stdint.h"
typedef union{
	uint8_t c[2];
	int16_t d;
}wl2data;

typedef union{
	uint8_t c[4];
	float f;
	uint32_t d;
}wl4data;
typedef struct{
	int16_t gy;
	int16_t gz;
	float angle;
	
}ToeGyro;

extern ToeGyro Gyroscope1;
extern ToeGyro Gyroscope2;

void CAN_Enable(void);
void CANFilterStart(CAN_HandleTypeDef *hcan);
void Can1_SendMsg(uint32_t id,int16_t current0,int16_t current1,int16_t current2,int16_t current3);
void Can2_SendMsg(uint32_t id,int16_t current0,int16_t current1,int16_t current2,int16_t current3);

#endif
