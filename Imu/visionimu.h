#ifndef _VISIONIMU_H_
#define _VISIONIMU_H_

#include "stm32f4xx_hal.h"
#include "BSP_UART.h"

typedef struct{
	float gz;
	float angle;
}Visionimu;

void VisionImuCal(void);
void VisionImu_IRQ(void);

extern Visionimu Vimu;
#endif
