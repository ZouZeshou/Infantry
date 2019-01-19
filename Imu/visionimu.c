#include "visionimu.h"

Visionimu Vimu;// left - right +

void VisionImuCal(void){
	uint8_t sum = 0;
	if(uart2_buff[0] != 0x55 && uart2_buff[1] != 0x52) return;
	for(int i=0;i<10;i++){
		sum+=uart2_buff[i];
	}
	if(sum == uart2_buff[10]){
		int16_t wz = (uart2_buff[6] | (uart2_buff[7] << 8));
		Vimu.gz = (float)wz / 16.384f;// dps / 57.2957795f; // dps->rad/s
	}
	__HAL_UART_CLEAR_PEFLAG(&huart2);
}

void VisionImu_IRQ(void){
	if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET){
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);		
		HAL_UART_DMAStop(&huart2);
		HAL_UART_Receive_DMA(&huart2,uart2_buff,11);
	}
}
