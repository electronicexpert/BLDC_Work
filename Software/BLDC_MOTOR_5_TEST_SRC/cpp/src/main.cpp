/*
 * main.cpp
 *
 *  Created on: May 29, 2024
 *      Author: joachim
 */


#include "main.h"

extern "C"
{
	extern TIM_HandleTypeDef htim1;
}

uint16_t dmaSignal[16] = {0};
uint16_t dmaSignalNormalized[16] = {0};
uint8_t newDmaSignal = 0;
uint16_t motorSpeed = 0;


uint16_t divClosest(uint16_t a, uint16_t b)
{
	return(a+b/2)/b;
}
void processDmaSignal()
{
	newDmaSignal = 0;
	for(int index = 0; index < 15; index++)
		dmaSignalNormalized[index] = divClosest(dmaSignal[index+1] - dmaSignal[index],8)-8;

	for(int index = 0; index <= 10; index++)
	{
		if(dmaSignalNormalized[index] < 100)
			continue;
		motorSpeed = dmaSignalNormalized[index + 1] << 6 |
				     dmaSignalNormalized[index + 3] << 2 |
					 dmaSignalNormalized[index + 5] >> 2;

		break;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		newDmaSignal = 1;
}

void setup()
{
	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t*)dmaSignal, 16);
}

void maincpp()
{
	setup();

	while(1)
	{
		if(newDmaSignal)
		{
			processDmaSignal();
			HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t*)dmaSignal, 16);

		}

	}
}



