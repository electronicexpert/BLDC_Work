/*
 * main.cpp
 *
 *  Created on: May 29, 2024
 *      Author: joachim
 */



// #define LED_PORT GPIOB
// #define LED_PIN  GPIO_PIN_5

#include "main.h"

extern "C"
{
	extern TIM_HandleTypeDef htim1;
	extern TIM_HandleTypeDef htim3;
	extern TIM_HandleTypeDef htim7;
	extern TIM_HandleTypeDef htim17;

}

uint16_t dmaSignal[16] = {0};
uint16_t dmaSignalNormalized[16] = {0};
uint8_t newDmaSignal = 0;
uint16_t motorSpeed = 0;
uint16_t motorSpeedCurrent = 0;


/////////////////////////////////////////////////////////////

const uint16_t dmaPulse = 1;
const uint16_t dmaPulseReload = 1024;
volatile uint16_t dmaBuffer[dmaPulse] = {0};
volatile uint16_t resolution = dmaPulse * dmaPulseReload;

void setDutyCycle(uint16_t dc)
{
	if( dc > resolution ) dc = resolution;

	dmaBuffer[0] = dc;

}


const uint32_t zcOff[6] = { //
		ZC_B_Pin,
		ZC_A_Pin,
		ZC_C_Pin,
		ZC_B_Pin,
		ZC_A_Pin,
		ZC_C_Pin, };

const uint32_t odLow[6] = { //
		OD_C_Pin,
		OD_C_Pin,
		OD_A_Pin,
		OD_A_Pin,
		OD_B_Pin,
		OD_B_Pin, };

const uint32_t odHigh[6] = { //
		OD_A_Pin,
		OD_B_Pin,
		OD_B_Pin,
		OD_C_Pin,
		OD_C_Pin,
		OD_A_Pin, };

const uint32_t odOff[6] = { //
		OD_B_Pin,
		OD_A_Pin,
		OD_C_Pin,
		OD_B_Pin,
		OD_A_Pin,
		OD_C_Pin, };

const uint32_t ccOff[6] = { //
		1<<8,           //  B
		1<<4,           //  A
		1<<12,          //  C
		1<<8,           //  B
		1<<4,           //  A
		1<<12, };        //  C

const uint32_t diOff[6] = { //
		1<<11,           //  B
		1<<10,           //  A
		1<<12,          //  C
		1<<11,           //  B
		1<<10,           //  A
		1<<12, };        //  C


const GPIO_TypeDef* zcPortOff[6] = { //
		(GPIO_TypeDef*) ZC_B_GPIO_Port,
		(GPIO_TypeDef*) ZC_A_GPIO_Port,
		(GPIO_TypeDef*) ZC_C_GPIO_Port,
		(GPIO_TypeDef*) ZC_B_GPIO_Port,
		(GPIO_TypeDef*) ZC_A_GPIO_Port,
		(GPIO_TypeDef*) ZC_C_GPIO_Port,
};

volatile uint32_t resetImrFlags = 0;
volatile uint8_t powerStep = 0;
volatile uint8_t powerStepCurrent = 0;
const uint8_t rising[2][6] = {{1,0,1,0,1,0},{0,1,0,1,0,1}};

volatile uint8_t reverse = 0;
#define zcPinOn ZC_A_Pin | ZC_B_Pin | ZC_C_Pin


void commutate()
{
	// clear wakeup with interrupt reg.

	EXTI->IMR1 = resetImrFlags;

	// disable zc interrupts

	EXTI->FTSR1 &= ~(ZC_A_Pin | ZC_B_Pin | ZC_C_Pin);
	EXTI->RTSR1 &= ~(ZC_A_Pin | ZC_B_Pin | ZC_C_Pin);

	// clear any zc pending interrupts

	EXTI->FPR1 &= ~(ZC_A_Pin | ZC_B_Pin | ZC_C_Pin);
	EXTI->RPR1 &= ~(ZC_A_Pin | ZC_B_Pin | ZC_C_Pin);

	// go to next power step
	powerStepCurrent ++;
	powerStepCurrent %= 6;

	if(reverse)
		powerStep = 5 - powerStepCurrent;
	else
		powerStep = powerStepCurrent;    // forward




	if(rising[reverse][powerStep])
	{
		// if zc rising
		// enable zc rising interrupt

		EXTI->FTSR1 = 0;
		EXTI->RTSR1 |= zcOff[powerStep];

		// enable odLow

		GPIOA->BSRR = odLow[powerStep];
		GPIOA->BRR = odOff[powerStep]; // disable odOff

		TIM3->CCER |= (ccOff[powerStep]);
		//		enable dma on timer 3.
		TIM3->DIER |= (diOff[powerStep]);

	}
	else
		// else
	{
		//enable zc falling interrupt

		EXTI->RTSR1 = 0;
		EXTI->FTSR1 |= zcOff[powerStep];

		// enable odHigh
		GPIOA->BRR = odOff[powerStep]; // disable odOff
		GPIOA->BSRR = odHigh[powerStep];

		TIM3->CCER &= ~(ccOff[powerStep]);
		//		disable dma on timer 3
		TIM3->DIER &= ~(diOff[powerStep]);

	}


	// enable wakeup with interrupt.
	EXTI->IMR1 = resetImrFlags | zcOff[powerStep];
	// clear any zc pending interrupts.
	EXTI->FPR1 = zcPinOn;
	EXTI->RPR1 = zcPinOn;

	// reset timer17 for bump motor.
	TIM17->CNT = 0;


}

volatile uint16_t checkRising = 0;
volatile uint16_t checkFalling = 0;



void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	TIM7->CNT = 0;
	while ( TIM7->CNT < checkRising)
		if((zcPortOff[powerStep]->IDR & zcOff[powerStep] ) == 0)
		   return;
	commutate();
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	TIM7->CNT = 0;
	while ( TIM7->CNT < checkFalling)
		if((zcPortOff[powerStep]->IDR & zcOff[powerStep] ))
			return;
	commutate();
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim17 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(motorSpeedCurrent > 0)
			commutate();
	}
}
/////////////////////////////////////////////////////////////


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
  //      motorSpeed = 100;

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
	EXTI->FTSR1 = 0;
	EXTI->RTSR1 = 0;
	HAL_Delay(1000);

	resetImrFlags = EXTI->IMR1;
	resetImrFlags &= ~(ZC_A_Pin | ZC_B_Pin | ZC_C_Pin);
			EXTI->IMR1 = resetImrFlags;

	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t*)dmaSignal, 16);

	if(HAL_TIM_Base_Start(&htim7) != HAL_OK)
		Error_Handler();
	if(HAL_TIM_Base_Start(&htim3) != HAL_OK)
			Error_Handler();

	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;

	HAL_TIM_PWM_Start (&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start (&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start (&htim3,TIM_CHANNEL_4);

	HAL_TIM_PWM_Stop_DMA(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop_DMA(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop_DMA(&htim3,TIM_CHANNEL_4);


	HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_2,(uint32_t *) dmaBuffer,dmaPulse);
	HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_3,(uint32_t *) dmaBuffer,dmaPulse);
	HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_4,(uint32_t *) dmaBuffer,dmaPulse);

	__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,250);
	HAL_TIM_OC_Start_IT(&htim17,TIM_CHANNEL_1);
	HAL_Delay(100);
}

void maincpp()
{
	setup();

//	LED_PORT->BSRR  |= LED_PIN;
//	LED_GPIO_Port->BSRR = LED_Pin;

	uint8_t direction = 0;

	while(1)
	{
		if(newDmaSignal)
		{
			processDmaSignal();

            if(GPIOA->IDR & REV_Pin)
            {
           	    LED_GPIO_Port->BSRR = LED_Pin;
               	direction = 1;
            }
            else
            {
            	LED_GPIO_Port->BRR = LED_Pin;
            	direction = 0;
            }
           	if(direction != reverse)
            	{
            		motorSpeedCurrent = 0;
            		setDutyCycle(motorSpeedCurrent);
            		reverse = direction;
            		HAL_Delay(100);
            	}
			if(motorSpeed != motorSpeedCurrent)
			{
				if((motorSpeed - motorSpeedCurrent) > 10)
					motorSpeedCurrent += 10;
				else
					motorSpeedCurrent = motorSpeed;

				setDutyCycle(motorSpeedCurrent);
			}

			HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t*)dmaSignal, 16);

		}

	}
}



