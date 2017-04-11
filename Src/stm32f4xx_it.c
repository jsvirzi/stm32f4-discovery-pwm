/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */

#include "stdlib.h"
#include "serial.h"
#include "uart.h"

extern SimpleCircularQueue uart1Queue;
extern SimpleCircularQueue uart2Queue;

extern int globalEnableFrames;
int localEnableFrames = 0;
extern int generateFrames;
extern int frameCounter;
extern int pulseCounter;
extern unsigned long tim2Counter;
extern unsigned long lastTim2Counter;
extern unsigned long int ppsCalibrationTicks[];
extern unsigned int ppsCalibrationTicksHead;
extern const int ppsCalibrationTicksSize;
extern const int ppsCalibrationTicksSizeMask;
extern const int ppsCalibrationTicksSizeLog;
void pushCalibrationTick(unsigned long int deltaTime);
int ppsOutCounter = 0;
extern int frameRate;
uint32_t expectedFrames;
uint32_t tmpFrameCounter;
uint32_t arr;
int frameDeficit;
int loopPacerCounter = 0;
int loopPacerDivisor = 1;
int started = 0;
extern uint32_t clockTicks;
extern uint32_t frameStartTime;
extern uint32_t frameStopTime;
extern uint32_t gpsTime;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	++clockTicks;
	
	if(ppsOutCounter > 0) {
		--ppsOutCounter;
		if(ppsOutCounter == 0) {
			GPIOD->BSRR = (1 << 29);
		}
	}

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	if(EXTI->PR & (1 << 0)) { /* happens each gps pulse */
		
		TIM4->CNT = 0;

		/* should we be generating frames? */
		uint32_t effectiveGpsTime = gpsTime + 1; /* gps time does not update until later in cycle */
		if(frameStartTime != 0) {
			if(effectiveGpsTime >= frameStartTime) {
				localEnableFrames = 1;
				frameStartTime = 0;
			}
		}
		
		/* should we stop generating frames? */
		if(frameStopTime != 0) {
			if(effectiveGpsTime >= frameStopTime) {
				localEnableFrames = 0;
				frameStopTime = 0;
			}
		}
		
		uint32_t regVal = TIM4->CR1;
		if(globalEnableFrames || localEnableFrames || generateFrames) {
			if(generateFrames > 0) --generateFrames;
			if((regVal & 0x81) != 0x81) {
				TIM4->CR1 |= 0x81;
			}
		} else {
			if((regVal & 0x81) != 0) {
				TIM4->CR1 &= ~0x81;
			}
		}

		GPIOD->BSRR = (1 << 13); /* set PPS high */
		if(ppsOutCounter == 0) ppsOutCounter = 10;
		
		started = 1;
		
		/* for calibrating pps to internal clock */
 		lastTim2Counter = tim2Counter;
 		tim2Counter = htim2.Instance->CNT;
 		long int deltaTime = tim2Counter;
 		deltaTime = deltaTime - lastTim2Counter;
 		ppsCalibrationTicks[ppsCalibrationTicksHead] = deltaTime;
 		ppsCalibrationTicksHead = (ppsCalibrationTicksHead + 1) & ppsCalibrationTicksSizeMask;
		
		if(loopPacerCounter && (ppsCalibrationTicksHead == 0)) {
			uint32_t i, acc = 0;
			for(i=0;i<ppsCalibrationTicksSize;++i) {
				acc += ppsCalibrationTicks[i];
			}
			arr = acc >> ppsCalibrationTicksSizeLog;			
			TIM4->ARR = arr; /* TODO am I reaching here? */
		}

		tmpFrameCounter = frameCounter; /* keep a snapshot because frameCounter changes externally */

		/*
		 * check if current pulse width is producing too few or too many frames
		 */
		++loopPacerCounter;
		if(loopPacerCounter == loopPacerDivisor) {
			expectedFrames = pulseCounter * frameRate;
			frameDeficit = expectedFrames - tmpFrameCounter;
			int frameAdjust = frameDeficit * frameDeficit;
			if((100 < frameDeficit) && (frameDeficit < 100)) frameAdjust = abs(frameDeficit);
			arr = TIM4->ARR;
			if(frameDeficit > 0) {
				arr -= frameAdjust; /* too few */
				// TIM4->ARR = arr;
			} else if(frameDeficit < 0) {
				arr += frameAdjust;
				// TIM4->ARR = arr;
			}
			loopPacerCounter = 0;
		}
		++pulseCounter;		

		EXTI->PR |= (1 << 0); /* clear the interrupt */
		
		/* LED */
		if(pulseCounter & 1) GPIOD->BSRR = 0x80000000;
		else GPIOD->BSRR = 0x8000;
	}

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	if(started) {
		++frameCounter;
	}
	if(frameCounter & 1) GPIOA->BSRR = 0x00000002;
	else GPIOA->BSRR = 0x00020000;

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	
	if(huart1.Instance->SR & uartRxNE) {
		unsigned char ch = huart1.Instance->DR;
		pushSimpleCircularQueue(&uart1RxQueue, &ch, 1);
	}
	
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

	if(huart2.Instance->SR & uartRxNE) {
		unsigned char ch = huart2.Instance->DR;
		pushSimpleCircularQueue(&uart2RxQueue, &ch, 1);
	}
	
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
