/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint32_t startTicks, currentTicks;
int frameCounter = 0;
int pulseCounter = 0;
unsigned long tim2Counter = 0, lastTim2Counter = 0;
unsigned long int *genericPointer = 0;

/* the next three constants must be consistent. 2^3 = 8. mask = 8-1 */
const int ppsCalibrationTicksSize = 8;
const int ppsCalibrationTicksSizeMask = 0x7;
const int ppsCalibrationTicksSizeLog = 3;

unsigned long int ppsCalibrationTicks[ppsCalibrationTicksSize];
unsigned int ppsCalibrationTicksHead = 0;
unsigned int ppsCalibrationTicksPoints = 0;
unsigned long int tickAcc;
unsigned long tmpPeriod;

unsigned int averageCalibrationTicks() {
//	return ppsCalibrationTicks[ppsCalibrationTicksHead];
	int i;
	tickAcc = 0;
	for(i=0;i<ppsCalibrationTicksSize;++i) {
		tickAcc += ppsCalibrationTicks[i];
	}
	tickAcc = tickAcc >> ppsCalibrationTicksSizeLog;
	return tickAcc;
}

// void pushCalibrationTick(unsigned long int deltaTime) {
// 	ppsCalibrationTicks[ppsCalibrationTicksHead] = deltaTime;
// 	ppsCalibrationTicksHead = (ppsCalibrationTicksHead + 1) & ppsCalibrationTicksSizeMask;
// }

// const int defaultPwmPeriod = 20000;
// const int updateRate = 50;
const int defaultPwmPeriod = 50000;
const int updateRate = 20;
const int pwmArrSize = 2 * updateRate;
unsigned int pwmArr[pwmArrSize];
unsigned int *pwmArrPing = &pwmArr[0], *pwmArrPong = &pwmArr[updateRate];
int pwmArrPingPongPhase = 0, pwmArrPhase = 0;

void updatePwmArrPeriods() {
	unsigned int *pPwmArr = (pwmArrPingPongPhase == 0) ? pwmArrPong : pwmArrPing;
	pwmArrPingPongPhase = (pwmArrPingPongPhase == 0) ? 1 : 0;
	unsigned int ppsCalibrationTicks = averageCalibrationTicks();
	unsigned int pwmPeriod = ppsCalibrationTicks / updateRate;
	unsigned int pwmRemainder = ppsCalibrationTicks - pwmPeriod * updateRate;
	tickAcc = 0;
	int i;
	for(i=0;i<updateRate;++i) {
		tmpPeriod = pwmPeriod;
		if(tickAcc < pwmRemainder) {
			++tmpPeriod;
		}
		pPwmArr[i] = tmpPeriod;
		++tickAcc;
	}
}

void updatePwmArrPeriod() {
	unsigned long int arr = pwmArr[pwmArrPhase];
	TIM4->ARR = arr;
	pwmArr[pwmArrPhase] = defaultPwmPeriod; /* will be overwritten with good data later */
	++pwmArrPhase;
	if(pwmArrPhase >= pwmArrSize) pwmArrPhase = 0;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
	
	int i;
	for(i=0;i<pwmArrSize;++i) {
		pwmArr[i] = defaultPwmPeriod;
	}
	
	TIM2->CR1 |= 0x1;
	TIM2->ARR = 0xFFFFFFFF;
	
	TIM4->CR1 |= 0x81;
	TIM4->CCER &= ~0xF;
	TIM4->CCER |= 0x1;
	TIM4->DIER |= 0x1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	startTicks = HAL_GetTick();
  while (1)
  {
		if(frameCounter == 500) {
			currentTicks = HAL_GetTick() - startTicks;
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
