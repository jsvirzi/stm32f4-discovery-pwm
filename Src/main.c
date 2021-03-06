/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "time.h"

#include "serial.h"
#include "uart.h"
#include "gps.h"

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

int verbose = 0;
int frameCountingStarted = 0; /* set when we have started counting frame pulses */
uint32_t frameCounter = 0; /* number of outgoing frame pulses */
uint32_t pulseCounter = 0; /* number of incoming gps pulses */
int frameRate = 20; /* 20 Hz frame rate out */
volatile uint32_t clockTicks = 0;
volatile int globalEnableFrames = 0;
volatile int generateFrames = 0;
uint32_t gpsTime = 0;
uint32_t frameStartTime = 0;
uint32_t frameStopTime = 0;
char gpsDateString[32];
char gpsTimeString[32];
extern int started;

void GpsPower(int flag) {
	GPIOB->BSRR = (1 << (4 + 16 * flag));
}

void LidarPower(int flag) {
	GPIOB->BSRR = (1 << (5 + 16 * flag));
}

void CameraPower(int flag) {
	GPIOB->BSRR = (1 << (6 + 16 * flag));
}

int compareFramesToPulses() {
	uint32_t expectedFrames = pulseCounter * frameRate;
	if(frameCounter > expectedFrames) return 1;
	else if(frameCounter < expectedFrames) return -1;
	else return 0;
}

unsigned long tim2Counter = 0, lastTim2Counter = 0;

/* the next three constants must be consistent. 2^3 = 8. mask = 8-1 */
const int ppsCalibrationTicksSize = 16;
const int ppsCalibrationTicksSizeMask = 0xF;
const int ppsCalibrationTicksSizeLog = 4;

unsigned long int ppsCalibrationTicks[ppsCalibrationTicksSize];
unsigned int ppsCalibrationTicksHead = 0;
unsigned int ppsCalibrationTicksPoints = 0;
unsigned long int tickAcc;

unsigned int totalCalibrationTicks() {
	int i;
	tickAcc = 0;
	for(i=0;i<ppsCalibrationTicksSize;++i) {
		tickAcc += ppsCalibrationTicks[i];
	}
	return tickAcc;
}

const int defaultPwmPeriod = 50000;
const int updateRate = 20;

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	initUarts();

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
	
	TIM2->CR1 |= 0x1;
	TIM2->ARR = 0xFFFFFFFF;
	
//	TIM4->CR1 |= 0x81;
	TIM4->CCER &= ~0xF;
	TIM4->CCER |= 0x1;
	TIM4->DIER |= 0x1;
	TIM4->ARR = defaultPwmPeriod;
	
	unsigned int value = huart1.Instance->CR1;
	value |= uartRxNEIE;
	huart1.Instance->CR1 = value;
	
	value = huart2.Instance->CR1;
	value |= uartRxNEIE;
	huart2.Instance->CR1 = value;
	
	// huart2.Instance->DR = 'A';
	
	const char *header1 = "$GPRMC";
	const char *trailer1 = "*";
	const char *header2 = "$AT";
	const char *trailer2 = "*";
	int start, final;
	int lengthTrailer1 = strlen(trailer1), lengthTrailer2 = strlen(trailer2);

	displayOn(1); /* turn ON display */

	GPIOD->BSRR = 0x80000000;
	GPIOB->BSRR = 0x00030000;
	cat("Nauto(2018) Rig Firmware V1.01\n");
	GPIOD->BSRR = 0x00008000;
	GPIOB->BSRR = 0x00000003;

	// display("\x0c\x80Nauto(2018)\x94Rig FW Rev V1.01");
/* essentially acts as a clear display to send 20 bytes per line */
	display(0, "NAUTO(2018) MAPPING ", 20);
	display(1, "H/W V1 F/W V1.01    ", 20);
	display(2, "i love my elenitas!!", 20);
	// huart1.Instance->DR = 'E';

	uint32_t previousGpsTime = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if (previousGpsTime != gpsTime) {
			previousGpsTime = gpsTime;
			char str[20];
			int n_bytes = snprintf(str, sizeof(str), "T=%8.8u", gpsTime);
			display(3, str, n_bytes);
		}
		processUarts();
		int status = syncSerialStream(&uart1RxQueue, header1, trailer1, &start, &final);
		if(status == 0) {
			char str1[64];
			int j, k = splitString(&uart1RxQueue, start, final);
			if(verbose >= 2) {
				for(j=0;j<k;++j) {
					snprintf(str1, sizeof(str1), "index = %d. field = [%s]\n", j, getField(1, j));
					cat(str1);
				}
			}
			strcpy(gpsTimeString, getField(1, 1));
			strcpy(gpsDateString, getField(1, 9));
			gpsTime = UtcTimeFromGprmcDateTimeStrings(gpsDateString, gpsTimeString);
			if(verbose) {
				snprintf(str1, sizeof(str1), "gps time = %d from [%s:%s]\n", gpsTime, gpsDateString, gpsTimeString);
				cat(str1);
			}
		}
		status = syncSerialStream(&uart2RxQueue, header2, trailer2, &start, &final);
		if(status == 0) {
			char str2[128];
			final = (final - lengthTrailer2) & uart2RxQueue.mask; /* remove trailer from payload */
			int j, k = splitString(&uart2RxQueue, start, final);
			if(verbose) {
				for(j=0;j<k;++j) {
					snprintf(str2, sizeof(str2), "index = %d. field = [%s]\n", j, getField(2, j));
					cat(str2);
				}
			}
			if(strcmp(getField(2, 0), "$ATENAF") == 0) {
				globalEnableFrames = 1;
				snprintf(str2, sizeof(str2), "enabling frames\n");
				started = 1;
				TIM4->CR1 |= 0x81; // TODO move to function
				cat(str2);
			} else if(strcmp(getField(2, 0), "$ATDISF") == 0) {
				globalEnableFrames = 0;
				snprintf(str2, sizeof(str2), "disabling frames\n");
				started = 0;
				TIM4->CR1 &= ~0x81; // TODO move to function
				cat(str2);
			} else if(strcmp(getField(2, 0), "$ATGENF") == 0) {
				generateFrames = atoi(getField(2, 1));
				snprintf(str2, sizeof(str2), "generating %d frames\n", generateFrames);
				cat(str2);
			} else if(strcmp(getField(2, 0), "$ATVERBOSE") == 0) {
				verbose = atoi(getField(2, 1));
				snprintf(str2, sizeof(str2), "verbose = %d\n", verbose);
				cat(str2);
			} else if(strcmp(getField(2, 0), "$ATGPSTIME") == 0) {
				snprintf(str2, sizeof(str2), "$GPSTIME,%u,%s,%s*\n", gpsTime, gpsDateString, gpsTimeString);
				cat(str2);
			} else if(strcmp(getField(2, 0), "$ATSTARTF") == 0) {
				frameStartTime = atoi(getField(2, 1));
				snprintf(str2, sizeof(str2), "frames starting at %u\n", frameStartTime);
				cat(str2);
			} else if(strcmp(getField(2, 0), "$ATSTOPF") == 0) {
				frameStopTime = atoi(getField(2, 1));
				snprintf(str2, sizeof(str2), "frames stopping at %u\n", frameStopTime);
				cat(str2);
			} else if(strcmp(getField(2, 0), "$ATFRAMES") == 0) {
				snprintf(str2, sizeof(str2), "$FRAMES,%u*\n", frameCounter);
				cat(str2);
			} else if(strcmp(getField(2, 0), "$ATGPS") == 0) {
				int flag = atoi(getField(2, 1));
				GpsPower(flag);
				if (flag) { display(1, "GPS ON", 0); } 
				else { display(1, "GPS OFF", 0); }
			} else if(strcmp(getField(2, 0), "$ATLIDAR") == 0) {
				int flag = atoi(getField(2, 1));
				LidarPower(flag);
				if (flag) { display(1, "LIDAR ON", 0); } 
				else { display(1, "LIDAR OFF", 0); }
			} else if(strcmp(getField(2, 0), "$ATCAMERA") == 0) {
				int flag = atoi(getField(2, 1));
				CameraPower(flag);
				if (flag) { display(1, "CAMERA ON", 0); } 
				else { display(1, "CAMERA OFF", 0); }
			}
		}
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
