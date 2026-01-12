/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    SYS_WAIT_PASSWORD,
    SYS_NORMAL_PLAY,
	SYS_NORMAL_PAUSE
} SystemState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
SemaphoreHandle_t passwordOkSem = NULL;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile SystemState_t systemState = SYS_WAIT_PASSWORD;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
QueueHandle_t freqQueue;
QueueHandle_t ampQueue;

void vTaskLed(void *pvParameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));

        HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));

		HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));

        HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(600));
	}
}

void vTaskPassword(void *pvParameters)
{
    uint8_t buf[sizeof(PASSWORD)];
    uint8_t tmp;
    uint8_t index = 0;

    printf("Lab 4: Programming industrial embedded systems. Please enter password:\r\n");

    while(1)
    {
        if (xQueueReceive(uartRxQueue, &tmp, portMAX_DELAY) == pdTRUE)
        {
            if (tmp == '\r' || tmp == '\n')
                continue;

            if (index < sizeof(buf) - 1)
            {
                buf[index++] = tmp;
            }

            if (index == (sizeof(buf) - 1))
            {
                buf[index] = '\0';

                if (strcmp((char *)buf, PASSWORD) == 0)
                {
                    printf("OK\r\n");
                    systemState = SYS_NORMAL_PLAY;
                    uint8_t delete;
					while (xQueueReceive(uartRxQueue, &delete, 0) == pdTRUE)
					{
						// make queue empty
					}
					xSemaphoreGive(passwordOkSem);
                    vTaskDelete(NULL);
                }
                else
                {
                    index = 0;
                    printf("Wrong password. Try again:\r\n");
                }
            }
        }
    }
}

void vTaskAudio(void *pvParameters)
{
	configAudio();
    uint32_t newFreq;
    float newAmp;

    while (1)
    {
        if (systemState == SYS_NORMAL_PLAY)
        {
        	HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)dmaBuffer, SINE_BUFFER_SIZE);

            while (systemState == SYS_NORMAL_PLAY)
            {
            	if (xQueueReceive(freqQueue, &newFreq, pdMS_TO_TICKS(10)) == pdTRUE)
				{
					changeAudioFrequency(newFreq);
				}
                if (xQueueReceive(ampQueue, &newAmp, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    changeAudioAmplitude(newAmp);
                }
            }

            HAL_I2S_DMAStop(&hi2s3);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void vTaskUartCmd(void *pvParameters)
{
    char cmdBuf[8];
    uint8_t tmp;
    uint8_t idx = 0;

    xSemaphoreTake(passwordOkSem, portMAX_DELAY);

    while(1)
    {
        xQueueReceive(uartRxQueue, &tmp, portMAX_DELAY);
        if (tmp == '\r' || tmp == '\n')
        {
            cmdBuf[idx] = '\0';
            idx = 0;

            if (cmdBuf[0] == 'f')
            {
                int freq = atoi(&cmdBuf[1]);

                if (freq > 0)
                {
                    //changeAudioFrequency((uint32_t)freq);
                	xQueueSend(freqQueue, &freq, 0);
                    printf("OK\r\n");
                }
                else
                {
                    printf("ERR\r\n");
                }
            }
            else if (cmdBuf[0] == 'a')
            {
                int ampPercent = atoi(&cmdBuf[1]);

                if (ampPercent >= 0 && ampPercent <= 100)
                {
                    float amp = ampPercent / 100.0f;
                    //changeAudioAmplitude(amp);
                    xQueueSend(ampQueue, &amp, 0);
                    printf("OK\r\n");
                }
                else
                {
                    printf("ERR\r\n");
                }
            }
            else
            {
                printf("ERR\r\n");
            }
        }
        else
        {
            if (idx < (sizeof(cmdBuf) - 1))
            {
                cmdBuf[idx++] = tmp;
            }
            else
            {
                idx = 0;
                printf("ERR\r\n");
            }
        }
    }
}

void toggleAudioPlayingState(){
	if(systemState == SYS_NORMAL_PLAY){
		systemState = SYS_NORMAL_PAUSE;
	} else if (systemState == SYS_NORMAL_PAUSE){
		systemState = SYS_NORMAL_PLAY;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2, uartRxDmaBuf, RX_DMA_BUF_SIZE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //signal = rand();
	  //HAL_I2S_Transmit(&hi2s3, &signal, 1, 10);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
