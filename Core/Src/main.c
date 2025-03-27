/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "motor.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOFA_FLOAT_NUM (6)
#define VOFA_BYTES_NUM (4*(VOFA_FLOAT_NUM+1))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t cnt = 0;
float    vofa_f[6];
uint8_t  vofa_d[28];
int16_t  data[3] = {0,0,0};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  __HAL_RCC_HSI_ENABLE();
  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSE);
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
  MX_CAN1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  float vel_ref = 0.0f;
  float pos_ref = 0.0f;
  float tor_ref = 0.0f;
  float sec     = 0.0f;
  uint32_t timestamp = 0;

  HAL_Delay(2999);

  for(int i=0;i<3000;i++) {
      MITSend(0X101,pos_ref,vel_ref,tor_ref,50.0f,1.0f);
      HAL_Delay(0);
  }

  HAL_Delay(2999);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    timestamp = 0;
    do{
      sec = (float)timestamp/1000.f;
      if(sec<0.5f){
          pos_ref = sec*sec;
          vel_ref = 2.f*sec;
          tor_ref = 0.75f;
      }else if(sec>=0.5f&&sec<1.5f){
          pos_ref = -(sec-1.f)*(sec-1.f)+0.5f;
          vel_ref =  -2.f*(sec-1.0f);
          tor_ref = -0.75f;
      }else{
          pos_ref = (sec-2.0f)*(sec-2.0f);
          vel_ref = 2*(sec-2.0f);
          tor_ref = 0.75f;
      }

      vofa_f[0] = vel_ref;
      vofa_f[1] = pos_ref;
      vofa_f[2] = tor_ref;

      MITSend(0X101,pos_ref,vel_ref,tor_ref,15.f,1.0f);
      HAL_Delay(0);
      timestamp++;
    }while(timestamp<2000);

    for(int i=0;i<10000;i++) {
        MITSend(0X101,pos_ref,vel_ref,tor_ref,15.0f,1.0f);
        HAL_Delay(0);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    if(hcan->Instance ==  CAN1) {
        uint8_t rxdata[8] = {0,0,0,0,0,0,0,0};

        CAN_RxHeaderTypeDef rxheader;
        HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&rxheader,rxdata);

        if(rxheader.StdId == 0X51)
        {
            data[0] = (int16_t)(rxdata[0]<<8|rxdata[1]);
            data[1] = (int16_t)(rxdata[4]<<8|rxdata[5]);
            data[2] = (int16_t)(rxdata[2]<<8|rxdata[3]);

//          vofa_f[3] rad/s
            vofa_f[3] = ((float)(data[0]) / 600.f);
            vofa_f[4] = ((float)(data[1]) / 600.0f);
            vofa_f[5] = ((float)(data[2]) / 100.f);
            memcpy(vofa_d,(uint8_t*)vofa_f,sizeof(vofa_f));
            vofa_d[26] = 0X80;
            vofa_d[27] = 0X7F;
            CDC_Transmit_FS(vofa_d,VOFA_BYTES_NUM);
        }
    }
}
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
