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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MySerial.h"
#include "MPU6050.h"
#include "Anonymity.h"
#include "AttitudeSolver.h"
#include "MadgWick.h"
#include "MyMadgWick.h"
#include "Motor.h"
#include "Receiver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern quaternion q;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	MySerial_Init();
	MPU6050_Init();
//	Motor_Init();
	Receiver_Init();
	// 初始化姿态解算器
//   AttitudeSolver_Init(1/0.08, 0.3f); // 采样频率125Hz，增益为0.3
//	 int count = 0; //用于Yaw值线性回归矫�??
//	Motor_SetPulse(1,0.4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	float ax, ay, az, gx, gy, gz;
//	float i = 0.0f;

//		Motor_SetPulse(1,0.07);
//		HAL_Delay(2000);
//		Motor_SetPulse(1,0.03);
//		HAL_Delay(2000);
  while (1)
  {
//		for(int i = 0;i < 4;i++) {
//			printf("Channel %d,mapVal: %.2f\n", i + 1, pwmMapVal[i]);
////			printf("Channel %d,wideth: %lu\n", i + 1, pwmWidth[i]);
//		}
		
          
		
//				MPU6050_GetAccelData(&ax, &ay, &az);
//        MPU6050_GetGyroData(&gx, &gy, &gz);
//			float ax, ay, az, gx, gy, gz;
//			float Roll, Pitch, Yaw;
		// 获取传感器数
//			MPU6050_GetAccelData(&ax, &ay, &az);
//			MPU6050_GetGyroData(&gx, &gy, &gz);
//			MPU6050_GetGyroAveData(&gx, &gy, &gz);
//			merge(ax, ay, az, gx, gy, gz);
//			quaternion_to_euler(&q,&Roll, &Pitch, &Yaw);
//			printf("q.w = %.3f\tq.x = %.3f\tq.y = %.3f\tq.z = %.3f\t\n",q.w, q.z, q.y, q.z);
//			SendToAno04(q.w, q.z, q.y, q.z);
		  // 更新姿�??
//			AttitudeSolver_UpdateIMU(gx, gy, gz, ax, ay, az);
//			
//			// 获取姿�?�角

//			AttitudeSolver_GetEulerAngles(&Roll, &Pitch, &Yaw,count++);
		 
		  
//        printf("Accel: X=%.2fg Y=%.2fg Z=%.2fg\n", ax, ay, az);
//		    printf("Gyro: X=%.2fPI Y=%.2fPI Z=%.2fPI\n", gx, gy, gz);
//				printf("**********************************************\n");
//				printf("q0: %.2f, q1: %.2f, q2: %.2f, q3: %.2f\n", q0, q1, q2, q3);
//				printf("**********************************************\n");
//		  SendToAno01();
//			printf("Roll = %.2f\tPitch = %.2f\tYaw = %.2f\n",Roll, Pitch, Yaw);
//			SendToAno03(Roll, Pitch, Yaw);

// 发�?�四元数数据到上位机
//        SendToAno04(q0, q1, q2, q3);
//        HAL_Delay(5);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
