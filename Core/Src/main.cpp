/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
#include "Kinematics.hpp"
#include "PidController.hpp"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
PidController pid_controller[2];
Kinematics kinematics;
uint8_t twist[6];
uint8_t odom[18];
float out_motor_speed[2];
float speed[2];
bool uart_flag=true;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
extern "C"
{
	void MX_FREERTOS_Init(void);
	void Handle_Twist(void const * argument);
	void Handle_Odom(void const * argument);
}

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim6)
	{
		kinematics.update_motor_ticks(__HAL_TIM_GetCounter(&htim2),__HAL_TIM_GetCounter(&htim3));
		out_motor_speed[0] = pid_controller[0].update(kinematics.motor_speed(0));
		out_motor_speed[1] = pid_controller[1].update(kinematics.motor_speed(1));
		speed[0]+=out_motor_speed[0];
		speed[1]+=out_motor_speed[1];
		if(speed[0]>=0)
		{
			__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,0);
			__HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,speed[0]);
		}
		else if(speed[0]<0)
		{
			__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,-speed[0]);
			__HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,0);
		}
		if(speed[1]>=0)
		{
			__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,0);
			__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,speed[1]);
		}
		else if(speed[1]<0)
		{
			__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,-speed[1]);
			__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,0);
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		uart_flag=true;
		HAL_UART_Receive_DMA(&huart1, twist, sizeof(twist));
	}
}
void Handle_Twist(void const * argument)
{
  /* USER CODE BEGIN Handle_Twist */
  /* Infinite loop */
  for(;;)
  {
	if(uart_flag==true && twist[0]==0x55 && twist[5]==0x1A)
	{
		int16_t datarx_lin=(twist[1]<<8)|twist[2];
		int16_t datarx_ang=(twist[3]<<8)|twist[4];
		float linear_x=datarx_lin/100.0f;
		float angular_z=datarx_ang/100.0f;
		static float target_motor_speed1,target_motor_speed2,target_alpha;
		kinematics.kinematic_inverse(linear_x*1000, angular_z, target_motor_speed1, target_motor_speed2,target_alpha);
		pid_controller[0].update_target(target_motor_speed1);
		pid_controller[1].update_target(target_motor_speed2);
		float steer_pulse=0.05824*target_alpha*target_alpha+7.47*target_alpha+355.8;
		__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_2,steer_pulse);
		uart_flag=false;
	}
	osDelay(1000);
  }
  /* USER CODE END Handle_Twist */
}

void Float_As_Complement(float data, int i)
{
	int32_t integer_data=(int32_t)(data*100);
	uint16_t abs_data=(integer_data<0)?~(-integer_data)+1:integer_data;
	uint8_t high_byte=(abs_data>>8) & 0xFF;
	uint8_t low_byte=abs_data & 0xFF;
	odom[i]=high_byte;
	odom[i+1]=low_byte;
}

void Handle_Odom(void const * argument)
{
  /* USER CODE BEGIN Handle_Odom */
  /* Infinite loop */
  for(;;)
  {
	odom_t odom_data = kinematics.odom();
	Float_As_Complement(odom_data.x,1);
	Float_As_Complement(odom_data.y,3);
	Float_As_Complement(odom_data.quaternion.w,5);
	Float_As_Complement(odom_data.quaternion.x,7);
	Float_As_Complement(odom_data.quaternion.y,9);
	Float_As_Complement(odom_data.quaternion.z,11);
	Float_As_Complement(odom_data.angular_speed,13);
	Float_As_Complement(odom_data.linear_speed,15);
    osDelay(3000);
  }
  /* USER CODE END Handle_Odom */
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
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  MX_TIM12_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();

  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1, twist, sizeof(twist));
  HAL_TIM_Base_Start_IT(&htim6);

  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

  odom[0]=0x55;
  odom[17]=0x1A;
  twist[1]=0x0D;
  twist[2]=0xDC;
  kinematics.set_motor_param(0, 30, 20, 65);
  kinematics.set_motor_param(1, 30, 20, 65);
  kinematics.set_kinematic_param(105,143,167);
  pid_controller[0].update_pid(0.3,0,0);
  pid_controller[1].update_pid(0.3,0,0);
  pid_controller[0].out_limit(-10, 10);
  pid_controller[1].out_limit(-10, 10);

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
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
#ifdef USE_FULL_ASSERT
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
