/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal_flash.h"
/* VL53L1X API ���� */
#include "VL53L1X_user.h"
/* ��� API ���� */
#include "SC.h"
#include "INST.h"          // Include the instruction set header
#include "ST.h"            // Include SC library header
#include "SCSerail.h"      // Include the modified serial communication header
#include "uart_hal.h"
// ����ң��ģ��ͷ�ļ�
#include "ir_remote.h"    
// �˶�����ģ��ͷ�ļ�
#include "movement.h"     
// PID����ģ��ͷ�ļ�
#include "pid_controller.h" 
//printf�ض���
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ȫ�ֱ��� */
//Ledָʾ��
#define LED_Pin			GPIO_PIN_12 
#define LED_Port		GPIOA   
// Servo definitions based on user's snippet
#define SERVO_ID1           1    // Define the ID of your servo
#define SERVO_ID2           2    // Define the ID of your servo
#define SERVO_TARGET_POS_1 2048 // Define target position 1 (e.g., zero angle)
#define SERVO_TARGET_POS_2 1536     // Define target position 2 (e.g., 45 angle)
#define SERVO_TARGET_POS_3 2560     // Define target position 2 (e.g., -45 angle)
#define SERVO_SPEED        2250 // Define movement speed
#define SERVO_ACCEL        50   // Define movement acceleration
volatile char received_command_char; // ����� USART2 ���յ����ַ�
volatile uint8_t new_command_flag = 0; // ��������՘��I
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ��������㹤��ģʽ


// ��ǰ����ģʽ
MoveCtrl_t myFishControl;
volatile uint32_t g_tick_ms = 0;

FishMode_t current_mode = MODE_STOP;
FishServo_t Servo_mode = MODE_Horizontal;
uint16_t Distance_mm;
uint8_t RangeStatus;
uint8_t DataReady;
// Ŀ����� (���磬��λ�����Ǻ���)
float target_depth = 100.0; // ʾ����Ŀ�����10cm

// PID������ʵ��
PIDController_t depth_pid;
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
  MX_TIM2_Init(); 
  MX_USART1_UART_Init();
  MX_I2C1_Init(); 
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
			HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_SET); 
			HAL_Delay(1000);
			HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
HAL_TIM_Base_Start_IT(&htim6);
		/* �����ʼ��  */
	//	IR_Remote_Init();
		/* �˶���ʼ��  */
    Movement_Init();
int duty=100;			//����
int offsetL=2;		//ƫ��
int offsetR=0;		//ƫ��
int TL=500;      	//���ʱ��
int TR=500;      	//�Ұ�ʱ��
  /* PID��ʼ��  */
    //PID_Controller_Init(&depth_pid, 1.0, 0.1, 0.05, 100); // ��ʼ��PID���� (Kp, Ki, Kd, OutputLimit)
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&received_command_char, 1);
		char command_char_local;
		/* �����ʼ��  */
//			VL53L1X_User_Init();
//			VL53L1X_User_StartRanging();
			
    printf("Robot Fish Control Ready!\r\n");
    printf("Enter command: 'f' for Forward, 'b' for Stop, 'l' for Left, 'r' for Right.\r\n");

  /* USER CODE END 2 */ 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//				/* ι������ */
//    if (HAL_GetTick() - last_feed_time > 1000) { // ÿ1000���� (1��)
//        // printf("ι�� (%lu ms)...\r\n", HAL_GetTick()); // ����ʱ���Դ򿪴˴�ӡ
//        HAL_IWDG_Refresh(&hiwdg);
//        last_feed_time = HAL_GetTick();
//    }
		




 // ����Ƿ�����������յ�
    if (new_command_flag)
    {
			for(int i=0;i<4;i++){
			HAL_GPIO_TogglePin(LED_Port, LED_Pin); 
			HAL_Delay(5);
}
			command_char_local = received_command_char; // ʹ�þֲ��������������
      new_command_flag = 0; // ������I
      printf("\r\n"); // ��̎������ǰ�ȓQ�У���������o������ʾ����
	
      // �������յ����������ģʽ
      switch (command_char_local) // ʹ�þֲ����� command_char_local
      {
        case 'f':
        case 'F':
          current_mode = MODE_FORWARD;
          printf("Mode set to: FORWARD\r\n");
          break;
        case 's': // 'b' for brake/stop
        case 'S':
          current_mode = MODE_STOP;
          printf("Mode set to: STOP\r\n");
          break;
        case 'l':
        case 'L':
          current_mode = MODE_LEFT;
          printf("Mode set to: LEFT\r\n");
          break;
        case 'r':
        case 'R':
          current_mode = MODE_RIGHT;
          printf("Mode set to: RIGHT\r\n");
          break;
        case 'h':
        case 'H':
					Servo_mode = MODE_Horizontal;
		WritePosEx(SERVO_ID1,SERVO_TARGET_POS_1,SERVO_SPEED,SERVO_ACCEL);
		WritePosEx(SERVO_ID2,SERVO_TARGET_POS_1,SERVO_SPEED,SERVO_ACCEL);
          printf("Servo MODE_Horizontal\r\n");
          break;
        case 'u':
        case 'U':
					Servo_mode = MODE_Up;
		WritePosEx(SERVO_ID1,SERVO_TARGET_POS_3,SERVO_SPEED,SERVO_ACCEL);
		WritePosEx(SERVO_ID2,SERVO_TARGET_POS_2,SERVO_SPEED,SERVO_ACCEL);
          printf("Servo MODE_Up\r\n");
          break;
        case 'd':
        case 'D':
          Servo_mode = MODE_Down;
		WritePosEx(SERVO_ID1,SERVO_TARGET_POS_2,SERVO_SPEED,SERVO_ACCEL);
		WritePosEx(SERVO_ID2,SERVO_TARGET_POS_3,SERVO_SPEED,SERVO_ACCEL);
          printf("Servo MODE_Down\r\n");
          break;
        case '+':
				duty+=5;
          printf("duty + 5 percent =%d \r\n",duty);
          break;
        case '-':
				duty-=5;
          printf("duty - 5 percent =%d \r\n",duty);
          break;
        case 'T':
				TL+=100;
				TR+=100;
          printf("T + 100 ms =%d,%d\r\n",TL,TR);
          break;
        case 't':
				TL-=100;
				TR-=100;
          printf("T - 100 ms =%d,%d\r\n",TL,TR);
          break;
        default:
          printf("Unknown command: %c.\r\n", command_char_local);
          break;
      }
			Movement_ParaSet(&g_MoveCtrlPara_test, TL, TR, duty,offsetL,offsetR, current_mode);
      printf("Enter command: 'f' for Forward, 's' for Stop, 'l' for Left, 'r' for Right,'h' for Horizontal,'u' for Up,'d' for Down.\r\n"); // ������ʾݔ��
    }
//	
//VL53L1X_User_GetMeasurement(&Distance_mm, &RangeStatus, &DataReady);


	}
  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void)
{
    g_tick_ms++; // ÿ���жϣ�ȫ�ֺ����������1
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
