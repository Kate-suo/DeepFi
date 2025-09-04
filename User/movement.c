/*
 * 文件名: movment.c
 * 描述: 机器鱼运动模块源文件
 * 作者: lili
 * 日期: 20250520
 */
 #include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "movement.h"
#include "SCSerail.h"     // Include the modified serial communication header
#include "uart_hal.h"     // Assuming this provides uart_init or similar
#include <stdio.h>        // 包含stdio.h 以使用printf和scanf
extern TIM_HandleTypeDef htim2; // Declare htim2 if it's defined elsewhere

MoveCtrl_t g_MoveCtrlPara_test;

void set_motor_speed_direction(int speed, int direction)
{
  // Ensure speed is within valid range (0-100)
  if (speed < 0) speed = 0;
  if (speed > 100) speed = 100;

  // Calculate the pulse value based on the speed and timer period
  // Note: This assumes TIM2 is configured for PWM output on Channel 1 and 2
  uint32_t pulse_value = (uint32_t)(((float)speed / 100.0) * (__HAL_TIM_GET_AUTORELOAD(&htim2)) );

  if (direction == 1) // Forward (assuming this means one direction for the pump)
  {
    // Set PWM on Channel 1, Channel 2 to 0
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_value);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  }
  else if (direction == -1) // Reverse (assuming this means the other direction for the pump)
  {
    // Set PWM on Channel 2, Channel 1 to 0
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_value);
  }
  else // Stop (direction == 0)
  {
    // Set both channels to 0
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  }
}

/**
  * @brief 初始化运动控制模块
  * @param None
  * @retval None
  */
    
void Movement_Init(void)
{
    /* 置0后初始化 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    // 初始化控制齿轮泵电机的GPIO (PA0, PA1)
}

///**
//  * @brief 执行直行运动
//  * @param None
//  * @retval None
//  */
//void Movement_Forward(void)
//{
//    set_motor_speed_direction(100, 1); // 示例：左轉時可能調整速度
//    HAL_Delay(170); // Run for 3 seconds
//    set_motor_speed_direction(100,-1); // 示例：左轉時可能調整速度
//    HAL_Delay(340); // Run for 3 seconds
//    set_motor_speed_direction(100, 1); // 示例：左轉時可能調整速度
//    HAL_Delay(170); // Run for 3 seconds
//    // 控制PA0和PA1产生合适的PWM或电平信号，驱动齿轮泵电机往复运动，实现鱼尾摆动
//}

///**
//  * @brief 执行左转运动
//  * @param None
//  * @retval None
//  */
//void Movement_TurnLeft(void)
//{
//    set_motor_speed_direction(100, 1); // 示例：左轉時可能調整速度
//    HAL_Delay(170); // Run for 3 seconds
//    set_motor_speed_direction(100,-1); // 示例：左轉時可能調整速度
//    HAL_Delay(170); // Run for 3 seconds
//    set_motor_speed_direction(100, 1); // 示例：左轉時可能調整速度
//    HAL_Delay(170); // Run for 3 seconds

//    // 实现左转运动逻辑，可能需要调整鱼尾摆动方式或结合其他执行器
//}

///**
//  * @brief 执行右转运动
//  * @param None
//  * @retval None
//  */
//void Movement_TurnRight(void)
//{
//    set_motor_speed_direction(100,-1); // 示例：左轉時可能調整速度
//    HAL_Delay(170); // Run for 3 seconds
//    set_motor_speed_direction(100,1); // 示例：左轉時可能調整速度
//    HAL_Delay(170); // Run for 3 seconds
//    set_motor_speed_direction(100,-1); // 示例：左轉時可能調整速度
//    HAL_Delay(170); // Run for 3 seconds
//    // 实现右转运动逻辑
//}

/**
  * @brief 执行直行运动
  * @param duty:占空比对应鱼摆尾幅度和速度
							T:周期对应鱼尾摆动频率
  * @retval None
  */
static uint8_t Movement_Forward(uint16_t TL, uint16_t TR, uint8_t duty, uint8_t offsetL, uint8_t offsetR, uint16_t time_cnt)
{
    static uint8_t state = 0;
    
    switch(state)
    {
        case 0:
            set_motor_speed_direction(duty-offsetL ,1); // 示例：左摆
            if(time_cnt == TL )
            {
                state += 1;
            }
            break;
        case 1:
            set_motor_speed_direction(duty-offsetR,-1); // 示例：右摆
            if(time_cnt == TL + TR)
            {
                state = 0;
                return 0;
            }
            break;
        default:
            break;
    }
    return 1;
}

/**
  * @brief 执行左转运动
  * @param None
  * @retval None
  */
static uint8_t Movement_TurnLeft(uint16_t TL, uint16_t TR, uint8_t duty, uint8_t offsetL, uint8_t offsetR, uint16_t time_cnt)
{
		TL=TL+400;
		TR=TR+300;
    static uint8_t state = 0;
    static uint8_t L = 0;
    switch(state)
    {
        case 0:
            set_motor_speed_direction(duty-offsetL ,1); // 示例：左轉時可能調整速度
            if(time_cnt == TL )
            {
                state += 1;
            }
            break;
        case 1:
            set_motor_speed_direction(duty-offsetR-L,-1); // 示例：左轉時可能調整速度
            if(time_cnt == TL + TR)
            {
                state = 0;
                return 0;
            }
            break;
        default:
            break;
    }
    return 1;
}

/**
  * @brief 执行右转运动
  * @param None
  * @retval None
  */
static uint8_t Movement_TurnRight(uint16_t TL, uint16_t TR, uint8_t duty, uint8_t offsetL, uint8_t offsetR, uint16_t time_cnt)
{
		TR=TR+400;
		TL=TL+300;
    static uint8_t state = 0;
    static uint8_t R = 0;
    switch(state)
    {
        case 0:
            set_motor_speed_direction(duty-offsetR ,-1); // 示例：左轉時可能調整速度
            if(time_cnt == TR )
            {
                state += 1;
            }
            break;
        case 1:
            set_motor_speed_direction(duty-offsetL-R,1); // 示例：左轉時可能調整速度
            if(time_cnt == TR + TL)
            {
                state = 0;
                return 0;
            }
            break;
        default:
            break;
    }
    return 1;
    
    // 实现右转运动逻辑
}

/**
  * @brief 停止所有运动
  * @param None
  * @retval None
  */
uint8_t Movement_Stop(void)
{
    set_motor_speed_direction(0, 0);
    // 停止齿轮泵电机
    return 0;
}

void Movement_ParaSet(MoveCtrl_t *para, uint16_t TL, uint16_t TR, uint8_t duty, uint8_t offsetL, uint8_t offsetR, FishMode_t mode)
{
    para->set.duty = duty;
    para->set.TL = TL;
		para->set.TR = TR;
    para->set.mode = mode;
		para->set.offsetL = offsetL;
		para->set.offsetR = offsetR;
}

void Movement_Ctrl(MoveCtrl_t *para , uint16_t tick_ms)
{
    if(para->running_flag == 0)
    {
        para->now.duty = para->set.duty;
        para->now.offsetL = para->set.offsetL;
			  para->now.offsetR = para->set.offsetR;
        para->now.TL = para->set.TL;
				para->now.TR = para->set.TR;
        para->now.mode = para->set.mode;
        para->running_flag = 1;
        para->time_cnt = 0;
    }
    
    switch( (para->now.mode) )
    {
        case MODE_STOP:
            para->running_flag = Movement_Stop();
            break;
        case MODE_FORWARD:
            para->running_flag = Movement_Forward(para->now.TL,para->now.TR, para->now.duty, para->now.offsetL, para->now.offsetR, para->time_cnt);
            break;
        case MODE_LEFT:  
            para->running_flag = Movement_TurnLeft(para->now.TL,para->now.TR, para->now.duty, para->now.offsetL, para->now.offsetR, para->time_cnt);
            break;
        case MODE_RIGHT:
            para->running_flag = Movement_TurnRight(para->now.TL,para->now.TR, para->now.duty, para->now.offsetL, para->now.offsetR, para->time_cnt);
            break;
        default:
            para->running_flag = Movement_Stop();
            break;
    }
    
    para->time_cnt += tick_ms;  /* 自增 */
    
}

// depth_sensor.c
/**
  * @brief 初始化深度传感模块
  * @param None
  * @retval None
  */



