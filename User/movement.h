/*
 * 文件名: movment.h
 * 描述: 机器鱼运动模块头文件
 * 作者: lili
 * 日期: 20250520
 */


#ifndef __MOVEMENT_H
#define __MOVEMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h" // 包含HAL库头文件，根据实际情况修改

	

//#define SERVO_ID           1    // Define the ID of your servo
//#define SERVO_TARGET_POS_1 4095 // Define target position 1 (e.g., max angle)
//#define SERVO_TARGET_POS_2 0    // Define target position 2 (e.g., min angle)
//#define SERVO_SPEED        2250 // Define movement speed
//#define SERVO_ACCEL        50   // Define movement acceleration
typedef enum {
    MODE_STOP = 0,
    MODE_FORWARD,
    MODE_LEFT,
    MODE_RIGHT,
    // 可以添加更多模式
} FishMode_t;
typedef enum {
    MODE_Down = 0,
    MODE_Horizontal = 2048,
    MODE_Up = 4095,
    // 可以添加更多模式
} FishServo_t;


typedef struct
{
    uint16_t TL;     /* 运动周期 */
		uint16_t TR;     /* 运动周期 */
    uint8_t duty;   /* 占空比 */
		uint8_t offsetL;/*非对称偏置补偿*/
		uint8_t offsetR;/*非对称偏置补偿*/
    FishMode_t mode;/* 运动模式 */
} MovePara_t;

typedef struct
{
    MovePara_t now; /* 当前para */
    MovePara_t set; /* 待设置的para */
    uint8_t running_flag;   /* 运行flag */
    uint16_t time_cnt;      /* 时间计数 */
} MoveCtrl_t;
extern MoveCtrl_t g_MoveCtrlPara_test;
/**
  * @brief 设置参数
  * @param MoveCtrl_t para, 
            uint16_t T, 
            uint8_t duty, 
            FishMode_t mode
  * @retval None
  */
void Movement_ParaSet(MoveCtrl_t *para, uint16_t TL, uint16_t TR, uint8_t duty, uint8_t offsetL, uint8_t offsetR, FishMode_t mode);

void Movement_Ctrl(MoveCtrl_t *para , uint16_t tick_ms);
/**
  * @brief 初始化运动控制模块
  * 配置控制齿轮泵电机的GPIO等
  * @param None
  * @retval None
  */
void Movement_Init(void);

///**
//  * @brief 执行直行运动
//  * 控制齿轮泵电机实现鱼尾往复摆动
//  * @param None
//  * @retval None
//  */
//void Movement_Forward(void);

///**
//  * @brief 执行左转运动
//  * 实现左转运动逻辑，可能需要调整鱼尾摆动或结合其他执行器
//  * @param None
//  * @retval None
//  */
//void Movement_TurnLeft(void);

///**
//  * @brief 执行右转运动
//  * 实现右转运动逻辑
//  * @param None
//  * @retval None
//  */
//void Movement_TurnRight(void);

/**
  * @brief 停止所有运动
  * 停止齿轮泵电机等
  * @param None
  * @retval None
  */
//void Movement_Stop(void);
uint8_t Movement_Stop(void);
    
#ifdef __cplusplus
}
#endif

#endif /* __MOVEMENT_H */
