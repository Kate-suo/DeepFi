/*
 * 文件名: pid_controller.h
 * 描述: PID控制器模块头文件
 * 作者: lili
 * 日期: 20250520
 */

#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h> // 包含stdint.h以使用标准整数类型，如int32_t或float

/**
  * @brief PID控制器结构体定义
  */
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数

    float target_value; // 目标值 (设定值)
    float current_value;  // 当前值 (测量值)

    float integral_error; // 积分项累积误差
    float previous_error; // 上一次的误差，用于计算微分项

    float output_limit; // 输出限幅

    // 可以根据需要添加其他成员，例如：
    // uint32_t last_update_time; // 上次更新时间戳，用于离散PID计算
    // float proportional_term; // 比例项输出
    // float integral_term;     // 积分项输出
    // float derivative_term;   // 微分项输出

} PIDController_t;

/**
  * @brief 初始化PID控制器
  * @param pid PID控制器结构体指针
  * @param Kp 比例系数
  * @param Ki 积分系数
  * @param Kd 微分系数
  * @param output_limit 输出限幅
  * @retval None
  */
void PID_Controller_Init(PIDController_t *pid, float Kp, float Ki, float Kd, float output_limit);

/**
  * @brief 更新PID控制器并计算输出
  * @param pid PID控制器结构体指针
  * @param target_value 目标值
  * @param current_value 当前值
  * @retval PID输出值
  */
float PID_Controller_Update(PIDController_t *pid, float target_value, float current_value);


#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H */
