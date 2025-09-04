/*
 * 文件名: pid_controller.c
 * 描述: 机器鱼p模块源文件
 * 作者: lili
 * 日期: 20250520
 */
 #include "pid_controller.h"
/**
  * @brief 初始化PID控制器
  * @param pid PID控制器结构体指针
  * @param Kp 比例系数
  * @param Ki 积分系数
  * @param Kd 微分系数
  * @param output_limit 输出限幅
  * @retval None
  */
void PID_Controller_Init(PIDController_t *pid, float Kp, float Ki, float Kd, float output_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_limit = output_limit;
    pid->integral_error = 0;
    pid->previous_error = 0;
}

/**
  * @brief 更新PID控制器并计算输出
  * @param pid PID控制器结构体指针
  * @param target_value 目标值
  * @param current_value 当前值
  * @retval PID输出值
  */
float PID_Controller_Update(PIDController_t *pid, float target_value, float current_value)
{
    float error = target_value - current_value;
    pid->integral_error += error;
    float derivative_error = error - pid->previous_error;
    float output = pid->Kp * error + pid->Ki * pid->integral_error + pid->Kd * derivative_error;

    // 输出限幅
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
         
    }

    pid->previous_error = error;
    return output;
}

