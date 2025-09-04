#ifndef __IR_REMOTE_H
#define __IR_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h" // 包含HAL庫頭文件
// #include <stdio.h> // 移除stdio.h，因為不再使用printf

// 定義紅外命令碼，這些值需要根據你實際使用的遙控器和解碼結果來確定
// 這裡的數值是示例，你需要根據你的遙控器按鍵實際解碼出的數據碼進行修改
// 根據您提供的遙控器鍵位碼進行更新
#define IR_COMMAND_NONE     0xFFFFFFFF // 沒有接收到新命令

// 假設遙控器按鍵功能映射 (請根據實際遙控器按鍵和您的需求進行映射)
// 圖片中的鍵位碼 (十六進位)
#define IR_KEY_45           0x45
#define IR_KEY_46           0x46
#define IR_KEY_47           0x47
#define IR_KEY_44           0x44
#define IR_KEY_40           0x40
#define IR_KEY_43           0x43
#define IR_KEY_07           0x07
#define IR_KEY_15           0x15
#define IR_KEY_09           0x09
#define IR_KEY_16           0x16
#define IR_KEY_19           0x19
#define IR_KEY_0D           0x0D
#define IR_KEY_0C           0x0C
#define IR_KEY_18           0x18 
#define IR_KEY_5E           0x5E
#define IR_KEY_08           0x08
#define IR_KEY_1C           0x1C
#define IR_KEY_5A           0x5A
#define IR_KEY_42           0x42
#define IR_KEY_52           0x52
#define IR_KEY_4A           0x4A

// 將遙控器鍵位碼映射到機器魚的動作
// 請根據您的遙控器按鍵和期望功能進行選擇和映射
#define IR_COMMAND_START    IR_KEY_45   // 示例：假設45鍵是啟動
#define IR_COMMAND_FORWARD  IR_KEY_16   // 示例：假設16鍵是直行
#define IR_COMMAND_LEFT     IR_KEY_18   // 示例：假設18鍵是左轉 (與您之前程式碼中的0x18匹配)
#define IR_COMMAND_RIGHT    IR_KEY_40   // 示例：假設40鍵是右轉
#define IR_COMMAND_STOP     IR_KEY_0C   // 示例：假設0C鍵是停止

/**
  * @brief 初始化紅外遙控模組（GPIO和定時器）
  * @param None
  * @retval None
  */
void IR_Remote_Init(void);

/**
  * @brief 讀取紅外遙控命令
  * @param None
  * @retval 紅外命令碼，如果沒有新命令則返回IR_COMMAND_NONE
  */
uint32_t IR_Remote_ReadCommand(void);

// 外部中斷回呼函數，需要在stm32l4xx_it.c中呼叫
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* __IR_REMOTE_H */
