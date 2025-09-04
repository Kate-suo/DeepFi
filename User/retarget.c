/*
 * 文件名: retarget.c
 * 描述: printf 重定向到 UART
 * 作者: lili
 * 日期: 20250520
 */

#include <stdio.h> // 包含标准输入输出头文件
#include "usart.h" // 包含你的UART初始化头文件，这里假设是usart.h
#include "stm32l4xx_hal.h" // 包含 STM32 HAL 庫定義 (用於 GPIO_PinState 等)
// 定义用于printf输出的UART句柄
// 假设你使用USART2作为printf输出，并且在usart.c中定义了huart2
extern UART_HandleTypeDef huart2; // 根据你的实际UART句柄名称修改
#define LED_Pin			GPIO_PIN_12 
#define LED_GPIO_Port		GPIOA   

// 定義用於 printf 輸出的 UART 句柄
// 假設你使用 USART2 作為 printf 輸出，並且在 usart.c 中定義了 huart2
extern UART_HandleTypeDef huart2; // 根據你的實際 UART 句柄名稱修改

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  ch: character to send
  * @param  f: pointer to file (not typically used in embedded retargeting)
  * @retval The character sent
  */
int fputc(int ch, FILE *f)
{
    // 使用 HAL 庫函數發送單個字符
    // 這裡假設你使用 USART2，並且其句柄是 huart2
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);

    return ch;
}

// 如果你需要 scanf 等輸入功能，還需要實現 fgetc 函數

int fgetc(FILE *f)
{
    uint8_t ch;
    // 使用 HAL 庫函數接收單個字符
    // 這裡假設你使用 USART2，並且其句柄是 huart2
    HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);
    return ch;
}
