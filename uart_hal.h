#ifndef __UART_H
#define __UART_H

#include "stm32l4xx_hal.h" // Include the HAL library header for STM32L4 series

// Define the UART instance and pins
#define SERVO_UART_INSTANCE         USART1
#define SERVO_UART_CLK_ENABLE()     __HAL_RCC_USART1_CLK_ENABLE()
#define SERVO_UART_RX_GPIO_PORT     GPIOA
#define SERVO_UART_TX_GPIO_PORT     GPIOA
#define SERVO_UART_RX_GPIO_PIN      GPIO_PIN_10
#define SERVO_UART_TX_GPIO_PIN      GPIO_PIN_9
#define SERVO_UART_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define SERVO_UART_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

// Define the UART Handle
//extern UART_HandleTypeDef hservo_uart;

// Function prototypes
void uart_init(uint32_t baudrate);
void uart_send_data(uint8_t *data, uint16_t size);
uint16_t uart_receive_data(uint8_t *data, uint16_t size, uint32_t timeout);
void delay_us(uint32_t us); // Simple microsecond delay (may need more accurate implementation)
void delay_ms(uint32_t ms); // Millisecond delay using HAL_Delay

#endif /* __UART_H */
