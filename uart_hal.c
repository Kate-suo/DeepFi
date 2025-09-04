#include "uart_hal.h"
#include "stm32l4xx_hal_uart.h" // Include the HAL library header for STM32L4 series
#include "usart.h"
//UART_HandleTypeDef hservo_uart;

// Initialize the UART peripheral
void uart_init(uint32_t baudrate)
{
//    // Enable GPIO clock
//    SERVO_UART_TX_GPIO_CLK_ENABLE();
//    SERVO_UART_RX_GPIO_CLK_ENABLE();

//    // Enable USART clock
//    SERVO_UART_CLK_ENABLE();

//    // Configure UART GPIO pins
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    GPIO_InitStruct.Pin = SERVO_UART_TX_GPIO_PIN | SERVO_UART_RX_GPIO_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate Function Push Pull
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    // Configure the Alternate Function for USART1 (check your STM32L432KBU6 datasheet for the correct AF)
//    // For USART1 on PA9/PA10, the AF is typically AF7
//    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
//    HAL_GPIO_Init(SERVO_UART_TX_GPIO_PORT, &GPIO_InitStruct);

//    // Configure the UART peripheral
//    hservo_uart.Instance = SERVO_UART_INSTANCE;
//    hservo_uart.Init.BaudRate = baudrate;
//    hservo_uart.Init.WordLength = UART_WORDLENGTH_8B;
//    hservo_uart.Init.StopBits = UART_STOPBITS_1;
//    hservo_uart.Init.Parity = UART_PARITY_NONE;
//    hservo_uart.Init.Mode = UART_MODE_TX_RX;
//    hservo_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//    hservo_uart.Init.OverSampling = UART_OVERSAMPLING_16;
//    hservo_uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//    hservo_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

//    if (HAL_UART_Init(&hservo_uart) != HAL_OK)
//    {
//        // Initialization Error
//        // You might want to add an error handling mechanism here
//    }
}

// Send data via UART
void uart_send_data(uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit(&huart1, data, size, HAL_MAX_DELAY); // Use HAL_MAX_DELAY for blocking transmit
}

// Receive data via UART
// Returns the number of bytes received
uint16_t uart_receive_data(uint8_t *data, uint16_t size, uint32_t timeout)
{
    HAL_StatusTypeDef status = HAL_UART_Receive(& huart1, data, size, timeout);
    if (status == HAL_OK)
    {
        return size; // Successfully received all bytes
    }
    else if (status == HAL_TIMEOUT)
    {
        // Handle timeout if necessary
        return 0; // No data received within the timeout
    }
    else
    {
        // Handle other errors (e.g., HAL_ERROR, HAL_BUSY)
        return 0; // Error during reception
    }
}

// Simple microsecond delay (Note: This is a basic implementation and may not be highly accurate)
// For more accurate microsecond delays, consider using a timer or DWT.
void delay_us(uint32_t us)
{
    uint32_t start_tick = SysTick->VAL;
    uint32_t ticks_per_us = SystemCoreClock / 1000000;
    uint32_t required_ticks = us * ticks_per_us;
    uint32_t elapsed_ticks = 0;

    // Wait until the required number of ticks have passed
    // This implementation assumes SysTick is configured to run at the system clock frequency
    // and is counting down.
    while (elapsed_ticks < required_ticks)
    {
        uint32_t current_tick = SysTick->VAL;
        if (current_tick > start_tick) { // Handle SysTick wrap-around
            elapsed_ticks += (SysTick->LOAD + 1 - start_tick) + current_tick;
        } else {
            elapsed_ticks += start_tick - current_tick;
        }
        start_tick = current_tick;
    }
}


// Millisecond delay using HAL_Delay
void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
