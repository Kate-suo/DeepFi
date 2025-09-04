/*
 * SCSerail.c
 * 舵机硬件接口层程序
 * 日期: 2022.3.29
 * 作者:
 * 迁移到 STM32L4xx HAL 库
 */

#include "stm32l4xx_hal.h" // Include the HAL library header for STM32L4 series
#include "uart_hal.h"          // Include the modified UART header
#include "usart.h"
uint32_t IOTimeOut = 5000; // 通信超时 (单位: 循环次数，非精确时间)
uint8_t wBuf[128];
uint8_t wLen = 0;

// UART 接收数据接口，带超时
// nDat: 接收缓冲区
// nLen: 期望接收的字节数
// TimeOut: 超时时间 (单位: us)
// 返回值: 实际接收到的字节数
int readSCTimeOut(unsigned char *nDat, int nLen, uint32_t TimeOut)
{
	int Size = 0;
	int bytes_received;
	uint32_t start_time = HAL_GetTick(); // Get current tick count for timeout

	while(Size < nLen)
	{
		// Attempt to receive one byte with a small timeout
		// We use a small timeout here to avoid blocking indefinitely if only partial data arrives
		// The overall timeout is handled by checking elapsed time
		bytes_received = uart_receive_data(nDat + Size, 1, 1); // Try to receive 1 byte, 1ms timeout

		if(bytes_received == 1)
		{
			Size++;
			start_time = HAL_GetTick(); // Reset timeout on receiving a byte
		}

		// Check for overall timeout
		if((HAL_GetTick() - start_time) > TimeOut)
		{
			break; // Timeout occurred
		}
	}
	return Size;
}

// UART 接收数据接口
// nDat: 接收缓冲区
// nLen: 期望接收的字节数
// 返回值: 实际接收到的字节数
int readSC(unsigned char *nDat, int nLen)
{
	// This function seems to rely on a timeout defined by IOTimeOut
	// We will use the readSCTimeOut function with IOTimeOut converted to milliseconds
	// Note: The original IOTimeOut unit is not explicitly defined, assuming it's related to loop counts.
	// We'll interpret it as milliseconds for use with HAL_GetTick.
	return readSCTimeOut(nDat, nLen, IOTimeOut); // Use the timeout version
}

// UART 发送数据接口
// nDat: 发送缓冲区
// nLen: 发送的字节数
// 返回值: 实际写入发送缓冲区的字节数 (在 wFlushSC 中实际发送)
int writeSC(unsigned char *nDat, int nLen)
{
	int bytes_written = 0;
	while(nLen-- > 0){
		if(wLen < sizeof(wBuf)){
			wBuf[wLen] = *nDat;
			wLen++;
			nDat++;
			bytes_written++;
		} else {
            // Buffer is full, handle error or break
            break;
        }
	}
	return bytes_written;
}

// 写入单个字节到发送缓冲区
// bDat: 要写入的字节
// 返回值: 发送缓冲区当前长度
int writeByteSC(unsigned char bDat)
{
	if(wLen < sizeof(wBuf)){
		wBuf[wLen] = bDat;
		wLen++;
	}
	return wLen;
}

// 总线切换延时
void SCDelay(void)
{
	// Original delay was a simple loop, estimated around 10us based on typical clock speeds.
	// Using a microsecond delay function from uart.c
	delay_us(10); // Adjust this value if needed based on servo requirements
}

// 接收缓冲区刷新
void rFlushSC()
{
	SCDelay(); // Delay before flushing, as in original code
    // Flush the receive buffer by reading the data register until RXNE is clear
    while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    {
        (void)huart1.Instance->RDR; // Read data register to clear RXNE flag
    }
}

// 发送缓冲区刷新
void wFlushSC()
{
	if(wLen > 0){
		uart_send_data(wBuf, wLen); // Use the HAL-based send function
		wLen = 0; // Clear the buffer length after sending
	}
}
