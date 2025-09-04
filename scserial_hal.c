/*
 * SCSerail.c
 * ���Ӳ���ӿڲ����
 * ����: 2022.3.29
 * ����:
 * Ǩ�Ƶ� STM32L4xx HAL ��
 */

#include "stm32l4xx_hal.h" // Include the HAL library header for STM32L4 series
#include "uart_hal.h"          // Include the modified UART header
#include "usart.h"
uint32_t IOTimeOut = 5000; // ͨ�ų�ʱ (��λ: ѭ���������Ǿ�ȷʱ��)
uint8_t wBuf[128];
uint8_t wLen = 0;

// UART �������ݽӿڣ�����ʱ
// nDat: ���ջ�����
// nLen: �������յ��ֽ���
// TimeOut: ��ʱʱ�� (��λ: us)
// ����ֵ: ʵ�ʽ��յ����ֽ���
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

// UART �������ݽӿ�
// nDat: ���ջ�����
// nLen: �������յ��ֽ���
// ����ֵ: ʵ�ʽ��յ����ֽ���
int readSC(unsigned char *nDat, int nLen)
{
	// This function seems to rely on a timeout defined by IOTimeOut
	// We will use the readSCTimeOut function with IOTimeOut converted to milliseconds
	// Note: The original IOTimeOut unit is not explicitly defined, assuming it's related to loop counts.
	// We'll interpret it as milliseconds for use with HAL_GetTick.
	return readSCTimeOut(nDat, nLen, IOTimeOut); // Use the timeout version
}

// UART �������ݽӿ�
// nDat: ���ͻ�����
// nLen: ���͵��ֽ���
// ����ֵ: ʵ��д�뷢�ͻ��������ֽ��� (�� wFlushSC ��ʵ�ʷ���)
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

// д�뵥���ֽڵ����ͻ�����
// bDat: Ҫд����ֽ�
// ����ֵ: ���ͻ�������ǰ����
int writeByteSC(unsigned char bDat)
{
	if(wLen < sizeof(wBuf)){
		wBuf[wLen] = bDat;
		wLen++;
	}
	return wLen;
}

// �����л���ʱ
void SCDelay(void)
{
	// Original delay was a simple loop, estimated around 10us based on typical clock speeds.
	// Using a microsecond delay function from uart.c
	delay_us(10); // Adjust this value if needed based on servo requirements
}

// ���ջ�����ˢ��
void rFlushSC()
{
	SCDelay(); // Delay before flushing, as in original code
    // Flush the receive buffer by reading the data register until RXNE is clear
    while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    {
        (void)huart1.Instance->RDR; // Read data register to clear RXNE flag
    }
}

// ���ͻ�����ˢ��
void wFlushSC()
{
	if(wLen > 0){
		uart_send_data(wBuf, wLen); // Use the HAL-based send function
		wLen = 0; // Clear the buffer length after sending
	}
}
