/**
  ******************************************************************************
  * @file  vl53l1_platform.c
  * @author  STMicroelectronics (��� ST �������� Gemini AI �m��)
  * @brief   VL53L1 ƽ̨�ض���ʽ���F (�m��� STM32 HAL)
  ******************************************************************************
  * @attention
  *
  * �ˌ��Fʹ�� STM32 HAL ��ʽ���M�� I2C ͨ�š�
  * Ո�_������ I2C ���O (���� hi2c1) �������Č��������_��ʼ��
  * (ͨ���� STM32CubeMX �� i2c.c �����)��
  *
  * �@Щ��ʽ�е� 'dev_addr' ������ VL53L1X �Мy���� 8 λԪ I2C ���O��λַ��
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h" // ���� VL53L1_Dev_t, ��ʽ����
#include "stm32l4xx_hal.h"   // STM32 HAL ��ʽ�� (�������ϵ�в�ͬ��Ո�{�� L4)
#include <string.h>          // ��� memcpy
#include "vl53l1_error_codes.h"
#include "i2c.h"

//************************************User Define *********************************************


//************************************User Define *********************************************



/*
 * I2C Handle ���档
 * ���O 'hi2c1' �����B�ӵ� VL53L1X �� I2C �R���ŵ� I2C_HandleTypeDef��
 * �� handle ͨ���� 'i2c.c' �ж��x���K�� 'i2c.h' ������� 'extern'��
 * ������� I2C handle ���Q��ͬ����ʹ�ò�ͬ�� I2C ���O��Ո�ڴ�̎�{����
 * �K�_������ 'i2c.h' (���Ч�n��)��
 */
extern I2C_HandleTypeDef hi2c1; // ���Oʹ�� I2C1�������ͬ��Ո���ġ�

#define VL53L1_I2C_TIMEOUT_MS 100   // ͨ�� I2C ��r�r�g (����)
#define MAX_I2C_XFER_SIZE   258     // ��� I2C ��ݔ��С (2 λԪ�M������λַ + ��� 256 λԪ�M�Y��)

/**
  * @brief  �� I2C �O�䌑�����λԪ�M��
  * @param  dev_addr    �O��λַ (8λԪ��API �{����ͨ�����ṩ HAL ����ĸ�ʽ)
  * @param  index   Ҫ����� 16 λԪ������������
  * @param  pdata   ָ��Ҫ�����Y�ϵľ��n�^ָ�ˡ�
  * @param  count   Ҫ�����λԪ�M����
  * @retval 0 ��ʾ�ɹ��������ʾʧ����VL53L1_ERROR_NONE (0) on success.
  */
int8_t VL53L1_WriteMulti(uint16_t dev_addr, uint16_t index, uint8_t *pdata, uint32_t count) {
    HAL_StatusTypeDef hal_status;
    uint8_t buffer[MAX_I2C_XFER_SIZE];

    if (count + 2 > MAX_I2C_XFER_SIZE) { // 2 λԪ�M��축�����λַ
        // printf("VL53L1_WriteMulti: Error - write count (%lu) exceeds buffer\r\n", count);
        return VL53L1_ERROR_INVALID_PARAMS;
    }

    // ǰ�ɂ�λԪ�M�� 16 λԪ���������� (��λԪ�M��ǰ)
    buffer[0] = (uint8_t)(index >> 8);
    buffer[1] = (uint8_t)(index & 0x00FF);

    // ������Y��Ҫ���룬�t�}�u������֮��
    if (count > 0) {
        memcpy(&buffer[2], pdata, count);
    }

    // �l�;��n�^ (���������� + �Y��)
    // dev_addr �������� 8 λԪ���O��λַ
    hal_status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr, buffer, count + 2, VL53L1_I2C_TIMEOUT_MS);

    if (hal_status == HAL_OK) {
        // printf("VL53L1_WriteMulti: Dev 0x%02X, Index 0x%04X, Count %lu - OK\r\n", dev_addr, index, count);
        return VL53L1_ERROR_NONE; // �ɹ�
    } else {
        // printf("VL53L1_WriteMulti: Dev 0x%02X, Index 0x%04X - HAL Error: %d\r\n", dev_addr, index, hal_status);
        return VL53L1_ERROR_CONTROL_INTERFACE; // I2C ͨ���e�`
    }
}

/**
  * @brief  �� I2C �O���xȡ����λԪ�M��
  * @param  dev_addr    �O��λַ (8λԪ)
  * @param  index   Ҫ�xȡ�� 16 λԪ������������
  * @param  pdata   ָ�򃦴��xȡ�Y�ϵľ��n�^ָ�ˡ�
  * @param  count   Ҫ�xȡ��λԪ�M����
  * @retval 0 ��ʾ�ɹ��������ʾʧ����VL53L1_ERROR_NONE (0) on success.
  */
int8_t VL53L1_ReadMulti(uint16_t dev_addr, uint16_t index, uint8_t *pdata, uint32_t count) {
    HAL_StatusTypeDef hal_status;
    uint8_t reg_addr_buffer[2];

    // �ʂ�Ҫ�l�͵� 16 λԪ���������� (��λԪ�M��ǰ)
    reg_addr_buffer[0] = (uint8_t)(index >> 8);
    reg_addr_buffer[1] = (uint8_t)(index & 0x00FF);

    // ���ȣ����� 16 λԪ������������ָ���ĺ�̎�xȡ
    hal_status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr, reg_addr_buffer, 2, VL53L1_I2C_TIMEOUT_MS);

    if (hal_status != HAL_OK) {
        // printf("VL53L1_ReadMulti: Write Index 0x%04X for Dev 0x%02X - HAL Error: %d\r\n", index, dev_addr, hal_status);
        return VL53L1_ERROR_CONTROL_INTERFACE; // I2C ͨ���e�`
    }

    // Ȼ�ᣬ��ԓ�������xȡָ��������λԪ�M
    hal_status = HAL_I2C_Master_Receive(&hi2c1, dev_addr, pdata, count, VL53L1_I2C_TIMEOUT_MS);

    if (hal_status == HAL_OK) {
        // printf("VL53L1_ReadMulti: Dev 0x%02X, Index 0x%04X, Count %lu - OK\r\n", dev_addr, index, count);
        return VL53L1_ERROR_NONE; // �ɹ�
    } else {
        // printf("VL53L1_ReadMulti: Read Data from Index 0x%04X for Dev 0x%02X - HAL Error: %d\r\n", index, dev_addr, hal_status);
        return VL53L1_ERROR_CONTROL_INTERFACE; // I2C ͨ���e�`
    }
}

/**
  * @brief  �� I2C �O�䌑���һλԪ�M��
  * @param  dev_addr    �O��λַ
  * @param  index   Ҫ����� 16 λԪ������������
  * @param  data    Ҫ�����λԪ�M��
  * @retval 0 ��ʾ�ɹ��������ʾʧ����
  */
int8_t VL53L1_WrByte(uint16_t dev_addr, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(dev_addr, index, &data, 1);
}

/**
  * @brief  �� I2C �O�䌑��һ�� 16 λԪ�֡�
  * @param  dev_addr    �O��λַ
  * @param  index   Ҫ����� 16 λԪ������������
  * @param  data    Ҫ����� 16 λԪ�� (��λԪ�M��ǰ)��
  * @retval 0 ��ʾ�ɹ��������ʾʧ����
  */
int8_t VL53L1_WrWord(uint16_t dev_addr, uint16_t index, uint16_t data) {
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(data >> 8);   // MSB
    buffer[1] = (uint8_t)(data & 0xFF); // LSB
    return VL53L1_WriteMulti(dev_addr, index, buffer, 2);
}

/**
  * @brief  �� I2C �O�䌑��һ�� 32 λԪ�p�֡�
  * @param  dev_addr    �O��λַ
  * @param  index   Ҫ����� 16 λԪ������������
  * @param  data    Ҫ����� 32 λԪ�p�� (��λԪ�M��ǰ)��
  * @retval 0 ��ʾ�ɹ��������ʾʧ����
  */
int8_t VL53L1_WrDWord(uint16_t dev_addr, uint16_t index, uint32_t data) {
    uint8_t buffer[4];
    buffer[0] = (uint8_t)(data >> 24); // MSB
    buffer[1] = (uint8_t)((data >> 16) & 0xFF);
    buffer[2] = (uint8_t)((data >> 8) & 0xFF);
    buffer[3] = (uint8_t)(data & 0xFF); // LSB
    return VL53L1_WriteMulti(dev_addr, index, buffer, 4);
}

/**
  * @brief  �� I2C �O���xȡ��һλԪ�M��
  * @param  dev_addr    �O��λַ
  * @param  index   Ҫ�xȡ�� 16 λԪ������������
  * @param  pdata   ָ�򃦴��xȡλԪ�M��ָ�ˡ�
  * @retval 0 ��ʾ�ɹ��������ʾʧ����
  */
int8_t VL53L1_RdByte(uint16_t dev_addr, uint16_t index, uint8_t *pdata) {
    return VL53L1_ReadMulti(dev_addr, index, pdata, 1);
}

/**
  * @brief  �� I2C �O���xȡһ�� 16 λԪ�֡�
  * @param  dev_addr    �O��λַ
  * @param  index   Ҫ�xȡ�� 16 λԪ������������
  * @param  pdata   ָ�򃦴��xȡ 16 λԪ�ֵ�ָ�� (��λԪ�M��ǰ)��
  * @retval 0 ��ʾ�ɹ��������ʾʧ����
  */
int8_t VL53L1_RdWord(uint16_t dev_addr, uint16_t index, uint16_t *pdata) {
    uint8_t buffer[2];
    int8_t status = VL53L1_ReadMulti(dev_addr, index, buffer, 2);
    if (status == VL53L1_ERROR_NONE) {
        *pdata = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    }
    return status;
}

/**
  * @brief  �� I2C �O���xȡһ�� 32 λԪ�p�֡�
  * @param  dev_addr    �O��λַ
  * @param  index   Ҫ�xȡ�� 16 λԪ������������
  * @param  pdata   ָ�򃦴��xȡ 32 λԪ�p�ֵ�ָ�� (��λԪ�M��ǰ)��
  * @retval 0 ��ʾ�ɹ��������ʾʧ����
  */
int8_t VL53L1_RdDWord(uint16_t dev_addr, uint16_t index, uint32_t *pdata) {
    uint8_t buffer[4];
    int8_t status = VL53L1_ReadMulti(dev_addr, index, buffer, 4);
    if (status == VL53L1_ERROR_NONE) {
        *pdata = ((uint32_t)buffer[0] << 24) |
                 ((uint32_t)buffer[1] << 16) |
                 ((uint32_t)buffer[2] << 8)  |
                 (uint32_t)buffer[3];
    }
    return status;
}

/**
  * @brief  ƽ̨�ض��ĺ��뼉���t��
  * @param  dev_addr    �O��λַ (�˺�ʽ��δʹ��)
  * @param  wait_ms   ���t�r�g (����)��
  * @retval 0 ��ʾ�ɹ���
  */
int8_t VL53L1_WaitMs(uint16_t dev_addr, int32_t wait_ms) {
    (void)dev_addr; // δʹ�õą���
    HAL_Delay((uint32_t)wait_ms);
    return VL53L1_ERROR_NONE;
}
