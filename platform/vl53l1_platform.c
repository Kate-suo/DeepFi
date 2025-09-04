/**
  ******************************************************************************
  * @file  vl53l1_platform.c
  * @author  STMicroelectronics (基於 ST 範例，由 Gemini AI 適配)
  * @brief   VL53L1 平台特定函式實現 (適用於 STM32 HAL)
  ******************************************************************************
  * @attention
  *
  * 此實現使用 STM32 HAL 函式庫進行 I2C 通信。
  * 請確保您的 I2C 外設 (例如 hi2c1) 已在您的專案中正確初始化
  * (通常由 STM32CubeMX 在 i2c.c 中完成)。
  *
  * 這些函式中的 'dev_addr' 參數是 VL53L1X 感測器的 8 位元 I2C 從設備位址。
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h" // 包含 VL53L1_Dev_t, 函式宣告
#include "stm32l4xx_hal.h"   // STM32 HAL 函式庫 (如果您的系列不同，請調整 L4)
#include <string.h>          // 用於 memcpy
#include "vl53l1_error_codes.h"
#include "i2c.h"

//************************************User Define *********************************************


//************************************User Define *********************************************



/*
 * I2C Handle 宣告。
 * 假設 'hi2c1' 是您連接到 VL53L1X 的 I2C 匯流排的 I2C_HandleTypeDef。
 * 此 handle 通常在 'i2c.c' 中定義，並在 'i2c.h' 中宣告為 'extern'。
 * 如果您的 I2C handle 名稱不同，或使用不同的 I2C 外設，請在此處調整，
 * 並確保包含 'i2c.h' (或等效檔案)。
 */
extern I2C_HandleTypeDef hi2c1; // 假設使用 I2C1。如果不同，請更改。

#define VL53L1_I2C_TIMEOUT_MS 100   // 通用 I2C 逾時時間 (毫秒)
#define MAX_I2C_XFER_SIZE   258     // 最大 I2C 傳輸大小 (2 位元組暫存器位址 + 最多 256 位元組資料)

/**
  * @brief  向 I2C 設備寫入多個位元組。
  * @param  dev_addr    設備位址 (8位元，API 調用者通常已提供 HAL 所需的格式)
  * @param  index   要寫入的 16 位元暫存器索引。
  * @param  pdata   指向要寫入資料的緩衝區指標。
  * @param  count   要寫入的位元組數。
  * @retval 0 表示成功，非零表示失敗。VL53L1_ERROR_NONE (0) on success.
  */
int8_t VL53L1_WriteMulti(uint16_t dev_addr, uint16_t index, uint8_t *pdata, uint32_t count) {
    HAL_StatusTypeDef hal_status;
    uint8_t buffer[MAX_I2C_XFER_SIZE];

    if (count + 2 > MAX_I2C_XFER_SIZE) { // 2 位元組用於暫存器位址
        // printf("VL53L1_WriteMulti: Error - write count (%lu) exceeds buffer\r\n", count);
        return VL53L1_ERROR_INVALID_PARAMS;
    }

    // 前兩個位元組是 16 位元暫存器索引 (高位元組在前)
    buffer[0] = (uint8_t)(index >> 8);
    buffer[1] = (uint8_t)(index & 0x00FF);

    // 如果有資料要寫入，則複製到索引之後
    if (count > 0) {
        memcpy(&buffer[2], pdata, count);
    }

    // 發送緩衝區 (暫存器索引 + 資料)
    // dev_addr 參數已是 8 位元從設備位址
    hal_status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr, buffer, count + 2, VL53L1_I2C_TIMEOUT_MS);

    if (hal_status == HAL_OK) {
        // printf("VL53L1_WriteMulti: Dev 0x%02X, Index 0x%04X, Count %lu - OK\r\n", dev_addr, index, count);
        return VL53L1_ERROR_NONE; // 成功
    } else {
        // printf("VL53L1_WriteMulti: Dev 0x%02X, Index 0x%04X - HAL Error: %d\r\n", dev_addr, index, hal_status);
        return VL53L1_ERROR_CONTROL_INTERFACE; // I2C 通信錯誤
    }
}

/**
  * @brief  從 I2C 設備讀取多個位元組。
  * @param  dev_addr    設備位址 (8位元)
  * @param  index   要讀取的 16 位元暫存器索引。
  * @param  pdata   指向儲存讀取資料的緩衝區指標。
  * @param  count   要讀取的位元組數。
  * @retval 0 表示成功，非零表示失敗。VL53L1_ERROR_NONE (0) on success.
  */
int8_t VL53L1_ReadMulti(uint16_t dev_addr, uint16_t index, uint8_t *pdata, uint32_t count) {
    HAL_StatusTypeDef hal_status;
    uint8_t reg_addr_buffer[2];

    // 準備要發送的 16 位元暫存器索引 (高位元組在前)
    reg_addr_buffer[0] = (uint8_t)(index >> 8);
    reg_addr_buffer[1] = (uint8_t)(index & 0x00FF);

    // 首先，寫入 16 位元暫存器索引以指定從何處讀取
    hal_status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr, reg_addr_buffer, 2, VL53L1_I2C_TIMEOUT_MS);

    if (hal_status != HAL_OK) {
        // printf("VL53L1_ReadMulti: Write Index 0x%04X for Dev 0x%02X - HAL Error: %d\r\n", index, dev_addr, hal_status);
        return VL53L1_ERROR_CONTROL_INTERFACE; // I2C 通信錯誤
    }

    // 然後，從該暫存器讀取指定數量的位元組
    hal_status = HAL_I2C_Master_Receive(&hi2c1, dev_addr, pdata, count, VL53L1_I2C_TIMEOUT_MS);

    if (hal_status == HAL_OK) {
        // printf("VL53L1_ReadMulti: Dev 0x%02X, Index 0x%04X, Count %lu - OK\r\n", dev_addr, index, count);
        return VL53L1_ERROR_NONE; // 成功
    } else {
        // printf("VL53L1_ReadMulti: Read Data from Index 0x%04X for Dev 0x%02X - HAL Error: %d\r\n", index, dev_addr, hal_status);
        return VL53L1_ERROR_CONTROL_INTERFACE; // I2C 通信錯誤
    }
}

/**
  * @brief  向 I2C 設備寫入單一位元組。
  * @param  dev_addr    設備位址
  * @param  index   要寫入的 16 位元暫存器索引。
  * @param  data    要寫入的位元組。
  * @retval 0 表示成功，非零表示失敗。
  */
int8_t VL53L1_WrByte(uint16_t dev_addr, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(dev_addr, index, &data, 1);
}

/**
  * @brief  向 I2C 設備寫入一個 16 位元字。
  * @param  dev_addr    設備位址
  * @param  index   要寫入的 16 位元暫存器索引。
  * @param  data    要寫入的 16 位元字 (高位元組在前)。
  * @retval 0 表示成功，非零表示失敗。
  */
int8_t VL53L1_WrWord(uint16_t dev_addr, uint16_t index, uint16_t data) {
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(data >> 8);   // MSB
    buffer[1] = (uint8_t)(data & 0xFF); // LSB
    return VL53L1_WriteMulti(dev_addr, index, buffer, 2);
}

/**
  * @brief  向 I2C 設備寫入一個 32 位元雙字。
  * @param  dev_addr    設備位址
  * @param  index   要寫入的 16 位元暫存器索引。
  * @param  data    要寫入的 32 位元雙字 (高位元組在前)。
  * @retval 0 表示成功，非零表示失敗。
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
  * @brief  從 I2C 設備讀取單一位元組。
  * @param  dev_addr    設備位址
  * @param  index   要讀取的 16 位元暫存器索引。
  * @param  pdata   指向儲存讀取位元組的指標。
  * @retval 0 表示成功，非零表示失敗。
  */
int8_t VL53L1_RdByte(uint16_t dev_addr, uint16_t index, uint8_t *pdata) {
    return VL53L1_ReadMulti(dev_addr, index, pdata, 1);
}

/**
  * @brief  從 I2C 設備讀取一個 16 位元字。
  * @param  dev_addr    設備位址
  * @param  index   要讀取的 16 位元暫存器索引。
  * @param  pdata   指向儲存讀取 16 位元字的指標 (高位元組在前)。
  * @retval 0 表示成功，非零表示失敗。
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
  * @brief  從 I2C 設備讀取一個 32 位元雙字。
  * @param  dev_addr    設備位址
  * @param  index   要讀取的 16 位元暫存器索引。
  * @param  pdata   指向儲存讀取 32 位元雙字的指標 (高位元組在前)。
  * @retval 0 表示成功，非零表示失敗。
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
  * @brief  平台特定的毫秒級延遲。
  * @param  dev_addr    設備位址 (此函式中未使用)
  * @param  wait_ms   延遲時間 (毫秒)。
  * @retval 0 表示成功。
  */
int8_t VL53L1_WaitMs(uint16_t dev_addr, int32_t wait_ms) {
    (void)dev_addr; // 未使用的參數
    HAL_Delay((uint32_t)wait_ms);
    return VL53L1_ERROR_NONE;
}
