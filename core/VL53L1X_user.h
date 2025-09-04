#ifndef VL53L1X_USER_H
#define VL53L1X_USER_H

#include "VL53L1X_api.h" // 包含 VL53L1X API 的核心定義
#include "stm32l4xx_hal.h" // 包含 STM32 HAL 庫定義 (用於 GPIO_PinState 等)

// VL53L1X I2C 位址 (7位元)
#define VL53L1X_DEFAULT_I2C_ADDR_7BIT   (0x29)
// VL53L1X I2C 位址，用於 HAL 函式 (8位元，左移一位)
#define VL53L1X_I2C_ADDRESS_HAL         (VL53L1X_DEFAULT_I2C_ADDR_7BIT << 1)

// 自訂啟動逾時錯誤碼
#define VL53L1_ERROR_BOOT_TIMEOUT       (-7)
// VL53L1X 預期的型號 ID
#define VL53L1_EXPECTED_MODEL_ID        (0xEEAC)


// 外部宣告設備位址，在 .c 檔案中定義
// 這是 HAL 使用的 8 位元位址
extern uint16_t vl53l1x_dev_addr;


/**
  * @brief 初始化並設定 VL53L1X 感測器。
  * @retval VL53L1X_ERROR 錯誤碼 (VL53L1_ERROR_NONE 表示成功)。
  */
VL53L1X_ERROR VL53L1X_User_Init(void);

/**
  * @brief 啟動 VL53L1X 連續測距模式。
  * @retval VL53L1X_ERROR 錯誤碼。
  */
VL53L1X_ERROR VL53L1X_User_StartRanging(void);

/**
  * @brief 獲取一次測量結果。
  * 此函式會檢查資料是否就緒，如果就緒則讀取距離和狀態，並清除中斷。
  * @param  pDistance_mm 指向儲存距離值 (毫米) 的變數指標。
  * @param  pRangeStatus 指向儲存測距狀態的變數指標。
  * @param  pDataReady 指向儲存資料是否就緒標誌的變數指標 (1 表示就緒，0 表示未就緒)。
  * @retval VL53L1X_ERROR 錯誤碼。
  */
VL53L1X_ERROR VL53L1X_User_GetMeasurement(uint16_t *pDistance_mm, uint8_t *pRangeStatus, uint8_t *pDataReady);

/**
  * @brief 管理 VL53L1X 的 XSHUT 硬體關斷引腳。
  * @param  state: 0 表示關斷 (拉低)，1 表示使能 (拉高)。
  * @retval 無。
  */
void VL53L1X_User_ManageXSHUT(uint8_t state);

#endif // VL53L1X_USER_H
