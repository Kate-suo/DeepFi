#include "VL53L1X_user.h"
#include "vl53l1_platform.h" // 包含 VL53L1_WaitMs, VL53L1_RdByte 等平台函式
#include <stdio.h>           // 用於 printf
#include "i2c.h"             // 包含 extern I2C_HandleTypeDef hi2c1; 的宣告

// I2C Handle (在 i2c.c 中由 CubeMX 定義)
// extern I2C_HandleTypeDef hi2c1; // 已移至 i2c.h 中，此處透過 i2c.h 包含

// 定義設備位址
uint16_t vl53l1x_dev_addr = VL53L1X_I2C_ADDRESS_HAL;
VL53L1X_ERROR VL53L1_ERROR_CONTROL_INTERFACE;
VL53L1X_ERROR VL53L1_ERROR_NONE;
// XSHUT 引腳定義 (特定於使用者的硬體)
#define XSHUT_PIN			GPIO_PIN_5
#define XSHUT_PORT		    GPIOB

/**
  * @brief 管理 VL53L1X 的 XSHUT 硬體關斷引腳。
  * @param  state: 0 表示關斷 (拉低)，1 表示使能 (拉高)。
  */
void VL53L1X_User_ManageXSHUT(uint8_t state) {
    // XSHUT 連接埠的 GPIO 時脈應在 MX_GPIO_Init() 中啟用
    if (state) {
        HAL_GPIO_WritePin(XSHUT_PORT, XSHUT_PIN, GPIO_PIN_SET); // XSHUT ON
    } else {
        HAL_GPIO_WritePin(XSHUT_PORT, XSHUT_PIN, GPIO_PIN_RESET); // XSHUT OFF
    }
}

/**
  * @brief 初始化並設定 VL53L1X 感測器。
  * @retval VL53L1X_ERROR 狀態
  */
VL53L1X_ERROR VL53L1X_User_Init(void) {
    VL53L1X_ERROR status = VL53L1_ERROR_NONE;
    uint8_t booted_flag_val = 0;
    int boot_attempts = 0;
    uint16_t sensor_id = 0;
    uint8_t firmware_system_status_val = 0;

    VL53L1X_User_ManageXSHUT(1); // 拉高 XSHUT 引腳以使能感測器
    printf("XSHUT pin set HIGH (PB5).\r\n");

    printf("Waiting for sensor power up (100ms after XSHUT HIGH)...\r\n");
    VL53L1_WaitMs(vl53l1x_dev_addr, 100); // 感測器上電穩定延遲

//    printf("Performing basic I2C check for device at 0x%02X (7-bit: 0x%02X)...\r\n", vl53l1x_dev_addr, VL53L1X_DEFAULT_I2C_ADDR_7BIT);
//    HAL_StatusTypeDef i2c_ready_status = HAL_I2C_IsDeviceReady(&hi2c1, vl53l1x_dev_addr, 3, 100); // 3 次嘗試，100ms 逾時
//    if (i2c_ready_status != HAL_OK) {
//        printf("HAL_I2C_IsDeviceReady FAILED! HAL Status: %d. Sensor not responding.\r\n", i2c_ready_status);
//        printf("CRITICAL: Check I2C wiring, PULL-UP RESISTORS (e.g. 4.7kOhm or 10kOhm on SDA & SCL to VDD).\r\n");
//        printf("CRITICAL: Ensure I2C_Timing param in MX_I2C1_Init (i2c.c) is CORRECT for your PCLK1 (currently %lu Hz) and chosen I2C speed (e.g. 100kHz).\r\n", HAL_RCC_GetPCLK1Freq());
//        printf("          The Timing value 0x10B17DB5 (if still used) is HIGHLY SUSPECT for 100kHz @ 64MHz PCLK1.\r\n");
//        return VL53L1_ERROR_CONTROL_INTERFACE; // 返回 I2C 控制介面錯誤
//    }
//    printf("HAL_I2C_IsDeviceReady OK. Device ACKed on the bus.\r\n");

//    printf("Attempting to read sensor Model ID (expected 0xEEAC)...\r\n");
		
		status = VL53L1_RdWord(vl53l1x_dev_addr, VL53L1_IDENTIFICATION__MODEL_ID, &sensor_id);
		while(status!=VL53L1_ERROR_NONE)
			{
			status = VL53L1_RdWord(vl53l1x_dev_addr, VL53L1_IDENTIFICATION__MODEL_ID, &sensor_id);
			}
    if (status != VL53L1_ERROR_NONE) {
        printf("Failed to read sensor Model ID! VL53L1_RdWord API status: %d\r\n", status);
        printf("This indicates a problem with I2C communication (check Timing, pull-ups, platform functions).\r\n");
        return status; // 傳播錯誤
    } else {
        printf("Sensor Model ID read: 0x%04X\r\n", sensor_id);
        if (sensor_id != VL53L1_EXPECTED_MODEL_ID) {
            printf("WARNING: Sensor Model ID (0x%04X) does not match expected (0x%04X)!\r\n", sensor_id, VL53L1_EXPECTED_MODEL_ID);
        }
    }

    printf("Waiting for VL53L1X boot sequence (checking FIRMWARE__SYSTEM_STATUS at 0x00E5)...\r\n");
    while(boot_attempts < 100) {
        status = VL53L1_RdByte(vl53l1x_dev_addr, VL53L1_FIRMWARE__SYSTEM_STATUS, &firmware_system_status_val);
        if (status != VL53L1_ERROR_NONE) {
            printf("VL53L1_RdByte(FIRMWARE__SYSTEM_STATUS) API communication error! Code: %d, Attempt: %d\r\n", status, boot_attempts + 1);
            return status; // 關鍵 I2C 失敗，停止嘗試啟動
        }
        
        booted_flag_val = firmware_system_status_val & 0x01; // 位元 0 表示啟動完成
        printf("Attempt %d: FIRMWARE__SYSTEM_STATUS raw value: 0x%02X, booted_flag (bit0): %d\r\n", boot_attempts + 1, firmware_system_status_val, booted_flag_val);

        if (booted_flag_val) {
            break; // 成功啟動
        }
        VL53L1_WaitMs(vl53l1x_dev_addr, 20); // 每次嘗試後等待 20ms
        boot_attempts++;
    }

    if (!booted_flag_val) {
        printf("VL53L1X boot timeout! (Attempts: %d, Last RdByte API status for FIRMWARE_SYS_STATUS: %d, last raw value: 0x%02X)\r\n", boot_attempts, status, firmware_system_status_val);
        printf("INFO: If RdByte API status is 0, I2C comm for that read was 'OK' by HAL, but sensor didn't signal 'booted'.\r\n");
        printf("      MOST LIKELY CAUSE: Incorrect I2C_Timing parameter in MX_I2C1_Init (i2c.c) for PCLK1 (%lu Hz).\r\n", HAL_RCC_GetPCLK1Freq());
        printf("      Also check I2C pull-up resistors and all hardware connections.\r\n");
        return VL53L1_ERROR_BOOT_TIMEOUT; // 返回自訂的啟動逾時錯誤
    }
    printf("VL53L1X booted successfully (FIRMWARE__SYSTEM_STATUS bit 0 is 1).\r\n");

    printf("Initializing VL53L1X sensor ...\r\n");
    status = VL53L1X_SensorInit(vl53l1x_dev_addr); // 使用全域的 vl53l1x_dev_addr
    if (status != VL53L1_ERROR_NONE) {
        printf("VL53L1X_SensorInit() failed! Code: %d\r\n", status);
        return status;
    }
    printf("VL53L1X sensor initialized successfully.\r\n");

    printf("Setting distance mode (mode 2 long)...\r\n");
    status = VL53L1X_SetDistanceMode(vl53l1x_dev_addr, 2); // 1=short, 2=long
    if (status != VL53L1_ERROR_NONE) {
        printf("VL53L1X_SetDistanceMode() failed! Code: %d\r\n", status);
        return status;
    }

    printf("Setting timing budget to 50ms...\r\n");
    status = VL53L1X_SetTimingBudgetInMs(vl53l1x_dev_addr, 50);
    if (status != VL53L1_ERROR_NONE) {
        printf("VL53L1X_SetTimingBudgetInMs() failed! Code: %d\r\n", status);
        return status;
    }

    printf("Setting inter-measurement period to 60ms...\r\n");
    status = VL53L1X_SetInterMeasurementInMs(vl53l1x_dev_addr, 60);
    if (status != VL53L1_ERROR_NONE) {
        printf("VL53L1X_SetInterMeasurementInMs() failed! Code: %d\r\n", status);
        return status;
    }

    printf("VL53L1X configuration complete.\r\n");
    return VL53L1_ERROR_NONE; // 所有步驟成功
}

/**
  * @brief 啟動 VL53L1X 連續測距模式。
  * @retval VL53L1X_ERROR 錯誤碼。
  */
VL53L1X_ERROR VL53L1X_User_StartRanging(void) {
    VL53L1X_ERROR status = VL53L1X_StartRanging(vl53l1x_dev_addr);
    if (status != VL53L1_ERROR_NONE) {
        printf("VL53L1X_User_StartRanging: VL53L1X_StartRanging() API failed! Code: %d\r\n", status);
    }
    return status;
}

/**
  * @brief 獲取一次測量結果。
  * @param  pDistance_mm 指向儲存距離值 (毫米) 的變數指標。
  * @param  pRangeStatus 指向儲存測距狀態的變數指標。
  * @param  pDataReady 指向儲存資料是否就緒標誌的變數指標 (1 表示就緒，0 表示未就緒)。
  * @retval VL53L1X_ERROR 錯誤碼。
  */

VL53L1X_ERROR VL53L1X_User_GetMeasurement(uint16_t *pDistance_mm, uint8_t *pRangeStatus, uint8_t *pDataReady) {
    VL53L1X_ERROR status = VL53L1_ERROR_NONE;
    *pDataReady = 0; // 預設為未就緒

    status = VL53L1X_CheckForDataReady(vl53l1x_dev_addr, pDataReady);
    if (status != VL53L1_ERROR_NONE) {
        // printf("VL53L1X_User_GetMeasurement: CheckForDataReady failed! Code: %d\r\n", status);
        return status;
    }

    if (*pDataReady) {
        status = VL53L1X_GetDistance(vl53l1x_dev_addr, pDistance_mm);
        if (status != VL53L1_ERROR_NONE) {
            // printf("VL53L1X_User_GetMeasurement: GetDistance failed! Code: %d\r\n", status);
            // 即使獲取距離失敗，仍嘗試獲取狀態並清除中斷
        }

        VL53L1X_ERROR status_range = VL53L1X_GetRangeStatus(vl53l1x_dev_addr, pRangeStatus);
        if (status_range != VL53L1_ERROR_NONE) {
            // printf("VL53L1X_User_GetMeasurement: GetRangeStatus failed! Code: %d\r\n", status_range);
            if (status == VL53L1_ERROR_NONE) status = status_range; // 如果 GetDistance 成功，則更新為 GetRangeStatus 的錯誤
        }

        VL53L1X_ERROR status_clear = VL53L1X_ClearInterrupt(vl53l1x_dev_addr);
        if (status_clear != VL53L1_ERROR_NONE) {
            // printf("VL53L1X_User_GetMeasurement: ClearInterrupt failed! Code: %d\r\n", status_clear);
            if (status == VL53L1_ERROR_NONE) status = status_clear; // 如果之前都成功，則更新為 ClearInterrupt 的錯誤
        }
    }
    return status;
}
