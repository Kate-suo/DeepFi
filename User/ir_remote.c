#include "ir_remote.h"
// #include <stdio.h> // 移除stdio.h，因為不再使用printf

//==============================================================================
// 紅外解碼狀態機定義
//==============================================================================
#define STATE_ERROR     10
#define STATE_IDLE      0
#define STATE_YINDAO    1       // 引导码阶段 (等待引导码低电平结束)
#define STATE_YINDAO_H  2       // 引导码高电平阶段 (等待引导码高电平结束)
#define STATE_SYSID     3       // 系统码阶段 (接收8位系统码)
#define STATE_SYSIDINV  4       // 系统码反码阶段 (接收8位系统码反码)
#define STATE_DATA      5       // 数据码阶段 (接收8位数据码)
#define STATE_DATAINV   6       // 数据码反码阶段 (接收8位数据码反码)
#define STATE_END       7       // 结束阶段 (通常是一个短脉冲)

//==============================================================================
// 紅外訊號脈衝時間閾值定義 (根據您提供的時序和時鐘計算)
// 定時器時鐘源為 64MHz，Prescaler = 16，則計數頻率 = 64MHz / (16 + 1) = 64MHz / 17 ≈ 3.7647 MHz
// 計數週期 ≈ 1 / 3.7647 MHz ≈ 0.2656us
//==============================================================================
// 引導碼低電平脈衝 (9ms)
#define YINDAO_LOW_MAX      34885 // 33885 + 1000
#define YINDAO_LOW_MIN      32885 // 33885 - 1000

// 引導碼高電平脈衝 (4.5ms)
#define YINDAO_HIGH_MAX     17443 // 16943 + 500
#define YINDAO_HIGH_MIN     16443 // 16943 - 500

// 數據位起始低電平脈衝 (0.56ms)
#define DATA_BIT_START_MAX  2308  // 2108 + 200
#define DATA_BIT_START_MIN  1908  // 2108 - 200

// 數據 0 高電平脈衝 (0.56ms)
#define DATA_BIT_0_HIGH_MAX 2308  // 2108 + 200
#define DATA_BIT_0_HIGH_MIN 1908  // 2108 - 200

// 數據 1 高電平脈衝 (1.96ms)
#define DATA_BIT_1_HIGH_MAX 7879  // 7379 + 500
#define DATA_BIT_1_HIGH_MIN 6879  // 7379 - 500

// 結束脈衝 (0.56ms)
#define END_PULSE_MAX       2308  // 2108 + 200
#define END_PULSE_MIN       1908  // 2108 - 200


//==============================================================================
// 宏定義：紅外接收器引腳和端口 (需要根據實際硬體連接修改)
//==============================================================================
#define IR_GPIO_PORT        GPIOA // 示例：假設紅外接收器連接在GPIOA
#define IR_GPIO_PIN         GPIO_PIN_11 // 根據用戶提供的信息修改為 PA11
#define IR_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE() // 示例：使能GPIOA時鐘
#define EXTI_IR_IRQn        EXTI15_10_IRQn // 對於PA11，使用EXTI15_10_IRQn
#define IR_Prior            5 // 示例：中斷優先級

//==============================================================================
// 偵錯輔助宏定義 (可以用於在中斷中切換LED或GPIO)
//==============================================================================
// 假設你一個LED連接在GPIOA pin 5 (例如，板載LED)
#define DEBUG_LED_PORT      GPIOA
#define DEBUG_LED_PIN       GPIO_PIN_12 // 修改為 PA12
#define DEBUG_LED_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_LED_TOGGLE()  HAL_GPIO_TogglePin(DEBUG_LED_PORT, DEBUG_LED_PIN)


//==============================================================================
// 全局變量
//==============================================================================
static uint8_t radioStatus = STATE_IDLE; // 當前解碼狀態
static uint8_t radioDataSequence = 0;    // 當前數據位序號
static uint8_t radioData = 0;            // 當前解碼的數據碼
static uint8_t radioAddress = 0;         // 儲存接收到的地址碼
static uint8_t radioAddressInv = 0;      // 儲存接收到的地址反碼
static uint8_t radioCommand = 0;         // 儲存接收到的命令碼
static uint8_t radioCommandInv = 0;      // 儲存接收到的命令反碼

static uint16_t levelLong = 0;           // 當前脈衝持續時間
static uint8_t HW_INPUT = 0;             // 當前引腳電平狀態

static TIM_HandleTypeDef htim6; // TIM6 定時器句柄

static volatile uint32_t last_ir_command = IR_COMMAND_NONE; // 儲存最近一次接收到的有效命令
static volatile uint8_t command_received_flag = 0; // 新增標誌，指示是否有新命令接收到

//==============================================================================
// 私有函數聲明
//==============================================================================
static void Timer6_Init(void);
static void IR_PIN_initial(void);
static uint8_t IR_StatusRead(void);
static void read_Infrared(void);
static void state_judgement(void);
static void IdDetection(void); // 現在用於校驗地址碼
static void ProcessReceivedCommand(uint8_t command_code);
static void Debug_LED_Init(void); // 偵錯LED初始化

//==============================================================================
// 函數實現
//==============================================================================

/**
  * @brief 初始化紅外遙控模組（GPIO和定時器）
  * @param None
  * @retval None
  */
void IR_Remote_Init(void)
{
    // 初始化定時器用於測量脈衝寬度
    Timer6_Init();
    // 初始化紅外接收器引腳並配置外部中斷
    IR_PIN_initial();
    // 初始化偵錯LED
    Debug_LED_Init();

    // 初始化狀態機
    radioStatus = STATE_IDLE;
    last_ir_command = IR_COMMAND_NONE;
    command_received_flag = 0;
    // printf("IR_Remote_Init: Module initialized.\r\n"); // 註解掉
}

/**
  * @brief 初始化 TIM6 定時器用於測量脈衝寬度
  * @param None
  * @retval None
  */
static void Timer6_Init(void)
{
    // 使能TIM6時鐘
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    // 配置定時器預分頻器和週期
    // 如果時鐘源為 64MHz，Prescaler = 16，則計數頻率 = 64MHz / (16 + 1) = 64MHz / 17 ≈ 3.7647 MHz
    // 計數週期 ≈ 1 / 3.7647 MHz ≈ 0.2656us
    htim6.Init.Prescaler = 16; // 修改為 16
    htim6.Init.Period = 0xFFFF; // 最大計數週期 (16位定時器)
    htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.RepetitionCounter = 0;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        // printf("Timer6_Init: HAL_TIM_Base_Init failed!\r\r\n"); // 註解掉
        // 初始化錯誤處理
        // Error_Handler(); // 請根據你的錯誤處理機制進行修改
    }

    // 啟動定時器
    HAL_TIM_Base_Start(&htim6);
    // 確保定時器計數器清零
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    // printf("Timer6_Init: TIM6 initialized and started.\r\n"); // 註解掉
}

/**
  * @brief 初始化紅外接收器引腳並配置外部中斷
  * @param None
  * @retval None
  */
static void IR_PIN_initial(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 使能紅外接收器引腳的GPIO時鐘
    IR_GPIO_CLK_ENABLE();

    // 配置紅外接收器引腳為輸入模式，帶上拉，並啟用上升沿和下降沿中斷
    GPIO_InitStruct.Pin = IR_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // 紅外接收器通常需要上拉
    HAL_GPIO_Init(IR_GPIO_PORT, &GPIO_InitStruct);

    // 使能並設置外部中斷優先級
    HAL_NVIC_SetPriority(EXTI_IR_IRQn, IR_Prior, 0);
    HAL_NVIC_EnableIRQ(EXTI_IR_IRQn);
    // printf("IR_PIN_initial: IR GPIO (PA11) and EXTI interrupt configured.\r\n"); // 註解掉
}

/**
  * @brief 初始化偵錯LED引腳
  * @param None
  * @retval None
  */
static void Debug_LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 使能偵錯LED引腳的GPIO時鐘
    DEBUG_LED_CLK_ENABLE();

    // 配置偵錯LED引腳為推挽輸出
    GPIO_InitStruct.Pin = DEBUG_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DEBUG_LED_PORT, &GPIO_InitStruct);
    // printf("Debug_LED_Init: Debug LED (PA12) initialized.\r\n"); // 更新註解
}


/**
  * @brief 讀取紅外接收器引腳當前的電平狀態
  * @param None
  * @retval 1: 高電平, 0: 低電平
  */
static uint8_t IR_StatusRead(void)
{
    return (HAL_GPIO_ReadPin(IR_GPIO_PORT, IR_GPIO_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

/**
  * @brief GPIO 外部中斷回呼函數
  * 當紅外接收器引腳電平變化時由HAL庫呼叫
  * @param GPIO_Pin 觸發中斷的引腳
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 檢查是否是紅外接收器引腳觸發的中斷
    if(GPIO_Pin == IR_GPIO_PIN)
    {
        // 偵錯輔助：在每次中斷觸發時翻轉LED
        DEBUG_LED_TOGGLE();

        // 讀取當前引腳電平
        HW_INPUT = IR_StatusRead();
        // 呼叫紅外解碼處理函數
        read_Infrared();
    }
}

/**
  * @brief 紅外訊號解碼處理函數 (狀態機實現)
  * @param None
  * @retval None
  */
static void read_Infrared(void)
{
    // 獲取定時器當前計數值，表示上一個脈衝的持續時間
    // TIM6 是16位定時器，直接讀取即可
    levelLong = __HAL_TIM_GET_COUNTER(&htim6);

    // 重置定時器計數值為0，準備測量下一個脈衝的持續時間
    __HAL_TIM_SET_COUNTER(&htim6, 0);

    // 根據當前狀態進行解碼
    switch (radioStatus)
    {
        case STATE_ERROR:
            // 發生錯誤後，等待引導碼重新開始
            if (HW_INPUT == 0) // 接收到低電平，可能是引導碼的開始
            {
                radioStatus = STATE_YINDAO;
                // printf("IR_Decode: STATE_ERROR -> STATE_YINDAO (HW_INPUT=0, levelLong=%lu)\r\n", levelLong); // 註解掉
            } else {
                // printf("IR_Decode: STATE_ERROR (HW_INPUT=1, levelLong=%lu) - Waiting for low pulse.\r\n", levelLong); // 註解掉
            }
            break;

        case STATE_IDLE: // 空閒狀態，等待引導碼的低電平
            if (HW_INPUT == 0) // 接收到低電平，進入引導碼階段
            {
                radioStatus = STATE_YINDAO;
                // printf("IR_Decode: STATE_IDLE -> STATE_YINDAO (HW_INPUT=0, levelLong=%lu)\r\n", levelLong); // 註解掉
            } else {
                 // printf("IR_Decode: STATE_IDLE (HW_INPUT=1, levelLong=%lu) - Waiting for low pulse.\r\n", levelLong); // 註解掉
            }
            break;

        case STATE_YINDAO: // 引導碼階段 (等待引導碼低電平結束)
            if (HW_INPUT == 1) // 引導碼的低電平結束，接收到高電平
            {
                // 檢查引導碼低電平的持續時間 (長脈衝)
                if ((levelLong < YINDAO_LOW_MAX) && (levelLong > YINDAO_LOW_MIN))
                {
                    // 引導碼低電平有效，進入引導碼高電平階段
                    radioStatus = STATE_YINDAO_H;
                    // printf("IR_Decode: STATE_YINDAO -> STATE_YINDAO_H (HW_INPUT=1, levelLong=%lu) - Low pulse OK.\r\n", levelLong); // 註解掉
                }
                else
                {
                    // 引導碼低電平持續時間無效，進入錯誤狀態
                    radioStatus = STATE_ERROR;
                    // printf("IR_Decode: STATE_YINDAO -> STATE_ERROR (HW_INPUT=1, levelLong=%lu) - Low pulse timing error!\r\n", levelLong); // 註解掉
                }
            }
            else // 意外情況：在等待低電平結束時又接收到低電平
            {
                 radioStatus = STATE_ERROR; // 視為錯誤
                 // printf("IR_Decode: STATE_YINDAO -> STATE_ERROR (HW_INPUT=0, levelLong=%lu) - Unexpected low pulse.\r\n", levelLong); // 註解掉
            }
            break;

        case STATE_YINDAO_H: // 引導碼高電平階段 (等待引導碼高電平結束)
             if (HW_INPUT == 0) // 引導碼的高電平結束，接收到低電平
             {
                 // 檢查引導碼高電平的持續時間 (短脈衝)
                 if ((levelLong < YINDAO_HIGH_MAX) && (levelLong > YINDAO_HIGH_MIN))
                 {
                     // 引導碼高電平有效，進入系統碼階段
                     radioStatus = STATE_SYSID;
                     radioDataSequence = 0; // 重置數據位序號
                     radioData = 0;         // 清空數據
                     // printf("IR_Decode: STATE_YINDAO_H -> STATE_SYSID (HW_INPUT=0, levelLong=%lu) - High pulse OK. Starting sys ID.\r\n", levelLong); // 註解掉
                 }
                 else
                 {
                     // 引導碼高電平持續時間無效，進入錯誤狀態
                     radioStatus = STATE_ERROR;
                     // printf("IR_Decode: STATE_YINDAO_H -> STATE_ERROR (HW_INPUT=0, levelLong=%lu) - High pulse timing error!\r\n", levelLong); // 註解掉
                 }
             }
             else // 意外情況：在等待高電平結束時又接收到高電平
             {
                  radioStatus = STATE_ERROR; // 視為錯誤
                  // printf("IR_Decode: STATE_YINDAO_H -> STATE_ERROR (HW_INPUT=1, levelLong=%lu) - Unexpected high pulse.\r\n", levelLong); // 註解掉
             }
             break;


        case STATE_SYSID: // 系統碼階段 (接收8位系統碼)
            state_judgement(); // 判斷當前位是0還是1，並更新radioData
            if (radioStatus == STATE_SYSIDINV) // 如果 state_judgement 成功接收了8位系統碼，會跳轉到 STATE_SYSIDINV
            {
                 radioAddress = radioData; // 儲存地址碼
                 // printf("IR_Decode: STATE_SYSID -> STATE_SYSIDINV (radioAddress=0x%02X). Starting sys ID inverse check.\r\n", radioAddress); // 註解掉
                 // IdDetection(); // 校驗系統碼 (現在在 SYSIDINV 之後統一校驗)
            } else if (radioStatus == STATE_ERROR) {
                // printf("IR_Decode: STATE_SYSID -> STATE_ERROR during judgement.\r\n"); // 註解掉
            }
            break;

        case STATE_SYSIDINV: // 系統碼反碼階段 (接收8位系統碼反碼)
            state_judgement(); // 判斷當前位是0還是1，並更新radioData
            if (radioStatus == STATE_DATA) // 如果 state_judgement 成功接收了8位反碼，會跳轉到 STATE_DATA
            {
                radioAddressInv = radioData; // 儲存地址反碼
                // printf("IR_Decode: STATE_SYSIDINV -> STATE_DATA (radioAddressInv=0x%02X). Starting data bits.\r\n", radioAddressInv); // 註解掉
                // 執行地址碼和反碼的校驗
                IdDetection(); // 呼叫 IdDetection 進行地址碼校驗
                if (radioStatus == STATE_ERROR) { // 如果 IdDetection 發現錯誤
                    // printf("IR_Decode: Address verification failed! Resetting.\r\n"); // 註解掉
                    radioStatus = STATE_IDLE; // 地址碼校驗失敗，回到空閒狀態
                }
            } else if (radioStatus == STATE_ERROR) {
                // printf("IR_Decode: STATE_SYSIDINV -> STATE_ERROR during judgement.\r\n"); // 註解掉
            }
            break;

        case STATE_DATA: // 數據碼階段 (接收8位數據碼)
            state_judgement(); // 判斷當前位是0還是1，並更新radioData
            if (radioStatus == STATE_DATAINV) // 如果 state_judgement 成功接收了8位數據碼，會跳轉到 STATE_DATAINV
            {
                 radioCommand = radioData; // 儲存命令碼
                 // printf("IR_Decode: STATE_DATA -> STATE_DATAINV (radioCommand=0x%02X). Starting data inverse check.\r\n", radioCommand); // 註解掉
            } else if (radioStatus == STATE_ERROR) {
                // printf("IR_Decode: STATE_DATA -> STATE_ERROR during judgement.\r\n"); // 註解掉
            }
            break;

        case STATE_DATAINV: // 數據碼反碼階段 (接收8位數據碼反碼)
            state_judgement(); // 判斷當前位是0還是1，並更新radioData
            if (radioStatus == STATE_END) // 如果 state_judgement 成功接收了8位反碼，會跳轉到 STATE_END
            {
                radioCommandInv = radioData; // 儲存命令反碼
                // printf("IR_Decode: STATE_DATAINV -> STATE_END (radioCommandInv=0x%02X). Command received!\r\n", radioCommandInv); // 註解掉
                // 數據碼反碼接收完成，整個命令接收結束
                // 最終校驗命令碼和反碼
                if ((radioCommand ^ radioCommandInv) == 0xFF) { // 檢查命令碼與反碼是否互補
                    ProcessReceivedCommand(radioCommand); // 在這裡處理接收到的命令
                } else {
                    // printf("IR_Decode: Command verification failed! Command=0x%02X, Inverse=0x%02X. Resetting.\r\n", radioCommand, radioCommandInv); // 註解掉
                    radioStatus = STATE_ERROR; // 命令碼校驗失敗，進入錯誤狀態
                }
                radioStatus = STATE_IDLE; // 返回空閒狀態，等待下一個命令
            } else if (radioStatus == STATE_ERROR) {
                // printf("IR_Decode: STATE_DATAINV -> STATE_ERROR during judgement.\r\r\n"); // 註解掉
            }
            break;

        case STATE_END: // 結束階段 (通常是一個短脈衝) - 在 DATAINV 狀態處理跳轉到 END
            radioStatus = STATE_IDLE; // 結束脈衝接收完成，返回空閒狀態
            // printf("IR_Decode: STATE_END -> STATE_IDLE. Ready for next command.\r\n"); // 註解掉
            break;

        default: // 未知狀態，返回空閒狀態
            radioStatus = STATE_IDLE;
            // printf("IR_Decode: Unknown state (%d) -> STATE_IDLE.\r\n", radioStatus); // 註解掉
            break;
    }
}

/**
  * @brief 在接收系統碼或數據碼階段，判斷當前脈衝代表的數據位 (0或1)
  * @param None
  * @retval None
  */
static void state_judgement(void)
{
    // 這個函數在接收系統碼或數據碼的低電平脈衝結束時被呼叫 (HW_INPUT == 0)
    // 或者在接收系統碼或數據碼的高電平脈衝結束時被呼叫 (HW_INPUT == 1)

    if (HW_INPUT == 1) // 如果當前引腳是高電平，表示上一個脈衝是低電平 (數據位的開始脈衝)
    {
        // 檢查低電平脈衝的持續時間 (所有數據位都以一個短的低電平脈衝開始)
        if ((levelLong > DATA_BIT_START_MAX) || (levelLong < DATA_BIT_START_MIN))
        {
            // 低電平脈衝持續時間無效，進入錯誤狀態
            radioStatus = STATE_ERROR;
            // printf("State_Judgement: HW_INPUT=1, levelLong=%lu. Start pulse timing error!\r\n", levelLong); // 註解掉
        }
        // 如果低電平脈衝有效，狀態保持不變，等待高電平脈衝結束
    }
    else // 如果當前引腳是低電平，表示上一個脈衝是高電平 (數據位的高電平部分)
    {
        // 檢查高電平脈衝的持續時間，判斷是數據位0還是1
        if ((levelLong < DATA_BIT_0_HIGH_MAX) || (levelLong > DATA_BIT_0_HIGH_MIN)) // 修改為 ||
        {
            // 高電平持續時間短，表示接收到數據位 0
            radioData = radioData << 1; // 數據左移一位，新位為0
            // printf("State_Judgement: HW_INPUT=0, levelLong=%lu. Bit 0 received. radioData=0x%02X, seq=%d\r\n", levelLong, radioData, radioDataSequence + 1); // 註解掉
        }
        else if ((levelLong < DATA_BIT_1_HIGH_MAX) || (levelLong > DATA_BIT_1_HIGH_MIN)) // 修改為 ||
        {
            // 高電平持續時間長，表示接收到數據位 1
            radioData = (radioData << 1) + 1; // 數據左移一位，新位為1
            // printf("State_Judgement: HW_INPUT=0, levelLong=%lu. Bit 1 received. radioData=0x%02X, seq=%d\r\n", levelLong, radioData, radioDataSequence + 1); // 註解掉
        }
        else
        {
            // 高電平脈衝持續時間無效，進入錯誤狀態
            radioStatus = STATE_ERROR;
            // printf("State_Judgement: HW_INPUT=0, levelLong=%lu. High pulse timing error (bit 0/1)! radioData=0x%02X, seq=%d\r\n", levelLong, radioData, radioDataSequence + 1); // 註解掉
        }

        // 數據位序號加1
        radioDataSequence++;

        // 如果接收完8位數據，則進入下一個狀態
        if (radioDataSequence >= 8)
        {
            radioStatus++; // 跳轉到下一個狀態 (例如從 SYSID 到 SYSIDINV，從 DATA 到 DATAINV)
            radioDataSequence = 0; // 重置數據位序號
            // printf("State_Judgement: 8 bits received. New status: %d\r\n", radioStatus); // 註解掉
            // 注意：這裡 radioData 儲存的是接收到的8位數據
        }
    }
}

/**
  * @brief 校驗接收到的系統碼 (地址碼及其反碼)
  * @param None
  * @retval None
  */
static void IdDetection(void)
{
    // 根據您提供的遙控器用戶碼 "00FF"
    // NEC 協議通常是 8 位地址碼 + 8 位地址反碼
    // 如果用戶碼是 "00FF"，則地址碼為 0x00，地址反碼為 0xFF
    // 檢查地址碼是否為 0x00 且地址反碼是否為 0xFF
    if (radioAddress == 0x00 && radioAddressInv == 0xFF)
    {
        // printf("IdDetection: Address 0x%02X (Inverse 0x%02X) OK.\r\n", radioAddress, radioAddressInv); // 註解掉
        // 地址碼校驗通過，狀態機繼續
    }
    else
    {
        // 地址碼不匹配，進入錯誤狀態
        radioStatus = STATE_ERROR;
        // printf("IdDetection: Address 0x%02X (Inverse 0x%02X) Mismatch! Expected 0x00/0xFF. -> STATE_ERROR.\r\n", radioAddress, radioAddressInv); // 註解掉
    }
}

/**
  * @brief 處理接收到的紅外命令碼，並儲存起來供主迴圈讀取
  * @param command_code 接收到的數據碼
  * @retval None
  */
static void ProcessReceivedCommand(uint8_t command_code)
{
    // 將接收到的命令碼儲存到全局變量中，並設置標誌
    last_ir_command = command_code;
    command_received_flag = 1; // 設置標誌
    // printf("ProcessReceivedCommand: Command 0x%02X stored. Flag set.\r\n", command_code); // 註解掉
}

/**
  * @brief 讀取最近一次接收到的紅外遙控命令
  * @param None
  * @retval 紅外命令碼，如果沒有新命令則返回IR_COMMAND_NONE
  */
uint32_t IR_Remote_ReadCommand(void)
{
    uint32_t command = IR_COMMAND_NONE;

    // 檢查是否有新命令接收到
    if (command_received_flag)
    {
        // 讀取儲存的命令
        command = last_ir_command;
        // 清除標誌和儲存的命令，表示已讀
        command_received_flag = 0;
        last_ir_command = IR_COMMAND_NONE;
        // printf("IR_Remote_ReadCommand: Retrieved command 0x%08lX. Flag cleared.\r\n", command); // 註解掉
    }
    return command;
}
