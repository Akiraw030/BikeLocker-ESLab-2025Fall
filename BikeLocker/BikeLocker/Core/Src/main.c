/* USER CODE BEGIN Header */
/**
  ****************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ****************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided    ['AS-IS.
  *
  ****************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "app_bluenrg_ms.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32l475e_iot01_accelero.h"
#include <stdlib.h>
#include "b_l475e_iot01a1.h"
#include <time.h>
#include <math.h>
#include <string.h>
#include "bluenrg_gap.h"
#include "bluenrg_aci.h"
#include "hci.h"
/* 宣告外部變數 (CubeMX 生成的 I2C handle) */
extern I2C_HandleTypeDef hi2c2;

// LSM6DSL 的 I2C 位址 (寫入)
#define LSM6DSL_ADDR  0xD4
// 定義閾值 (根據上面的建議)
#define THRESHOLD_LOW   0.05f
#define THRESHOLD_MED   0.25f
#define THRESHOLD_HIGH  0.60f
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
osThreadId SystemTaskHandle;
uint32_t SystemTaskBuffer[ 256 ];
osStaticThreadDef_t SystemTaskControlBlock;
osThreadId SensorTaskHandle;
uint32_t SensorTaskBuffer[ 512 ];
osStaticThreadDef_t SensorTaskControlBlock;
osThreadId CommTaskHandle;
uint32_t CommTaskBuffer[ 256 ];
osStaticThreadDef_t CommTaskControlBlock;
/* USER CODE BEGIN PV */
// 1. 系統狀態定義
typedef enum {
  STATE_LOCKED,
  STATE_UNLOCKED
} SystemState_t;

typedef enum {
    MODE_IDLE,
    MODE_LOW,
    MODE_MED,
    MODE_HIGH
} BikeMode;

SystemState_t currentState = STATE_LOCKED; // 預設上鎖

// 2. 按鈕節奏偵測變數 (短-短-長)
uint32_t btn_press_start_time = 0; // 按下的時間點
uint8_t  btn_pattern_stage = 0;    // 目前進度: 0=初始, 1=短1完成, 2=短2完成
uint32_t last_btn_activity = 0;    // 上次按鈕操作時間 (用來重置過久的輸入)

// --- 變數宣告 ---
volatile float total_kcal = 0.0f;
volatile const float user_weight = 70.0f;

int16_t pDataXYZ[3] = {0};

float current_mets = 1.0f;

//GPIO_PinState last_btn_state = BTN_PRESSED_STATE;
GPIO_PinState current_btn_state;

//uint32_t btn_press_start_time = 0;
//uint32_t last_btn_activity = 0;
//uint8_t  btn_pattern_stage = 0;

static uint32_t last_motion_tick = 0;

// [新增] 異常紀錄緩衝區
uint32_t motion_history[MAX_HISTORY_LEN]; // 存放時間戳 (秒)
uint8_t  history_count = 0;               // 目前存了幾筆
uint8_t  history_head = 0;                // 環形寫入位置 (如果想做循環覆蓋的話)

// 用來防止震動一次就寫入 10 次的 Debounce 計時器
uint32_t last_motion_save_tick = 0;

// 熱量積分時間點
static uint32_t last_kcal_calc_tick = 0;

static uint32_t last_print = 0;
char lcd_buffer[64];

// 定義時間閾值 (毫秒)
#define BTN_PRESSED_STATE  GPIO_PIN_RESET  // 假設 PC13 按下是 Low
#define BTN_RELEASED_STATE GPIO_PIN_SET
#define PRESS_SHORT_MAX   500   // 短按上限 0.5秒
#define PRESS_LONG_MIN    1000  // 長按下限 1.0秒
#define PATTERN_TIMEOUT   2000  // 按鍵間隔超過 2秒 則重置進度

#define MOTION_THRESHOLD  3000  // 震動靈敏度 (數值越大越不靈敏)
#define MOTION_COOLDOWN   2000  // 冷卻時間 2秒 (避免一秒鐘紀錄10次)
#define MAX_LOG_SIZE      10    // 最多紀錄幾筆異常 (超過覆蓋舊的)

#define AUTO_LOCK_TIMEOUT 60000

typedef struct {
    uint32_t event_tick; // 發生時間 (HAL_GetTick)
    //int16_t  max_g;     // 當下測得的最大震動值 (參考用)
} SecurityLog;

GPIO_PinState last_btn_state = BTN_PRESSED_STATE;

SecurityLog alertLogs[MAX_LOG_SIZE];
uint8_t logHead = 0;   // 寫入位置
uint8_t logCount = 0;  // 目前存了幾筆
uint32_t last_motion_time = 0; // 上一次觸發的時間

// 儲存上一刻的加速度值 (用來比對變化)
int16_t last_xyz[3] = {0};
int16_t curr_xyz[3] = {0};

volatile uint8_t g_MotionDetected = 0;

#define LCD_ADDR 0x4E

extern I2C_HandleTypeDef hi2c1; // 確保可以使用 i2c1

// --- 3144E 測速變數 ---
#define WHEEL_CIRCUMFERENCE 2.0f  // 輪子周長 (單位: 公尺)，請依實際調整
#define HALL_DMA_SIZE 2           // 緩衝區大小 (收2次訊號算一次速度)

uint32_t hall_buff[HALL_DMA_SIZE]; // 存放 DMA 搬運來的時間點
volatile float bike_speed_kmh = 0.0f; // 算出來的時速 (全域變數)
volatile uint32_t last_hall_tick = 0; // 用來判斷車子是不是停了

extern volatile uint8_t request_buzzer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void StartSystemTask(void const * argument);
void StartSensorTask(void const * argument);
void StartCommTask(void const * argument);
void LCD_Clear(void);

/* USER CODE BEGIN PFP */
void Enable_LSM6DSL_WakeUp(uint8_t threshold);
BikeMode Determine_Intensity(float ax, float ay, float az);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- 內部底層函式 ---
void LCD_Send_Nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = nibble & 0xF0; // 取高 4 位
    data |= 0x08;                 // 保持背光開啟 (Bit 3)
    if (rs) data |= 0x01;         // 設定 RS 位

    uint8_t data_block[2];

    // 1. 準備資料 + EN=1 (Enable Pulse High)
    data_block[0] = data | 0x04;
    // 2. 準備資料 + EN=0 (Enable Pulse Low) -> 資料被寫入
    data_block[1] = data & ~0x04;

    // 發送 I2C
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_block, 2, 100);
    HAL_Delay(1); // 等待寫入完成
}

// 發送一個完整的 Byte (指令或資料)
// 會拆成兩次 Nibble 發送
void LCD_Send_Byte(uint8_t val, uint8_t rs) {
    LCD_Send_Nibble(val & 0xF0, rs);        // 先送高 4 位
    LCD_Send_Nibble((val << 4) & 0xF0, rs); // 再送低 4 位
}

// 寫指令
void LCD_Write_Cmd(uint8_t cmd) {
    LCD_Send_Byte(cmd, 0);
}

// 寫資料 (顯示字元)
void LCD_Write_Data(uint8_t data) {
    LCD_Send_Byte(data, 1);
}

// 顯示字串
void LCD_String(char *str) {
    int count = 0;
    // 只要字串沒結束，且寫入少於 20 個字，就繼續寫
    while (*str && count < 20) {
        LCD_Write_Data(*str++);
        count++;

    }
}
// 設定游標位置
void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row >= 4) row = 0;
    LCD_Write_Cmd(0x80 | (col + row_offsets[row]));
}

// --- 初始化 (最重要的一步) ---
void LCD_Init(void) {
    HAL_Delay(50); // 上電後等待電壓穩定

    // 1. 特殊啟動序列 (Magic Sequence)
    // 必須連續送三次 0x03 (只送 Nibble，不能送 Byte!)
    LCD_Send_Nibble(0x30, 0); HAL_Delay(5);
    LCD_Send_Nibble(0x30, 0); HAL_Delay(1);
    LCD_Send_Nibble(0x30, 0); HAL_Delay(1);

    // 2. 切換到 4-bit 模式
    LCD_Send_Nibble(0x20, 0); HAL_Delay(1);

    // 從這裡開始，螢幕才真正進入 4-bit 模式，可以用完整的 Byte 指令了

    // 3. 設定參數
    LCD_Write_Cmd(0x28); // 4-bit, 2 lines, 5x7 font
    HAL_Delay(1);
    LCD_Write_Cmd(0x08); // 關閉顯示
    HAL_Delay(1);
    LCD_Write_Cmd(0x01); // 清除螢幕 (Clear)
    HAL_Delay(2);        // Clear 指令需要比較久
    LCD_Write_Cmd(0x06); // 輸入模式 (文字向右)
    HAL_Delay(1);
    LCD_Write_Cmd(0x0C); // 開啟顯示, 關閉游標
}
// I2C 強制重置序列
// 手動控制 GPIO 模擬 9 個 Clock 脈衝，解開被 LCD 拉住的 SDA 線
void I2C_ForceReset(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. 開啟 GPIO 時鐘 (根據你的腳位修改，這裡是 GPIOB)
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // 2. 設定 SCL (PB6) 和 SDA (PB7) 為 Output Open-Drain
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. 設定 SDA 為 High
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    // 4. 產生 9 個 Clock 脈衝 (Toggle SCL)
    for(int i=0; i<9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // SCL Low
        HAL_Delay(1);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // SCL High
        HAL_Delay(1);
    }

    // 5. 產生 STOP condition
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // SDA Low
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // SCL High
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   // SDA High
    HAL_Delay(1);

    // 之後 MX_I2C1_Init() 會接手把腳位設回 I2C 模式
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  I2C_ForceReset(); // 加在這裡
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_BlueNRG_MS_Init();

  /* USER CODE BEGIN 2 */
  LCD_Init();
  LCD_SetCursor(0, 0);
  LCD_String("BikeLocker Ready!");
  LCD_SetCursor(1, 0); // 第2行 第1格
  LCD_String("Status: LOCKED");
  HAL_Delay(500); // 等一下，讓畫面穩定
  BSP_ACCELERO_Init();
  // 啟動 Timer 2 Channel 3 的 DMA 模式
  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, hall_buff, 1);
  Enable_LSM6DSL_WakeUp(2);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of SystemTask */
  osThreadStaticDef(SystemTask, StartSystemTask, osPriorityHigh, 0, 256, SystemTaskBuffer, &SystemTaskControlBlock);
  SystemTaskHandle = osThreadCreate(osThread(SystemTask), NULL);

  /* definition and creation of SensorTask */
  osThreadStaticDef(SensorTask, StartSensorTask, osPriorityNormal, 0, 512, SensorTaskBuffer, &SensorTaskControlBlock);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* definition and creation of CommTask */
  osThreadStaticDef(CommTask, StartCommTask, osPriorityIdle, 0, 256, CommTaskBuffer, &CommTaskControlBlock);
  CommTaskHandle = osThreadCreate(osThread(CommTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|LED_RED_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin
                          |LED2_Pin|SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VL53L0X_XSHUT_GPIO_Port, VL53L0X_XSHUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin LED_RED_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin
                           ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|LED_RED_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin
                          |ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(ARD_D0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VL53L0X_XSHUT_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VL53L0X_XSHUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Enable_LSM6DSL_WakeUp(uint8_t threshold)
{
    uint8_t regData = 0x02;

    // 1. 設定 TAP_CFG (0x58) 暫存器 -> 開啟中斷功能
    // Bit 7 (INTERRUPTS_ENABLE) = 1
    // Bit 5 (SLOPE_FDS) = 1 (套用高通濾波器，偵測變化而非角度)
    regData = 0x90; // Binary: 1001 0000
    HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, 0x58, I2C_MEMADD_SIZE_8BIT, &regData, 1, 100);

    // 2. 設定 WAKE_UP_THS (0x5B) -> 設定閾值
    // 只取低 6 bit (0~63)
    regData = threshold & 0x3F;
    HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, 0x5B, I2C_MEMADD_SIZE_8BIT, &regData, 1, 100);

    // 3. 設定 WAKE_UP_DUR (0x5C) -> 設定持續時間
    // 設為 0 代表「只要有一瞬間震動」就觸發
    regData = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, 0x5C, I2C_MEMADD_SIZE_8BIT, &regData, 1, 100);

    // 4. 設定 MD1_CFG (0x5E) -> 把 Wake-up 訊號導向 INT1 腳位
    // Bit 5 (INT1_WU) = 1
    regData = 0x20; // Binary: 0010 0000
    HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, 0x5E, I2C_MEMADD_SIZE_8BIT, &regData, 1, 100);

    //printf("LSM6DSL Wake-up Mode Enabled!\r\n");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 確認是 PD11 (LSM6DSL INT1) 觸發的
    if (GPIO_Pin == GPIO_PIN_11) {
        g_MotionDetected = 1; // 舉起旗子，告訴 Task 有狀況
    }
}

// DMA 傳輸完成回呼函式 (buffer 填滿時觸發)

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        // 1. 取得最新的時間 (因為 DMA Size=1，永遠在 buff[0])
        uint32_t current_capture = hall_buff[0];

        // 2. 定義靜態變數，用來記住「上一次」的時間 (離開函式後數值會保留)
        static uint32_t last_capture = 0;

        // 3. 計算時間差
        // 利用 unsigned int 的特性，直接相減就能自動處理 Timer 溢位 (Overflow)
        // 例如：現在 10 - 上次 4294967290 = 16 (正確差值)
        uint32_t diff = current_capture - last_capture;

        // 4. 更新「上一次」的時間，供下一圈使用
        last_capture = current_capture;

        // --- 速度計算邏輯 ---
        // 簡單濾波: 假設輪子最快 10ms 轉一圈 (時速約 720km/h)，過濾掉彈跳雜訊
        // 如果 diff 太大 (例如 > 20000000 = 20秒)，可能是剛開機的第一筆，也忽略
        if (diff > 10000 && diff < 20000000)
        {
            float time_sec = diff / 1000000.0f; // 轉成秒
            float speed_mps = WHEEL_CIRCUMFERENCE / time_sec;
            bike_speed_kmh = speed_mps * 3.6f;

            // 更新最後活動時間 (給 Task 做歸零判定用)
            last_hall_tick = HAL_GetTick();

            // 除錯用：印出正確的間隔 (應該會是 500000 ~ 1000000 左右，看你手速)
            // printf("[ISR] Valid Diff: %lu, Speed: %.1f\r\n", diff, bike_speed_kmh);
        }
    }
}
void Alarm_Beep(int times, int duration_ms) {
    for (int i = 0; i < times; i++) {
        // 1. 開啟 (響)
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        HAL_Delay(duration_ms);

        // 2. 關閉 (靜音)
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        HAL_Delay(duration_ms); // 間隔時間
    }
}
void LCD_Clear(void)
{
    // 方法：把兩行都印滿空白，就等於清除螢幕了
    LCD_SetCursor(0, 0);
    LCD_String("                    "); // 16 個空白

    LCD_SetCursor(1, 0);
    LCD_String("                    "); // 16 個空白

    // 清除後把游標歸位
    LCD_SetCursor(0, 0);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSystemTask */
/**
* @brief Function implementing the SystemTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSystemTask */
void StartSystemTask(void const * argument)
{
  /* USER CODE BEGIN StartSystemTask */

  // 用來記錄上一次的狀態，以偵測是否發生改變
  // 設為一個不存在的狀態 (如 255) 確保第一次開機一定會刷新螢幕
  SystemState_t last_loop_state = 255;

  /* Infinite loop */
  for(;;)
  {
    // =========================================================
    // Part 0: [新增] 狀態改變偵測 (統一處理 LCD)
    // =========================================================
	  if (currentState != last_loop_state)
	      {
	      LCD_Clear();

	      if (currentState == STATE_LOCKED) {
	          // [修改這裡] 上鎖時，顯示這趟騎乘的總熱量

	          // 第一行：顯示騎乘結束
	          LCD_SetCursor(0, 0);
	          LCD_String("Locked!       ");

	          // 第二行：顯示總熱量
	          char msg[20];
	          snprintf(msg, sizeof(msg), "Total: %.2f kcal", total_kcal);
	          LCD_SetCursor(1, 0);
	          LCD_String(msg);

	          // [重要] 顯示完之後，再歸零熱量
	          total_kcal = 0;

	          // LED 亮起表示上鎖
	          HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
	          HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_SET);
	      }
	      else if (currentState == STATE_UNLOCKED) {
	          // 解鎖時的畫面 (保持原本的)
	    	  LCD_SetCursor(0, 0);
	          LCD_String("Bike Unlocked!  ");

	          // 顯示目前熱量 (剛開始是 0.00)
	          char msg[20];
	          snprintf(msg, sizeof(msg), "Kcal: %.2f      ", total_kcal);
	          LCD_SetCursor(1, 0);
	          LCD_String(msg);

	          // 重置計時器
	          last_motion_tick = HAL_GetTick();
	          last_kcal_calc_tick = HAL_GetTick();

	          HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
	          HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);

	      }
	      last_loop_state = currentState;
	  }

    // =========================================================
    // Part 1: 按鈕偵測 (只負責改變 currentState)
    // =========================================================
    GPIO_PinState raw_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

    if (raw_state != last_btn_state) {
        osDelay(20);
        current_btn_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

        if (current_btn_state != last_btn_state) {
            // 超時歸零 Pattern
            if ((HAL_GetTick() - last_btn_activity) > PATTERN_TIMEOUT) {
                if (current_btn_state == BTN_PRESSED_STATE) btn_pattern_stage = 0;
            }

            if (current_btn_state == BTN_PRESSED_STATE) {
                btn_press_start_time = HAL_GetTick();
            }
            else {
                // 放開按鈕
                uint32_t press_duration = HAL_GetTick() - btn_press_start_time;
                last_btn_activity = HAL_GetTick();

                // 短按邏輯
                if (press_duration < PRESS_SHORT_MAX && press_duration > 50) {
                    if (btn_pattern_stage == 0) btn_pattern_stage = 1;
                    else if (btn_pattern_stage == 1) btn_pattern_stage = 2;
                    else btn_pattern_stage = 0;
                }
                // 長按邏輯 (觸發切換狀態)
                else if (press_duration > PRESS_LONG_MIN) {
                    if (btn_pattern_stage == 2) {
                        // [修改] 這裡只負責改狀態，LCD 交給 Part 0 處理
                        if (currentState == STATE_LOCKED) {
                            currentState = STATE_UNLOCKED;
                        }
                        else {
                            // 手動上鎖前，若要顯示這次騎了多少卡路里，可以暫時覆蓋 LCD
                            // 但因為 Part 0 會馬上刷新成 "LOCKED"，這個顯示會很快消失
                            // 簡單作法：直接切換，讓它歸零
                            currentState = STATE_LOCKED;
                        }
                        btn_pattern_stage = 0;
                    } else {
                        btn_pattern_stage = 0;
                    }
                }
            }
            last_btn_state = current_btn_state;
        }
    }

    // =========================================================
    // Part 2: 持續執行的邏輯 (感測器、計算)
    // =========================================================
    switch (currentState) {
        case STATE_LOCKED:
            // 防盜偵測
            if (g_MotionDetected == 1) {
                // 這裡不用改
                LCD_SetCursor(1, 0);
                LCD_String("!! WARNING !!   ");
                Alarm_Beep(3, 500);
                // [新增] 寫入歷史紀錄邏輯
                // 限制：距離上一次寫入至少要過 2 秒 (2000ms)，避免瞬間填滿
                if ((HAL_GetTick() - last_motion_save_tick) > 2000) {

                	// 只有當陣列還沒滿的時候才寫入 (或是你可以改成滿了就覆蓋舊的)
                    // 這裡示範：存滿 10 筆就不存了，直到解鎖清空
                    if (history_count < MAX_HISTORY_LEN) {
                    	// 存入開機後經過的秒數
                        motion_history[history_count] = HAL_GetTick() / 1000;
                        history_count++;
                    }

                    // 更新最後寫入時間
                    last_motion_save_tick = HAL_GetTick();
                }
                g_MotionDetected = 0; // 清除旗標

                // 為了讓 "WARNING" 消失並回到 "LOCKED"，可以在這裡設一個 flag
                // 或是讓 Part 0 的邏輯在下一次循環修復它，不過暫時這樣也可以
            }
            if (request_buzzer == 1) {
            	Alarm_Beep(3, 200); // 叫 3 聲，每聲 200ms
                request_buzzer = 0; // 處理完畢，歸零
            }

            break;

        case STATE_UNLOCKED:
            // 1. 取得加速度
            BSP_ACCELERO_AccGetXYZ(pDataXYZ);

            // 2. 停車歸零邏輯
            if ((HAL_GetTick() - last_hall_tick) > 3000) {
                bike_speed_kmh = 0.0f;
            }

            // 3. METs 與熱量計算 (這段保持你的原樣，沒變)
            if (bike_speed_kmh < 1.0f) current_mets = 1.0f;
            else if (bike_speed_kmh < 16.0f) current_mets = 4.0f;
            else if (bike_speed_kmh < 19.0f) current_mets = 6.8f;
            else if (bike_speed_kmh < 22.0f) current_mets = 8.0f;
            else if (bike_speed_kmh < 26.0f) current_mets = 10.0f;
            else current_mets = 12.0f;

            uint32_t current_tick = HAL_GetTick();
            float dt_seconds = (current_tick - last_kcal_calc_tick) / 1000.0f;
            if (current_mets > 1.5f && dt_seconds > 0) {
                float time_hour = dt_seconds / 3600.0f;
                total_kcal += current_mets * user_weight * time_hour;
            }
            last_kcal_calc_tick = current_tick;

            // 4. 自動上鎖偵測
            if (bike_speed_kmh > 0.5f) {
                last_motion_tick = HAL_GetTick();
                g_MotionDetected = 0;
            }

            // [修改] 自動上鎖觸發
            if ((HAL_GetTick() - last_motion_tick) > AUTO_LOCK_TIMEOUT) {
                // 這裡只要改狀態，Part 0 會負責更新 LCD 和 LED
                currentState = STATE_LOCKED;
            }

            // 5. 定時更新 LCD (顯示速度/熱量)
            // [注意] 這會覆蓋掉 "Bike Unlocked" 字樣，這是正常的
            if (HAL_GetTick() - last_print > 1000) {
                char spd_str[20];
                char row0_buf[22];
                snprintf(spd_str, sizeof(spd_str), "Speed: %.1f km/h", bike_speed_kmh);
                snprintf(row0_buf, 21, "%-20.20s", spd_str);
                LCD_SetCursor(0, 0);
                LCD_String(row0_buf);

                char cal_str[20];
                char row1_buf[22];
                snprintf(cal_str, sizeof(cal_str), "Kcal: %.2f", total_kcal);
                snprintf(row1_buf, 21, "%-20.20s", cal_str);
                LCD_SetCursor(1, 0);
                LCD_String(row1_buf);

                last_print = HAL_GetTick();
            }
            break;
    }
    osDelay(10);
  }
  /* USER CODE END StartSystemTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void const * argument)
{
  /* USER CODE BEGIN StartCommTask */
  /* Infinite loop */
  for(;;)
  {
	// [新增] 處理藍牙事件
	MX_BlueNRG_MS_Process();
	osDelay(10); // 給一點時間釋放資源

  }
  /* USER CODE END StartCommTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
