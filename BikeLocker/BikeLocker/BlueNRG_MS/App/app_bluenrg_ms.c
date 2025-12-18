/**
  ******************************************************************************
  * @file    app_bluenrg_ms.c
  * @author  SRA Application Team
  * @brief   BlueNRG-M0 initialization and applicative code
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_bluenrg_ms.h"

#include "hci.h"
#include "hci_le.h"
#include "hci_tl.h"
#include "link_layer.h"
#include "sensor.h"
#include "gatt_db.h"

#include "compiler.h"
#include "bluenrg_utils.h"
#include "b_l475e_iot01a1.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_hal_aci.h"
#include "sm.h"
#include "stm32l4xx_hal_tim.h"

/* USER CODE BEGIN Includes */
#include "stm32l475e_iot01_accelero.h"
#include "cmsis_os.h"               // <-- ADD THIS (for osDelay)
#include "bluenrg_gatt_aci.h"       // <-- ADD THIS (for GATT_SERVER_ATTR_WRITE)
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/**
 * 1 to send environmental and motion data when pushing the user button
 * 0 to send environmental and motion data automatically (period = 1 sec)
 */
#define USE_BUTTON 0

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern AxesRaw_t x_axes;
extern AxesRaw_t g_axes;
extern AxesRaw_t m_axes;
extern AxesRaw_t q_axes;

extern volatile uint8_t set_connectable;
extern volatile int     connected;
/* at startup, suppose the X-NUCLEO-IDB04A1 is used */
uint8_t bnrg_expansion_board = IDB04A1;
uint8_t bdaddr[BDADDR_SIZE];
static volatile uint8_t user_button_init_state = 1;
static volatile uint8_t user_button_pressed = 0;
volatile uint8_t request_buzzer = 0;
/* USER CODE BEGIN PV */
typedef enum {
    STATE_LOCKED,
    STATE_UNLOCKED
} SystemState_t;

// 引入 main.c 的變數
extern const float user_weight;
extern volatile SystemState_t currentState;
extern volatile float bike_speed_kmh;
extern volatile float total_kcal;
extern uint32_t motion_history[MAX_HISTORY_LEN];
extern uint8_t  history_count;
extern uint8_t  history_head;
volatile uint32_t abnormal_timestamp = 0;
/* [新增] 1. 定義 SPEC.md 裡的 UUID 和 Handle 變數 */
// Service UUID: 0000ffe0-0000-1000-8000-00805f9b34fb
const uint8_t bikelocker_service_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xe0, 0xff, 0x00, 0x00
};

// Lock Control: 0000ffe1...
const uint8_t lock_char_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xe1, 0xff, 0x00, 0x00
};

// Abnormal History: 0000ffe2...
const uint8_t history_char_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xe2, 0xff, 0x00, 0x00
};

// Speed: 0000ffe3...
const uint8_t speed_char_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xe3, 0xff, 0x00, 0x00
};

// Calorie: 0000ffe4...
const uint8_t calorie_char_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xe4, 0xff, 0x00, 0x00
};
// 用來存放系統分配的 Handle
uint16_t bikelocker_serv_handle, lock_char_handle, history_char_handle, speed_char_handle, calorie_char_handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Process(void);
static void User_Init(void);
tBleStatus Add_BikeLocker_Service(void);
/* USER CODE BEGIN PFP */
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle, uint16_t Attr_Handle, uint16_t Offset, uint16_t Attr_Data_Length, uint8_t *Attr_Data)
{
    // 檢查是否為 Lock Control 特徵值 (+1 是因為 Value Handle)
    if (Attr_Handle == lock_char_handle + 1)
    {
        uint8_t command = Attr_Data[0];

        if (command == 0x01) {      // Lock 指令
            currentState = STATE_LOCKED;

            // 回傳狀態確認
            uint8_t status = 0x01;
            aci_gatt_update_char_value(bikelocker_serv_handle, lock_char_handle, 0, 1, &status);
        }
        else if (command == 0x02) { // UNLOCK 指令

                // 1. 先回傳解鎖狀態 (Status)
                uint8_t status = 0x02;
                aci_gatt_update_char_value(bikelocker_serv_handle, lock_char_handle, 0, 1, &status);

                // 延遲
                for(volatile int k = 0; k < 500000; k++) { __NOP(); }

                // =======================================================
                // 2. 判斷是否有異常紀錄
                // =======================================================
                if (history_count == 0) {
                    // [情境 A] 平安無事：只傳 0，不傳時間
                    uint32_t safe_val = 0;
                    aci_gatt_update_char_value(bikelocker_serv_handle, history_char_handle, 0, 4, (uint8_t*)&safe_val);

                    // 這裡直接結束，不傳解鎖時間
                }
                else {
                    // [情境 B] 有異常：傳送 震動歷史 + 解鎖時間

                    // B-1. 傳送震動歷史
                    for (int i = 0; i < history_count; i++) {
                        uint32_t history_sec = motion_history[i];
                        aci_gatt_update_char_value(bikelocker_serv_handle, history_char_handle, 0, 4, (uint8_t*)&history_sec);
                        for(volatile int k = 0; k < 200000; k++) { __NOP(); }
                    }

                    // B-2. 最後傳送解鎖時間 (當作結尾)
                    uint32_t current_uptime_sec = HAL_GetTick() / 1000;
                    aci_gatt_update_char_value(bikelocker_serv_handle, history_char_handle, 0, 4, (uint8_t*)&current_uptime_sec);
                    for(volatile int k = 0; k < 200000; k++) { __NOP(); }
                }

                // =======================================================
                // 3. 清除計數與更改狀態
                // =======================================================
                history_count = 0;
                currentState = STATE_UNLOCKED;
            }
        else if (command == 0x03) { // Ringing
        	// [修改] 不要直接叫，改為設定旗標
            request_buzzer = 1;
        }
    }
}
/* USER CODE END PFP */

#if PRINT_CSV_FORMAT
extern volatile uint32_t ms_counter;
/**
 * @brief  This function is a utility to print the log time
 *         in the format HH:MM:SS:MSS (DK GUI time format)
 * @param  None
 * @retval None
 */
void print_csv_time(void){
  uint32_t ms = HAL_GetTick();
  PRINT_CSV("%02ld:%02ld:%02ld.%03ld", (long)(ms/(60*60*1000)%24), (long)(ms/(60*1000)%60), (long)((ms/1000)%60), (long)(ms%1000));
}
#endif

void MX_BlueNRG_MS_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN BlueNRG_MS_Init_PreTreatment */

  /* USER CODE END BlueNRG_MS_Init_PreTreatment */

  /* Initialize the peripherals and the BLE Stack */
  const char *name = "BlueNRG";
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

  uint8_t  bdaddr_len_out;
  uint8_t  hwVersion;
  uint16_t fwVersion;
  int ret;

  User_Init();

  /* Get the User Button initial state */
  //user_button_init_state = BSP_PB_GetState(BUTTON_KEY);

  hci_init(user_notify, NULL);

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  /*
   * Reset BlueNRG again otherwise we won't
   * be able to change its MAC address.
   * aci_hal_write_config_data() must be the first
   * command after reset otherwise it will fail.
   */
  hci_reset();
  HAL_Delay(100);

  PRINTF("HWver %d\nFWver %d\n", hwVersion, fwVersion);
  if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
    bnrg_expansion_board = IDB05A1;
  }

  ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, BDADDR_SIZE, &bdaddr_len_out, bdaddr);

  if (ret) {
    PRINTF("Read Static Random address failed.\n");
  }

  if ((bdaddr[5] & 0xC0) != 0xC0) {
    PRINTF("Static Random address not well formed.\n");
    while(1);
  }

  /* GATT Init */
  ret = aci_gatt_init();
  if(ret){
    PRINTF("GATT_Init failed.\n");
  }

  /* GAP Init */
  if (bnrg_expansion_board == IDB05A1) {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }
  else {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("GAP_Init failed.\n");
  }

  /* Update device name */
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   strlen(name), (uint8_t *)name);
  if (ret) {
    PRINTF("aci_gatt_update_char_value failed.\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);
  if (ret) {
    PRINTF("aci_gap_set_authentication_requirement failed.\n");
    while(1);
  }

  PRINTF("BLE Stack Initialized\n");

  ret = Add_BikeLocker_Service(); // <--- 加入這個

  if(ret == BLE_STATUS_SUCCESS) {
      PRINTF("BikeLocker Service added successfully.\n");
    }
    else {
      PRINTF("Error while adding service: 0x%02x\r\n", ret);
      // 這裡通常不加 while(1)，避免藍牙失敗導致整台車死機，
      // 但如果要嚴格檢查可以加。
    }

  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);

  /* USER CODE BEGIN BlueNRG_MS_Init_PostTreatment */

  /* USER CODE END BlueNRG_MS_Init_PostTreatment */
}

/*
 * BlueNRG-MS background task
 */
void MX_BlueNRG_MS_Process(void)
{
  /* USER CODE BEGIN BlueNRG_MS_Process_PreTreatment */

  /* USER CODE END BlueNRG_MS_Process_PreTreatment */

  User_Process();
  hci_user_evt_proc();

  /* USER CODE BEGIN BlueNRG_MS_Process_PostTreatment */

  /* USER CODE END BlueNRG_MS_Process_PostTreatment */
}

/**
 * @brief  Initialize User process.
 *
 * @param  None
 * @retval None
 */
static void User_Init(void)
{
  //BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  BSP_LED_Init(LED2);

  //BSP_COM_Init(COM1);
}

/**
 * @brief  Process user input (i.e. pressing the USER button on Nucleo board)
 *         and send the updated acceleration data to the remote client.
 *
 * @param  None
 * @retval None
 */
static void User_Process(void)
{
  if (set_connectable) {
    Set_DeviceConnectable();
    set_connectable = FALSE;
  }

  // 只有連線時才發送
  if (connected) {

    // [新增] 簡易防盜偵測 (之前的邏輯)
    if (currentState == STATE_LOCKED && bike_speed_kmh > 1.0f && abnormal_timestamp == 0) {
        abnormal_timestamp = HAL_GetTick() / 1000;
    }

    static uint32_t last_update = 0;

    // 每 500ms 更新一次數據
    if (HAL_GetTick() - last_update > 1000) {


        // 1. 更新 Speed (UInt16, 0.1km/h)
        uint16_t speed_val = (uint16_t)(bike_speed_kmh * 10);
        aci_gatt_update_char_value(bikelocker_serv_handle, speed_char_handle, 0, 2, (uint8_t*)&speed_val);

        // 2. 更新 Calorie (UInt16, 0.1cal)
        uint16_t cal_val = (uint16_t)(total_kcal * 10);
        aci_gatt_update_char_value(bikelocker_serv_handle, calorie_char_handle, 0, 2, (uint8_t*)&cal_val);

        last_update = HAL_GetTick();
    }
  }
}
/* [新增] 2. 建立 BikeLocker 服務的函式 */
tBleStatus Add_BikeLocker_Service(void)
{
    tBleStatus ret;

    // 1. 加入 Service
    ret = aci_gatt_add_serv(UUID_TYPE_128, bikelocker_service_uuid, PRIMARY_SERVICE, 20, &bikelocker_serv_handle);
    if (ret != BLE_STATUS_SUCCESS) return BLE_STATUS_ERROR;

    // 2. 加入 Lock Control (Write, Notify)
    ret = aci_gatt_add_char(bikelocker_serv_handle, UUID_TYPE_128, lock_char_uuid, 1,
                            CHAR_PROP_WRITE | CHAR_PROP_NOTIFY,
                            ATTR_PERMISSION_NONE,
                            GATT_NOTIFY_ATTRIBUTE_WRITE,
                            16, 1, &lock_char_handle);

    // 3. 加入 Abnormal History (Notify)
    ret = aci_gatt_add_char(bikelocker_serv_handle, UUID_TYPE_128, history_char_uuid, 4,
                            CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, GATT_DONT_NOTIFY_EVENTS,
                            16, 1, &history_char_handle);

    // 4. 加入 Speed (Notify)
    ret = aci_gatt_add_char(bikelocker_serv_handle, UUID_TYPE_128, speed_char_uuid, 2,
                            CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, GATT_DONT_NOTIFY_EVENTS,
                            16, 1, &speed_char_handle);
    PRINTF("Add Speed Char: 0x%02x, Handle: %d\n", ret, speed_char_handle);
    // 5. 加入 Calorie (Notify)
    ret = aci_gatt_add_char(bikelocker_serv_handle, UUID_TYPE_128, calorie_char_uuid, 2,
                            CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, GATT_DONT_NOTIFY_EVENTS,
                            16, 1, &calorie_char_handle);
    PRINTF("Add Calorie Char: 0x%02x, Handle: %d\n", ret, calorie_char_handle);
    return BLE_STATUS_SUCCESS;
}
