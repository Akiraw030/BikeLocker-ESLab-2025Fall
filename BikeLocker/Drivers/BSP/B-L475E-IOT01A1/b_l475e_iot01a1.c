/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file  : b_l475e_iot01a1.c
  * @brief : Source file for the BSP Common driver
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "b_l475e_iot01a1.h"

/** @defgroup BSP BSP
 * @{
 */

/** @defgroup B_L475E_IOT01A1 B_L475E_IOT01A1
 * @{
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL B_L475E_IOT01A1 LOW LEVEL
 *  @brief This file provides set of firmware functions to manage Leds and push-button
 *         available on STM32L4xx-Nucleo Kit from STMicroelectronics.
 * @{
 */

/**
 * @}
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Private_Defines B_L475E_IOT01A1 LOW LEVEL Private Defines
 * @{
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_FunctionPrototypes B_L475E_IOT01A1 LOW LEVEL Private Function Prototypes
 * @{
 */

/**
 * @}
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Private_Variables B_L475E_IOT01A1 LOW LEVEL Private Variables
 * @{
 */
typedef void (* BSP_LED_GPIO_Init) (void);
static GPIO_TypeDef*  LED_PORT[LEDn] = {LED2_GPIO_PORT};
static const uint16_t LED_PIN[LEDn]  = {LED2_PIN};
static void LED_USER_GPIO_Init(void);

/**
 * @}
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Private_Functions B_L475E_IOT01A1 LOW LEVEL Private Functions
 * @{
 */
/**
 * @brief  This method returns the STM32L4xx NUCLEO BSP Driver revision
 * @retval version: 0xXYZR (8bits for each decimal, R for RC)
 */
int32_t BSP_GetVersion(void)
{
  return (int32_t)__B_L475E_IOT01A1_BSP_VERSION;
}

/**
 * @brief  Configures LED on GPIO and/or on MFX.
 * @param  Led: LED to be configured.
 *              This parameter can be one of the following values:
 *              @arg  LED2, LED4, ...
 * @retval HAL status
 */
int32_t BSP_LED_Init(Led_TypeDef Led)
{
  static const BSP_LED_GPIO_Init LedGpioInit[LEDn] = {LED_USER_GPIO_Init};
  LedGpioInit[Led]();
  return BSP_ERROR_NONE;
}

/**
 * @brief  DeInit LEDs.
 * @param  Led: LED to be configured.
 *              This parameter can be one of the following values:
 *              @arg  LED2, LED4, ...
 * @note Led DeInit does not disable the GPIO clock nor disable the Mfx
 * @retval HAL status
 */
int32_t BSP_LED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Turn off LED */
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
  /* DeInit the GPIO_LED pin */
  gpio_init_structure.Pin = LED_PIN[Led];
  HAL_GPIO_DeInit(LED_PORT[Led], gpio_init_structure.Pin);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Turns selected LED On.
 * @param  Led: LED to be set on
 *              This parameter can be one of the following values:
 *              @arg  LED1
 *              @arg  LED2
 *              @arg  LED3
 *              @arg  LED4
 * @retval HAL status
 */
int32_t BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT [Led], LED_PIN [Led], GPIO_PIN_SET);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Turns selected LED Off.
 * @param  Led: LED to be set off
 *              This parameter can be one of the following values:
 *              @arg  LED1
 *              @arg  LED2
 *              @arg  LED3
 *              @arg  LED4
 * @retval HAL status
 */
int32_t BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT [Led], LED_PIN [Led], GPIO_PIN_RESET);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Toggles the selected LED.
 * @param  Led: LED to be toggled
 *              This parameter can be one of the following values:
 *              @arg  LED1
 *              @arg  LED2
 *              @arg  LED3
 *              @arg  LED4
 * @retval HAL status
 */
int32_t BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Get the status of the LED.
 * @param  Led: LED for which get the status
 *              This parameter can be one of the following values:
 *              @arg  LED1
 *              @arg  LED2
 *              @arg  LED3
 *              @arg  LED4
 * @retval HAL status (1=high, 0=low)
 */
int32_t BSP_LED_GetState(Led_TypeDef Led)
{
  return (int32_t)(HAL_GPIO_ReadPin (LED_PORT [Led], LED_PIN [Led]) == GPIO_PIN_RESET);
}
/**
  * @brief
  * @retval None
  */
static void LED_USER_GPIO_Init(void) {

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUS_BSP_LED_GPIO_PORT, BUS_BSP_LED_GPIO_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : PTPIN */
  GPIO_InitStruct.Pin = BUS_BSP_LED_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUS_BSP_LED_GPIO_PORT, &GPIO_InitStruct);

}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
//I2C_HandleTypeDef hbus_i2c1;
int32_t BSP_I2C1_Init(void);
int32_t BSP_I2C1_DeInit(void);
int32_t BSP_I2C1_IsReady(uint16_t DevAddr, uint32_t Trials);
int32_t BSP_I2C1_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);

extern I2C_HandleTypeDef hi2c2;
void SENSOR_IO_Init(void)
{
  BSP_I2C1_Init();
}

/**
  * @brief  Deinitialize Sensor I/O (I2C)
  * @retval None
  */
void SENSOR_IO_DeInit(void)
{
  BSP_I2C1_DeInit();
}

/**
  * @brief  Write value to register
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Value: Data to write
  * @retval None
  */
void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  BSP_I2C1_WriteReg(Addr, Reg, &Value, 1);
}

/**
  * @brief  Read single register value
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Register value
  */
uint8_t SENSOR_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t value = 0;
  BSP_I2C1_ReadReg(Addr, Reg, &value, 1);
  return value;
}

/**
  * @brief  Read multiple bytes from sensor
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  pBuffer: pointer to buffer
  * @param  Length: number of bytes
  * @retval None
  */
void SENSOR_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
  BSP_I2C1_ReadReg(Addr, Reg, pBuffer, Length);
}

/**
  * @brief  Delay for sensor timing
  * @param  Delay: delay in ms
  * @retval None
  */
void SENSOR_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
/* I2C1 init function */
int32_t BSP_I2C1_Init(void)
{
   return 0;
}

/* I2C deinit */
int32_t BSP_I2C1_DeInit(void)
{
  return 0;
}

/* Check device readiness */
int32_t BSP_I2C1_IsReady(uint16_t DevAddr, uint32_t Trials)
{
  return (HAL_I2C_IsDeviceReady(&hi2c2, DevAddr, Trials, HAL_MAX_DELAY) == HAL_OK) ? 0 : -1;
}

/* Write one or more bytes to a sensor register */
int32_t BSP_I2C1_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  if (HAL_I2C_Mem_Write(&hi2c2, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, HAL_MAX_DELAY) != HAL_OK)
    return -1;
  return 0;
}

/* Read one or more bytes from a sensor register */
int32_t BSP_I2C1_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  if (HAL_I2C_Mem_Read(&hi2c2, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, HAL_MAX_DELAY) != HAL_OK)
    return -1;
  return 0;
}
