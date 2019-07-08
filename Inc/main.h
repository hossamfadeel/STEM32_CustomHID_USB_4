/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void Check_JOYSTICK(void);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
static volatile bool My_check = false;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define QSPI_BK1_IO2_Pin GPIO_PIN_2
#define QSPI_BK1_IO2_GPIO_Port GPIOE
#define LED1_RED_Pin GPIO_PIN_3
#define LED1_RED_GPIO_Port GPIOE
#define MEMS_LED_Pin GPIO_PIN_4
#define MEMS_LED_GPIO_Port GPIOE
#define LCD_BL_CTRL_Pin GPIO_PIN_5
#define LCD_BL_CTRL_GPIO_Port GPIOE
#define ARD_D5_Pin GPIO_PIN_6
#define ARD_D5_GPIO_Port GPIOE
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define PSRAM_A0_Pin GPIO_PIN_0
#define PSRAM_A0_GPIO_Port GPIOF
#define PSRAM_A1_Pin GPIO_PIN_1
#define PSRAM_A1_GPIO_Port GPIOF
#define PSRAM_A2_Pin GPIO_PIN_2
#define PSRAM_A2_GPIO_Port GPIOF
#define PSRAM_A3_Pin GPIO_PIN_3
#define PSRAM_A3_GPIO_Port GPIOF
#define PSRAM_A4_Pin GPIO_PIN_4
#define PSRAM_A4_GPIO_Port GPIOF
#define PSRAM_A5_Pin GPIO_PIN_5
#define PSRAM_A5_GPIO_Port GPIOF
#define ARD_D0_Pin GPIO_PIN_6
#define ARD_D0_GPIO_Port GPIOF
#define LEFT_JOY_Pin GPIO_PIN_7
#define LEFT_JOY_GPIO_Port GPIOF
#define QSPI_BK1_IO0_Pin GPIO_PIN_8
#define QSPI_BK1_IO0_GPIO_Port GPIOF
#define QSPI_BK1_IO1_Pin GPIO_PIN_9
#define QSPI_BK1_IO1_GPIO_Port GPIOF
#define ARD_D3_Pin GPIO_PIN_10
#define ARD_D3_GPIO_Port GPIOF
#define STLK_MCO_Pin GPIO_PIN_0
#define STLK_MCO_GPIO_Port GPIOH
#define WIFI_RST_Pin GPIO_PIN_1
#define WIFI_RST_GPIO_Port GPIOH
#define ARD_A0_Pin GPIO_PIN_0
#define ARD_A0_GPIO_Port GPIOC
#define CTP_INT_Pin GPIO_PIN_1
#define CTP_INT_GPIO_Port GPIOC
#define CODEC_ext_SD_Pin GPIO_PIN_2
#define CODEC_ext_SD_GPIO_Port GPIOC
#define CODEC_SD_Pin GPIO_PIN_3
#define CODEC_SD_GPIO_Port GPIOC
#define B_USER_Pin GPIO_PIN_0
#define B_USER_GPIO_Port GPIOA
#define ARD_A1_Pin GPIO_PIN_1
#define ARD_A1_GPIO_Port GPIOA
#define ARD_A2_Pin GPIO_PIN_2
#define ARD_A2_GPIO_Port GPIOA
#define CODEC_MCK_Pin GPIO_PIN_3
#define CODEC_MCK_GPIO_Port GPIOA
#define ARD_D8_Pin GPIO_PIN_4
#define ARD_D8_GPIO_Port GPIOA
#define ARD_A3_Pin GPIO_PIN_5
#define ARD_A3_GPIO_Port GPIOA
#define SD_CMD_Pin GPIO_PIN_6
#define SD_CMD_GPIO_Port GPIOA
#define DFSDM2_DATIN1_Pin GPIO_PIN_7
#define DFSDM2_DATIN1_GPIO_Port GPIOA
#define ARD_A5_Pin GPIO_PIN_4
#define ARD_A5_GPIO_Port GPIOC
#define LED2_GREEN_Pin GPIO_PIN_5
#define LED2_GREEN_GPIO_Port GPIOC
#define ARD_D6_Pin GPIO_PIN_0
#define ARD_D6_GPIO_Port GPIOB
#define ARD_A4_Pin GPIO_PIN_1
#define ARD_A4_GPIO_Port GPIOB
#define QSPI_CLK_Pin GPIO_PIN_2
#define QSPI_CLK_GPIO_Port GPIOB
#define SD_Detect_Pin GPIO_PIN_11
#define SD_Detect_GPIO_Port GPIOF
#define PSRAM_A6_Pin GPIO_PIN_12
#define PSRAM_A6_GPIO_Port GPIOF
#define PSRAM_A7_Pin GPIO_PIN_13
#define PSRAM_A7_GPIO_Port GPIOF
#define PSRAM_A8_Pin GPIO_PIN_14
#define PSRAM_A8_GPIO_Port GPIOF
#define PSRAM_A9_Pin GPIO_PIN_15
#define PSRAM_A9_GPIO_Port GPIOF
#define PSRAM_A10_Pin GPIO_PIN_0
#define PSRAM_A10_GPIO_Port GPIOG
#define PSRAM_A11_Pin GPIO_PIN_1
#define PSRAM_A11_GPIO_Port GPIOG
#define LCD_PSRAM_D4_Pin GPIO_PIN_7
#define LCD_PSRAM_D4_GPIO_Port GPIOE
#define LCD_PSRAM_D5_Pin GPIO_PIN_8
#define LCD_PSRAM_D5_GPIO_Port GPIOE
#define LCD_PSRAM_D6_Pin GPIO_PIN_9
#define LCD_PSRAM_D6_GPIO_Port GPIOE
#define LCD_PSRAM_D7_Pin GPIO_PIN_10
#define LCD_PSRAM_D7_GPIO_Port GPIOE
#define LCD_PSRAM_D8_Pin GPIO_PIN_11
#define LCD_PSRAM_D8_GPIO_Port GPIOE
#define LCD_PSRAM_D9_Pin GPIO_PIN_12
#define LCD_PSRAM_D9_GPIO_Port GPIOE
#define LCD_PSRAM_D10_Pin GPIO_PIN_13
#define LCD_PSRAM_D10_GPIO_Port GPIOE
#define LCD_PSRAM_D11_Pin GPIO_PIN_14
#define LCD_PSRAM_D11_GPIO_Port GPIOE
#define LCD_PSRAM_D12_Pin GPIO_PIN_15
#define LCD_PSRAM_D12_GPIO_Port GPIOE
#define ARD_D15_Pin GPIO_PIN_10
#define ARD_D15_GPIO_Port GPIOB
#define ARD_D14_Pin GPIO_PIN_11
#define ARD_D14_GPIO_Port GPIOB
#define ARD_D13_Pin GPIO_PIN_12
#define ARD_D13_GPIO_Port GPIOB
#define LCD_CTP_RST_Pin GPIO_PIN_13
#define LCD_CTP_RST_GPIO_Port GPIOB
#define LCD_TE_Pin GPIO_PIN_14
#define LCD_TE_GPIO_Port GPIOB
#define WIFI_WKUP_Pin GPIO_PIN_15
#define WIFI_WKUP_GPIO_Port GPIOB
#define LCD_PSRAM_D13_Pin GPIO_PIN_8
#define LCD_PSRAM_D13_GPIO_Port GPIOD
#define LCD_PSRAM_D14_Pin GPIO_PIN_9
#define LCD_PSRAM_D14_GPIO_Port GPIOD
#define LCd_PSRAM_D15_Pin GPIO_PIN_10
#define LCd_PSRAM_D15_GPIO_Port GPIOD
#define PSRAM_A16_Pin GPIO_PIN_11
#define PSRAM_A16_GPIO_Port GPIOD
#define PSRAM_A17_Pin GPIO_PIN_12
#define PSRAM_A17_GPIO_Port GPIOD
#define QSPI_BK1_IO3_Pin GPIO_PIN_13
#define QSPI_BK1_IO3_GPIO_Port GPIOD
#define LCD_PSRAM_D0_Pin GPIO_PIN_14
#define LCD_PSRAM_D0_GPIO_Port GPIOD
#define LCD_PSRAM_D1_Pin GPIO_PIN_15
#define LCD_PSRAM_D1_GPIO_Port GPIOD
#define PSRAM_A12_Pin GPIO_PIN_2
#define PSRAM_A12_GPIO_Port GPIOG
#define PSRAM_A13_Pin GPIO_PIN_3
#define PSRAM_A13_GPIO_Port GPIOG
#define PSRAM_A14_Pin GPIO_PIN_4
#define PSRAM_A14_GPIO_Port GPIOG
#define PSRAM_A15_Pin GPIO_PIN_5
#define PSRAM_A15_GPIO_Port GPIOG
#define QSPI_BK1_NCS_Pin GPIO_PIN_6
#define QSPI_BK1_NCS_GPIO_Port GPIOG
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_8
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOG
#define I2CFMP1_SCL_Pin GPIO_PIN_6
#define I2CFMP1_SCL_GPIO_Port GPIOC
#define I2CFMP_SDA_Pin GPIO_PIN_7
#define I2CFMP_SDA_GPIO_Port GPIOC
#define SD_D0_Pin GPIO_PIN_8
#define SD_D0_GPIO_Port GPIOC
#define SD_D1_Pin GPIO_PIN_9
#define SD_D1_GPIO_Port GPIOC
#define DFSDM1_CKOUT_Pin GPIO_PIN_8
#define DFSDM1_CKOUT_GPIO_Port GPIOA
#define USB_OTG_FS_VBUS_Pin GPIO_PIN_9
#define USB_OTG_FS_VBUS_GPIO_Port GPIOA
#define USB_OTG_FS_ID_Pin GPIO_PIN_10
#define USB_OTG_FS_ID_GPIO_Port GPIOA
#define USB_OTG_FS_DM_Pin GPIO_PIN_11
#define USB_OTG_FS_DM_GPIO_Port GPIOA
#define USB_OTG_FS_DP_Pin GPIO_PIN_12
#define USB_OTG_FS_DP_GPIO_Port GPIOA
#define STLINK_JTMS_SWDIO_Pin GPIO_PIN_13
#define STLINK_JTMS_SWDIO_GPIO_Port GPIOA
#define STLINK_JTMS_SWCLK_Pin GPIO_PIN_14
#define STLINK_JTMS_SWCLK_GPIO_Port GPIOA
#define ARD_D10_Pin GPIO_PIN_15
#define ARD_D10_GPIO_Port GPIOA
#define SD_D2_Pin GPIO_PIN_10
#define SD_D2_GPIO_Port GPIOC
#define SD_D3_Pin GPIO_PIN_11
#define SD_D3_GPIO_Port GPIOC
#define SD_CLK_Pin GPIO_PIN_12
#define SD_CLK_GPIO_Port GPIOC
#define LCD_PSRAM_D2_Pin GPIO_PIN_0
#define LCD_PSRAM_D2_GPIO_Port GPIOD
#define LCD_PSRAM_D3_Pin GPIO_PIN_1
#define LCD_PSRAM_D3_GPIO_Port GPIOD
#define DFSDM2_CKOUT_Pin GPIO_PIN_2
#define DFSDM2_CKOUT_GPIO_Port GPIOD
#define CODEC_CK_Pin GPIO_PIN_3
#define CODEC_CK_GPIO_Port GPIOD
#define LCD_PSRAM_NOE_Pin GPIO_PIN_4
#define LCD_PSRAM_NOE_GPIO_Port GPIOD
#define LCD_PSRAM_NWE_Pin GPIO_PIN_5
#define LCD_PSRAM_NWE_GPIO_Port GPIOD
#define DFSDM1_DATIN1_Pin GPIO_PIN_6
#define DFSDM1_DATIN1_GPIO_Port GPIOD
#define PSRAM_NE1_Pin GPIO_PIN_7
#define PSRAM_NE1_GPIO_Port GPIOD
#define VCP_RX_Pin GPIO_PIN_9
#define VCP_RX_GPIO_Port GPIOG
#define LCD_NE3_Pin GPIO_PIN_10
#define LCD_NE3_GPIO_Port GPIOG
#define WIFI_SPI_CSN_Pin GPIO_PIN_11
#define WIFI_SPI_CSN_GPIO_Port GPIOG
#define WIFI_DRDY_Pin GPIO_PIN_12
#define WIFI_DRDY_GPIO_Port GPIOG
#define UP_JOY_Pin GPIO_PIN_13
#define UP_JOY_GPIO_Port GPIOG
#define VCP_TX_Pin GPIO_PIN_14
#define VCP_TX_GPIO_Port GPIOG
#define CODEC_INT_Pin GPIO_PIN_15
#define CODEC_INT_GPIO_Port GPIOG
#define STLINK_JTDO_SWCLK_Pin GPIO_PIN_3
#define STLINK_JTDO_SWCLK_GPIO_Port GPIOB
#define ARD_D12_Pin GPIO_PIN_4
#define ARD_D12_GPIO_Port GPIOB
#define ARD_D11_Pin GPIO_PIN_5
#define ARD_D11_GPIO_Port GPIOB
#define DOWN_JOY_Pin GPIO_PIN_6
#define DOWN_JOY_GPIO_Port GPIOB
#define DFSDM2_DATIN7_Pin GPIO_PIN_7
#define DFSDM2_DATIN7_GPIO_Port GPIOB
#define ARD_D9_Pin GPIO_PIN_8
#define ARD_D9_GPIO_Port GPIOB
#define CODEC_WS_Pin GPIO_PIN_9
#define CODEC_WS_GPIO_Port GPIOB
#define PSRAM_NBL0_Pin GPIO_PIN_0
#define PSRAM_NBL0_GPIO_Port GPIOE
#define PSRAM_NBL1_Pin GPIO_PIN_1
#define PSRAM_NBL1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define USB_HID_MODIFIER_LEFT_CTRL   0x01

#define USB_HID_MODIFIER_LEFT_SHIFT  0x02

#define USB_HID_MODIFIER_LEFT_ALT    0x04

#define USB_HID_MODIFIER_LEFT_GUI    0x08 // (Win/Apple/Meta)

#define USB_HID_MODIFIER_RIGHT_CTRL  0x10

#define USB_HID_MODIFIER_RIGHT_SHIFT 0x20

#define USB_HID_MODIFIER_RIGHT_ALT   0x40

#define USB_HID_MODIFIER_RIGHT_GUI   0x80

#define USB_HID_KEY_A     0x04

#define USB_HID_KEY_B     0x05

#define USB_HID_KEY_C     0x06

#define USB_HID_KEY_D     0x07

#define USB_HID_KEY_E     0x08

#define USB_HID_KEY_F     0x09

#define USB_HID_KEY_G     0x0A

#define USB_HID_KEY_H     0x0B

#define USB_HID_KEY_I     0x0C

#define USB_HID_KEY_J     0x0D

#define USB_HID_KEY_K     0x0E

#define USB_HID_KEY_L     0x0F

#define USB_HID_KEY_M     0x10

#define USB_HID_KEY_N     0x11

#define USB_HID_KEY_O     0x12

#define USB_HID_KEY_P     0x13

#define USB_HID_KEY_Q     0x14

#define USB_HID_KEY_R     0x15

#define USB_HID_KEY_S     0x16

#define USB_HID_KEY_T     0x17

#define USB_HID_KEY_U     0x18

#define USB_HID_KEY_V     0x19

#define USB_HID_KEY_W     0x1A

#define USB_HID_KEY_X     0x1B

#define USB_HID_KEY_Y     0x1C

#define USB_HID_KEY_Z     0x1D


/**
 * Button status typedef
 * 
 * Parameters:
 *     - USB_HIDDEVICE_Button_Released
 *         Button is not pressed
 *     - USB_HIDDEVICE_Button_Pressed
 *         Button is pressed
 */
typedef enum {
    USB_HIDDEVICE_Button_Released = 0,
    USB_HIDDEVICE_Button_Pressed = 1
} USB_HIDDEVICE_Button_t;


/** @defgroup JOYSTICK_BUTTON  BUTTON Constants
 * //We have 4 PINs for the JOYSTICK
  * @{
  */
#define JOYn                  4//JOYSTICK_BUTTONs

//#define JOYSTICK_BUTTONs           ((uint8_t)4)

/**
 * @brief JOYSTICK Types Definition
 */
typedef enum
{
  JOY_NONE  = 0, 
  JOY_LEFT  = 1,
  JOY_RIGHT = 2,
  JOY_DOWN  = 3,
  JOY_UP    = 4
} JOYState_TypeDef;

typedef enum
{
  JOY_MODE_GPIO = 0,
  JOY_MODE_EXTI = 1
} JOYMode_TypeDef;




/**
* @brief Joystick Right push-button
*/
#define RIGHT_JOY_Pin GPIO_PIN_6
#define RIGHT_JOY_GPIO_Port GPIOF
#define RIGHT_JOY_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define RIGHT_JOY_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOA_CLK_DISABLE()
#define RIGHT_JOY_EXTI_IRQn               EXTI9_5_IRQn

/**
* @brief Joystick Left push-button
*/

#define LEFT_JOY_Pin GPIO_PIN_7
#define LEFT_JOY_GPIO_Port GPIOF
#define LEFT_JOY_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define LEFT_JOY_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOA_CLK_DISABLE()
#define LEFT_JOY_EXTI_IRQn                EXTI9_5_IRQn

/**
* @brief Joystick Up push-button
*/
#define UP_JOY_Pin GPIO_PIN_13
#define UP_JOY_GPIO_Port GPIOG
#define UP_JOY_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define UP_JOY_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOA_CLK_DISABLE()
#define UP_JOY_EXTI_IRQn                  EXTI15_10_IRQn

/**
 * @brief Joystick Down push-button
 */

#define DOWN_JOY_Pin GPIO_PIN_6
#define DOWN_JOY_GPIO_Port GPIOB
#define DOWN_JOY_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define DOWN_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOA_CLK_DISABLE()
#define DOWN_JOY_EXTI_IRQn                EXTI9_5_IRQn 


#define JOYx_GPIO_CLK_ENABLE(__JOY__)     do { if((__JOY__) == JOY_DOWN)  { DOWN_JOY_GPIO_CLK_ENABLE();  } else \
                                               if((__JOY__) == JOY_LEFT)  { LEFT_JOY_GPIO_CLK_ENABLE();  } else \
                                               if((__JOY__) == JOY_RIGHT) { RIGHT_JOY_GPIO_CLK_ENABLE(); } else \
                                               if((__JOY__) == JOY_UP)    { UP_JOY_GPIO_CLK_ENABLE(); }  } while(0)

#define JOYx_GPIO_CLK_DISABLE(__JOY__)    do { if((__JOY__) == JOY_DOWN)  { DOWN_JOY_GPIO_CLK_DISABLE();  } else \
                                               if((__JOY__) == JOY_LEFT)  { LEFT_JOY_GPIO_CLK_DISABLE();  } else \
                                               if((__JOY__) == JOY_RIGHT) { RIGHT_JOY_GPIO_CLK_DISABLE(); } else \
                                               if((__JOY__) == JOY_UP)    { UP_JOY_GPIO_CLK_DISABLE(); }  } while(0)

#define JOY_ALL_PINS                      (RIGHT_JOY_PIN | LEFT_JOY_PIN | UP_JOY_PIN | DOWN_JOY_PIN)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
