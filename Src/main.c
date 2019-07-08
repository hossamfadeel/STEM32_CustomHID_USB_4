/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "usbd_hid.h"
#include "keycodes.h"
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
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm2_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm2_channel7;

FMPI2C_HandleTypeDef hfmpi2c1;

I2S_HandleTypeDef hi2s2;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd;

UART_HandleTypeDef huart10;
UART_HandleTypeDef huart6;

SRAM_HandleTypeDef hsram1;
SRAM_HandleTypeDef hsram2;

/* USER CODE BEGIN PV */
extern   USBD_HandleTypeDef  hUsbDeviceFS;

uint8_t USB_RX_Buffer[64];
uint8_t USB_TX_Buffer[64]={45}; // To send usb data to PC

GPIO_TypeDef* JOY_PORT[JOYn] =  {LEFT_JOY_GPIO_Port,
                                 RIGHT_JOY_GPIO_Port,
                                 DOWN_JOY_GPIO_Port,
                                 UP_JOY_GPIO_Port};

const uint16_t JOY_PIN[JOYn] =  {LEFT_JOY_Pin,
                                 RIGHT_JOY_Pin,
                                 DOWN_JOY_Pin,                               
                                 UP_JOY_Pin};

const uint8_t JOY_IRQn[JOYn] =  {LEFT_JOY_EXTI_IRQn,
                                 RIGHT_JOY_EXTI_IRQn,
                                 DOWN_JOY_EXTI_IRQn,
                                 UP_JOY_EXTI_IRQn};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_DFSDM2_Init(void);
static void MX_FMPI2C1_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2S2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_UART10_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t buffer[4];
static uint8_t chars[] = {
	0x04, // a & A
	0x05, // b & B
	0x06, // c
	0x07, // d
	0x08, // e
	0x09, // f
	0x0a, // g
	0x0b, // h
	0x0c, // i
};

#define KeyBoard_ReportID  0x01
#define Mouse_ReportID     0x02

//uint8_t KeyBoard_Buffer[10];
//
//uint8_t Mouse_Buffer[5];

#define CURSOR_STEP     5
//uint8_t USB_RX_Buffer[64];
//uint8_t USB_TX_Buffer[64]; // To send usb data to PC
HAL_StatusTypeDef CheckUSB_Connection(void);
void SystemClockConfig_STOP(void);
/**
  * @brief  Configures all buttons of the joystick in GPIO or EXTI modes.
  * @param  Joy_Mode: Joystick mode.
  *    This parameter can be one of the following values:
  *     @arg  JOY_MODE_GPIO: Joystick pins will be used as simple IOs
  *     @arg  JOY_MODE_EXTI: Joystick pins will be connected to EXTI line
  *                                 with interrupt generation capability
  * @retval HAL_OK: if all initializations are OK. Other value if error.
  */
uint8_t BSP_JOY_Init(JOYMode_TypeDef Joy_Mode)
{
  JOYState_TypeDef joykey;
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Initialized the Joystick. */
  for (joykey = JOY_LEFT; joykey < (JOY_LEFT + JOYn) ; joykey++)
  {
    /* Enable the JOY clock */
    JOYx_GPIO_CLK_ENABLE(joykey);

    GPIO_InitStruct.Pin = JOY_PIN[joykey];
    GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_PULLDOWN;//GPIO_PULLUP; //GPIO_NOPULL
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if (Joy_Mode == JOY_MODE_GPIO)
    {
      /* Configure Joy pin as input */
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      HAL_GPIO_Init(JOY_PORT[joykey], &GPIO_InitStruct);
    }
    else if (Joy_Mode == JOY_MODE_EXTI)
    {

      /* Configure Joy pin as input with External interrupt */
      GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; //GPIO_MODE_IT_RISING //GPIO_MODE_IT_FALLING //GPIO_MODE_IT_RISING_FALLING
      HAL_GPIO_Init(JOY_PORT[joykey], &GPIO_InitStruct);

      /* Enable and set Joy EXTI Interrupt to the lowest priority */
      HAL_NVIC_SetPriority((IRQn_Type)(JOY_IRQn[joykey]), 0x0F, 0x00);
      HAL_NVIC_EnableIRQ((IRQn_Type)(JOY_IRQn[joykey]));
    }
  }

  return HAL_OK;
}


/**
  * @brief  Unonfigures all GPIOs used as buttons of the joystick.
  * @retval None.
  */
void BSP_JOY_DeInit(void)
{
  JOYState_TypeDef joykey;

  /* Initialized the Joystick. */
  for (joykey = JOY_LEFT; joykey < (JOY_LEFT + JOYn) ; joykey++)
  {
    /* Enable the JOY clock */
    JOYx_GPIO_CLK_ENABLE(joykey);

    HAL_GPIO_DeInit(JOY_PORT[joykey], JOY_PIN[joykey]);
  }
}

/**
* @brief  Returns the current joystick status.
* @retval Code of the joystick key pressed
*          This code can be one of the following values:
*            @arg  JOY_NONE
*            @arg  JOY_DOWN
*            @arg  JOY_LEFT
*            @arg  JOY_RIGHT
*            @arg  JOY_UP
*/

JOYState_TypeDef joyState=JOY_NONE;

struct mouseHID_t {
  uint8_t id;
  union{
        uint8_t Buttons; // 1=Left,  2=MIDDLE, and 3=Right Buttons
        uint8_t B_LEFT: 1;
        uint8_t B_MIDDLE: 1;
        uint8_t B_RIGHT: 1;
        uint8_t B_UNUSED: 5;
        } uButtons;
   //uint8_t buttons;
      
      int8_t x;
      int8_t y;
      int8_t wheel;
  };
 struct mouseHID_t mouseHID;
 
  // HID Keyboard
  struct keyboardHID_t {
      uint8_t id; //report_id
      uint8_t modifiers;
      uint8_t key1;
      uint8_t key2;
      uint8_t key3;
//      uint8_t key4;
//      uint8_t key5;
//      uint8_t key6;
  };
  struct keyboardHID_t keyboardHID;

  
//buffer[0]=1;//reportID
//buffer[1]=0;//modifier
//buffer[2]=0;//OEM
//buffer[3]=0;//keycode data
//buffer[4]=0;//keycode data
//buffer[5]=0;//keycode data
//buffer[6]=0;//keycode data
//buffer[7]=0;//keycode data
  
  //press_report[2] = 7; // send 'd'
 
void Check_JOYSTICK(void)
{
  /* Check the pressed keys */
  // HID mouse

// KeyBoard_Buffer[10] = 0;
// Mouse_Buffer[5] = 0;
  
 // HID Mouse
  mouseHID.id = Mouse_ReportID;
  mouseHID.uButtons.B_LEFT = 0; 
  mouseHID.uButtons.B_MIDDLE = 0; 
  mouseHID.uButtons.B_RIGHT = 0; 
  mouseHID.uButtons.B_UNUSED = 0; 
  mouseHID.x = 0;
  mouseHID.y = 0;
  mouseHID.wheel = 0;
  // HID KeyBoard
  keyboardHID.id = KeyBoard_ReportID;
  keyboardHID.modifiers = 0;
  keyboardHID.key1 = 0;//OEM -- Padding Always 0x00
  keyboardHID.key2 = 0;
  keyboardHID.key3 = 0;
  
//  keyboardHID.key4 = 0;
//  keyboardHID.key5 = 0;
//  keyboardHID.key6 = 0;
//  
  /* Check the pressed keys */
    if(HAL_GPIO_ReadPin(B_USER_GPIO_Port,B_USER_Pin) == GPIO_PIN_SET)
    { 
        keyboardHID.id = 1;
        keyboardHID.modifiers = 0;
        keyboardHID.key1 = 0x00; //
        keyboardHID.key2 = 0x4E; // PgDwn press (0x4E)
        keyboardHID.key3 = 0x00; //d        
//        keyboardHID.key4 = 0x09; //f
//        keyboardHID.key5 = 0x0A; //g
//        keyboardHID.key6 = 0x0B; //h
      //report keycode data -
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&keyboardHID, sizeof(struct keyboardHID_t));
        //HAL_Delay(1);
        for(int i=0; i<1000000; i++);
        keyboardHID.modifiers = 0;
        keyboardHID.key1 = 0x00; 
        keyboardHID.key2 = 0x00; 
        keyboardHID.key3 = 0x00; 
//        keyboardHID.key4 = 0x00; 
//        keyboardHID.key5 = 0x00; 
//        keyboardHID.key6 = 0x00; 	
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&keyboardHID, sizeof(struct keyboardHID_t));
        //HAL_Delay(10);  
        for(int i=0; i<1000000; i++);        
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_SET); 
        //HAL_Delay(5);
    }
    else if(HAL_GPIO_ReadPin(DOWN_JOY_GPIO_Port,DOWN_JOY_Pin) == GPIO_PIN_RESET)
    {
        mouseHID.uButtons.B_RIGHT = 1; 
        mouseHID.x = 0;
        mouseHID.y = -CURSOR_STEP;
        mouseHID.wheel = 0;
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&mouseHID,sizeof(struct mouseHID_t)); // To send usb buffer to PC
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_SET);
        //HAL_Delay(5);     
    }
    else if(HAL_GPIO_ReadPin(LEFT_JOY_GPIO_Port,LEFT_JOY_Pin) == GPIO_PIN_RESET)
    {
        mouseHID.uButtons.B_MIDDLE = 1; 
        mouseHID.x = -CURSOR_STEP;
        mouseHID.y = 0;
        mouseHID.wheel = 0;
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&mouseHID,sizeof(struct mouseHID_t)); // To send usb buffer to PC
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_SET);
        //HAL_Delay(5);        
    }
    else if(HAL_GPIO_ReadPin(RIGHT_JOY_GPIO_Port,RIGHT_JOY_Pin) == GPIO_PIN_RESET)
    {
        mouseHID.x = CURSOR_STEP;
        mouseHID.y = 0;
        mouseHID.wheel = 0;
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&mouseHID,sizeof(struct mouseHID_t)); // To send usb buffer to PC
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_SET);
        //HAL_Delay(5);      
    }
    else if(HAL_GPIO_ReadPin(UP_JOY_GPIO_Port,UP_JOY_Pin) == GPIO_PIN_RESET)
    {
//        mouseHID.buttons = 0; 
        mouseHID.x = 0;
        mouseHID.y = CURSOR_STEP;
        mouseHID.wheel = 0;    
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&mouseHID,sizeof(struct mouseHID_t)); // To send usb buffer to PC
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_SET);

        //HAL_Delay(5);       
    }
    else /* No Joystick key pressed */
    {    
        __NOP();  	
    }
}

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_DFSDM1_Init();
  MX_DFSDM2_Init();
  MX_FMPI2C1_Init();
  MX_FSMC_Init();
  MX_I2S2_Init();
  MX_QUADSPI_Init();
  MX_SDIO_SD_Init();
  MX_UART10_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  BSP_JOY_Init(JOY_MODE_EXTI); //JOY_MODE_EXTI //JOY_MODE_GPIO
  printf("\n\r JOYSTICK Start....\n\r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    HAL_GPIO_TogglePin(LED1_RED_GPIO_Port, LED1_RED_Pin);
//    HAL_Delay(5);
//    HAL_GPIO_TogglePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin);
//    HAL_Delay(50);
    Check_JOYSTICK();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1|RCC_PERIPHCLK_DFSDM1
                              |RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48
                              |RCC_PERIPHCLK_FMPI2C1;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 8;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 4;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLI2SQ;
  PeriphClkInitStruct.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_APB2;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  PeriphClkInitStruct.PLLI2SSelection = RCC_PLLI2SCLKSOURCE_PLLSRC;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  PeriphClkInitStruct.Fmpi2c1ClockSelection = RCC_FMPI2C1CLKSOURCE_APB;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_MANCHESTER_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
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
  * @brief DFSDM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM2_Init(void)
{

  /* USER CODE BEGIN DFSDM2_Init 0 */

  /* USER CODE END DFSDM2_Init 0 */

  /* USER CODE BEGIN DFSDM2_Init 1 */

  /* USER CODE END DFSDM2_Init 1 */
  hdfsdm2_channel1.Instance = DFSDM2_Channel1;
  hdfsdm2_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm2_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm2_channel1.Init.OutputClock.Divider = 2;
  hdfsdm2_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm2_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm2_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm2_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_MANCHESTER_RISING;
  hdfsdm2_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm2_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm2_channel1.Init.Awd.Oversampling = 1;
  hdfsdm2_channel1.Init.Offset = 0;
  hdfsdm2_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm2_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm2_channel7.Instance = DFSDM2_Channel7;
  hdfsdm2_channel7.Init.OutputClock.Activation = ENABLE;
  hdfsdm2_channel7.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm2_channel7.Init.OutputClock.Divider = 2;
  hdfsdm2_channel7.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm2_channel7.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm2_channel7.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm2_channel7.Init.SerialInterface.Type = DFSDM_CHANNEL_MANCHESTER_RISING;
  hdfsdm2_channel7.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm2_channel7.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm2_channel7.Init.Awd.Oversampling = 1;
  hdfsdm2_channel7.Init.Offset = 0;
  hdfsdm2_channel7.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm2_channel7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM2_Init 2 */

  /* USER CODE END DFSDM2_Init 2 */

}

/**
  * @brief FMPI2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMPI2C1_Init(void)
{

  /* USER CODE BEGIN FMPI2C1_Init 0 */

  /* USER CODE END FMPI2C1_Init 0 */

  /* USER CODE BEGIN FMPI2C1_Init 1 */

  /* USER CODE END FMPI2C1_Init 1 */
  hfmpi2c1.Instance = FMPI2C1;
  hfmpi2c1.Init.Timing = 0x00C0EAFF;
  hfmpi2c1.Init.OwnAddress1 = 0;
  hfmpi2c1.Init.AddressingMode = FMPI2C_ADDRESSINGMODE_7BIT;
  hfmpi2c1.Init.DualAddressMode = FMPI2C_DUALADDRESS_DISABLE;
  hfmpi2c1.Init.OwnAddress2 = 0;
  hfmpi2c1.Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK;
  hfmpi2c1.Init.GeneralCallMode = FMPI2C_GENERALCALL_DISABLE;
  hfmpi2c1.Init.NoStretchMode = FMPI2C_NOSTRETCH_DISABLE;
  if (HAL_FMPI2C_Init(&hfmpi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_FMPI2CEx_ConfigAnalogFilter(&hfmpi2c1, FMPI2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMPI2C1_Init 2 */

  /* USER CODE END FMPI2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief UART10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART10_Init(void)
{

  /* USER CODE BEGIN UART10_Init 0 */

  /* USER CODE END UART10_Init 0 */

  /* USER CODE BEGIN UART10_Init 1 */

  /* USER CODE END UART10_Init 1 */
  huart10.Instance = UART10;
  huart10.Init.BaudRate = 115200;
  huart10.Init.WordLength = UART_WORDLENGTH_8B;
  huart10.Init.StopBits = UART_STOPBITS_1;
  huart10.Init.Parity = UART_PARITY_NONE;
  huart10.Init.Mode = UART_MODE_TX_RX;
  huart10.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart10.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART10_Init 2 */

  /* USER CODE END UART10_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED1_RED_Pin|MEMS_LED_Pin|LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CTP_RST_Pin|LCD_TE_Pin|WIFI_WKUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_RED_Pin MEMS_LED_Pin LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LED1_RED_Pin|MEMS_LED_Pin|LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D5_Pin */
  GPIO_InitStruct.Pin = ARD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
  HAL_GPIO_Init(ARD_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEFT_JOY_Pin */
  GPIO_InitStruct.Pin = LEFT_JOY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LEFT_JOY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CTP_INT_Pin */
  GPIO_InitStruct.Pin = CTP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTP_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B_USER_Pin */
  GPIO_InitStruct.Pin = B_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_GREEN_Pin */
  GPIO_InitStruct.Pin = LED2_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_Detect_Pin */
  GPIO_InitStruct.Pin = SD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D13_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_SPI3;
  HAL_GPIO_Init(ARD_D13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CTP_RST_Pin LCD_TE_Pin WIFI_WKUP_Pin */
  GPIO_InitStruct.Pin = LCD_CTP_RST_Pin|LCD_TE_Pin|WIFI_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D10_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UP_JOY_Pin */
  GPIO_InitStruct.Pin = UP_JOY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(UP_JOY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CODEC_INT_Pin */
  GPIO_InitStruct.Pin = CODEC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CODEC_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DOWN_JOY_Pin */
  GPIO_InitStruct.Pin = DOWN_JOY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DOWN_JOY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(ARD_D9_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_DISABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FSMC_WRITE_FIFO_ENABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Perform the SRAM2 memory initialization sequence
  */
  hsram2.Instance = FSMC_NORSRAM_DEVICE;
  hsram2.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram2.Init */
  hsram2.Init.NSBank = FSMC_NORSRAM_BANK3;
  hsram2.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram2.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram2.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram2.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram2.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram2.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram2.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram2.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram2.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram2.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram2.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram2.Init.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram2.Init.WriteFifo = FSMC_WRITE_FIFO_ENABLE;
  hsram2.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram2, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
PCD_HandleTypeDef hpcd;
__IO uint32_t remotewakeupon = 0;
//Check USB Connection

HAL_StatusTypeDef CheckUSB_Connection(void)
{
    if ((((USBD_HandleTypeDef *)hpcd.pData)->dev_remote_wakeup == 1)&&
        (((USBD_HandleTypeDef *)hpcd.pData)->dev_state == USBD_STATE_SUSPENDED))
    {
      if ((&hpcd)->Init.low_power_enable)
      {
        /* Reset SLEEPDEEP bit of Cortex System Control Register */
        SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));  
        
        SystemClockConfig_STOP();
      }
      
      /* Ungate PHY clock */
      __HAL_PCD_UNGATE_PHYCLOCK((&hpcd));
      
      /* Activate Remote wakeup */
      HAL_PCD_ActivateRemoteWakeup((&hpcd));
      
      /* Remote wakeup delay */
      HAL_Delay(10);
      
      /* Disable Remote wakeup */
      HAL_PCD_DeActivateRemoteWakeup((&hpcd));
      
      /* change state to configured */
      ((USBD_HandleTypeDef *)hpcd.pData)->dev_state = USBD_STATE_CONFIGURED;
      
      /* Change remote_wakeup feature to 0*/
      ((USBD_HandleTypeDef *)hpcd.pData)->dev_remote_wakeup=0;
      remotewakeupon = 1;
    }
    else{
      return HAL_OK;
    }
 return HAL_OK;
}
/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    //uint8_t pbuf[4];
    /* Check the pressed keys */
  // HID mouse
 // HID Mouse
  mouseHID.id = Mouse_ReportID;
  mouseHID.uButtons.B_LEFT = 0; 
  mouseHID.uButtons.B_MIDDLE = 0; 
  mouseHID.uButtons.B_RIGHT = 0; 
  mouseHID.uButtons.B_UNUSED = 0; 
  mouseHID.x = 0;
  mouseHID.y = 0;
  mouseHID.wheel = 0;
  
  // HID KeyBoard
  keyboardHID.id = KeyBoard_ReportID;
  keyboardHID.modifiers = 0;
  keyboardHID.key1 = 0;//OEM -- Padding Always 0x00
  keyboardHID.key2 = 0;
  keyboardHID.key3 = 0;
//  keyboardHID.key4 = 0;
//  keyboardHID.key5 = 0;
//  keyboardHID.key6 = 0;
  
  
  /* Check the pressed keys */
    
  if (GPIO_Pin == B_USER_Pin)
    { 
        keyboardHID.key1 = 0x00; //
        keyboardHID.key2 = 0x4E; //PgDwn press (0x4E)
        keyboardHID.key3 = 0x00; //
//        keyboardHID.key4 = 0x09; //f
//        keyboardHID.key5 = 0x0A; //g
//        keyboardHID.key6 = 0x0B; //h
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&keyboardHID, sizeof(struct keyboardHID_t));
        //HAL_Delay(100);
        keyboardHID.key1 = 0x00; 
        keyboardHID.key2 = 0x00; 
        keyboardHID.key3 = 0x00; 
//        keyboardHID.key4 = 0x00; 
//        keyboardHID.key5 = 0x00; 
//        keyboardHID.key6 = 0x00; 	
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&keyboardHID, sizeof(struct keyboardHID_t));
        //HAL_Delay(100);          
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);
        //HAL_Delay(5);
        HAL_Delay(10);
    }
    if (GPIO_Pin == DOWN_JOY_Pin)
    {
        mouseHID.uButtons.B_RIGHT = 1; 
        mouseHID.x = 0;
        mouseHID.y = -CURSOR_STEP;
        mouseHID.wheel = 0;
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&mouseHID,sizeof(struct mouseHID_t)); // To send usb buffer to PC
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_SET);
        HAL_Delay(10);     
    }
    if (GPIO_Pin == LEFT_JOY_Pin)
    {
        mouseHID.uButtons.B_MIDDLE = 1; 
        mouseHID.x = -CURSOR_STEP;
        mouseHID.y = 0;
        mouseHID.wheel = 0;
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&mouseHID,sizeof(struct mouseHID_t)); // To send usb buffer to PC
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_SET);
        HAL_Delay(10);        
    }
    if (GPIO_Pin == RIGHT_JOY_Pin)
    {
        mouseHID.x = CURSOR_STEP;
        mouseHID.y = 0;
        mouseHID.wheel = 0;
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&mouseHID,sizeof(struct mouseHID_t)); // To send usb buffer to PC
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_SET);
        HAL_Delay(10);      
    }
    if (GPIO_Pin == UP_JOY_Pin)
    {
        //mouseHID.Buttons = 0; 
        mouseHID.x = 0;
        mouseHID.y = CURSOR_STEP;
        mouseHID.wheel = 0;    
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);
        USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&mouseHID,sizeof(struct mouseHID_t)); // To send usb buffer to PC
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_SET);
        HAL_Delay(10);       
    }
    else 
    {    
        __NOP();  	//No Joystick key pressed 
    }
    
}

void SystemClockConfig_STOP(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select PLLSAI output as USB clock source */
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 8;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 4;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CK48CLKSOURCE_PLLI2SQ;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
