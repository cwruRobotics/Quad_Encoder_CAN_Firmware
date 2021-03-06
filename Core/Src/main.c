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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>


#include "CAN_api.h"
#include "types.h"
#include "encoder.h"
#include "eeprom.h"
#include "util.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//#define DEBUG_CAN_MESSAGE
//#define NO_5V5
#define ENCODING 2
#pragma message("Using encoding: X" XSTR(ENCODING))

#define UART_PRINT_TIMEOUT 200 // ms
#define UART_BUFFER_SIZE 1024


//#define DEBUG_PRINTS
#ifdef DEBUG_PRINTS
#define DEBUG(message) debug(&huart1, message)
#else
#define DEBUG(message)
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t current_time_ms;
bool already_panicked;

uint32_t uart_size;
uint8_t uart_buffer[UART_BUFFER_SIZE] = {};
HAL_StatusTypeDef UART_status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void panic();
void debug(UART_HandleTypeDef* huart, char* message);
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  // enable microsecond delays
  DWT_Init();

  // set already_panicked to false
  already_panicked = false;

  // start CAN
  CAN_status = HAL_CAN_Start(&hcan);
  if(CAN_status != HAL_OK) {
      // bad
      DEBUG("Could not start CAN transmission.");
  }

  // get CAN ID from jumpers
  CAN_id = 0b00000000;
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_1_GPIO_Port   ,   CAN_ID_1_Pin   ) << 0u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_2_GPIO_Port   ,   CAN_ID_2_Pin   ) << 1u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_4_GPIO_Port   ,   CAN_ID_4_Pin   ) << 2u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_8_GPIO_Port   ,   CAN_ID_8_Pin   ) << 3u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_16_GPIO_Port  ,   CAN_ID_16_Pin  ) << 4u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_32_GPIO_Port  ,   CAN_ID_32_Pin  ) << 5u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_64_GPIO_Port  ,   CAN_ID_64_Pin  ) << 6u);
  CAN_id += (HAL_GPIO_ReadPin(CAN_ID_128_GPIO_Port ,   CAN_ID_128_Pin ) << 7u);

  // transmit CAN ID for debug
  #ifdef DEBUG_PRINTS
  uart_size = sprintf((char*) uart_buffer, "CAN ID selected: %d \t", CAN_id);
  UART_status = HAL_UART_Transmit(&huart1, uart_buffer, uart_size, UART_PRINT_TIMEOUT);
  #endif

  // declare buffer for reading EEPROM values (4 bytes wide)
  const uint8_t eeprom_buffer_length = 4;
  uint8_t buffer[eeprom_buffer_length];
  memset(buffer, 0, eeprom_buffer_length);

  // disable interrupts while reading eeprom to ensure correct timing
  __disable_irq();

  // TODO(Ben): Does the byte order need to be reversed when reading from EEPROM?

  // read encoder ticks
  i2c_receive_continuous_bytes(ENCODER_TICKS_LOCATION, buffer, 4);
  memcpy_v(&encoder_count, buffer, 4);

  // read encoder polarity
  i2c_receive_byte(ENCODER_POLARITY_LOCATION, buffer);
  memcpy_v(&encoder_inverted, buffer, 1);

  // read encoder feedback period
  i2c_receive_continuous_bytes(FEEDBACK_PERIOD_LOCATION, buffer, 2);
//  memcpy_v(&CAN_outgoing_message_period_ms, buffer, 2);

  // reenable interrupts
  __enable_irq();

  #ifdef DEBUG_PRINTS
  uart_size = sprintf((char*) uart_buffer, "Starting up with %l ticks \t", encoder_count);
  UART_status = HAL_UART_Transmit(&huart1, uart_buffer, uart_size, UART_PRINT_TIMEOUT);
  #endif
  {
    GPIO_InitTypeDef initTypeDef;
    #if(ENCODING == 1)
        initTypeDef.Pin =  ENC_A_Pin;
        initTypeDef.Mode = GPIO_MODE_IT_RISING;
        initTypeDef.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(ENC_A_GPIO_Port, &initTypeDef);

        initTypeDef.Pin =  ENC_B_Pin;
        initTypeDef.Mode = GPIO_MODE_INPUT;
        initTypeDef.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(ENC_B_GPIO_Port, &initTypeDef);
    #elif(ENCODING == 2)
        initTypeDef.Pin = ENC_A_Pin;
        initTypeDef.Mode = GPIO_MODE_IT_RISING_FALLING;
        initTypeDef.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(ENC_A_GPIO_Port, &initTypeDef);

        initTypeDef.Pin = ENC_B_Pin;
        initTypeDef.Mode = GPIO_MODE_INPUT;
        initTypeDef.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(ENC_B_GPIO_Port, &initTypeDef);
    #elif(ENCODING == 4)
        initTypeDef.Pin =  ENC_A_Pin;
        initTypeDef.Mode = GPIO_MODE_IT_RISING_FALLING;
        initTypeDef.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(ENC_A_GPIO_Port, &initTypeDef);

        initTypeDef.Pin =  ENC_B_Pin;
        initTypeDef.Mode = GPIO_MODE_IT_RISING_FALLING;
        initTypeDef.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(ENC_B_GPIO_Port, &initTypeDef);
    #else
        #error
    #endif
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//    __disable_irq();
//    i2c_receive_continuous_bytes(ENCODER_TICKS_LOCATION, buffer, 4);
//    memcpy_v(&encoder_count, buffer, 4);
//    __enable_irq();

    // first, check if power is ok
    bool power_ok;
    #ifndef NO_5V5
        power_ok = HAL_GPIO_ReadPin(POWER_SENSE_GPIO_Port, POWER_SENSE_Pin);
    #else
        power_ok = true;
    #endif

    // if it's not ok, freak out and save ticks to EEPROM
    if(!power_ok) {
        // freak out if we haven't already
        if(!already_panicked) {
            // disable irq so we dont waste cpu cycles or mess up i2c timing
            __disable_irq();
            panic();
        }

        // set this to prevent panic running again
        already_panicked = true;

        // wait for impending doom
        continue;
    } else {
        // reenable irq
        __enable_irq();

        // set this here just in case we're on the brink of death but come back
        already_panicked = false;
    }

    // get current time
    current_time_ms = HAL_GetTick();

    // check if we have any CAN messages to read
    uint32_t can_frame_available = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
    if(can_frame_available > 0) {
      // initialize
      CAN_RxHeaderTypeDef hddr;
      uint8_t data[8];

      // get data from message
      CAN_status = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &hddr, data);
      CAN_connected = true;

      // read messages
      if(CAN_status == HAL_OK && hddr.StdId == CAN_id) {
          // check if message is less than 2 bytes long because bad
          if(hddr.DLC < 1) {
//              HAL_GPIO_TogglePin(Encoder_LED_GPIO_Port, Encoder_LED_Pin);
              // not an error, but we won't do anything about it
          } else {
              // check for requests that we recognize
              switch (data[0]) {
                  // read first bit in the message as the request type
                  case REQUEST_RESET_TICKS:
                      // set ticks to 0
                      reset_encoder_count();

                      // transmit a message if in debug mode
                      #ifdef DEBUG_PRINTS
                      uart_size = sprintf((char *) uart_buffer, "Encoder ticks set to %l", encoder_count);
                      UART_status = HAL_UART_Transmit(&huart1, uart_buffer, uart_size, UART_PRINT_TIMEOUT);
                      #endif

                      break;
                  case REQUEST_SET_TICKS:
                      // check if CAN message is long enough to store the correct information
                      if(hddr.DLC < 5) {
                          // bad CAN frame, just leave the ticks the same and throw an error
                          // encoder_ticks = 0; // probably should'nt do this
                          // tell the user that they messed up
                          DEBUG("Bad CAN frame. Set ticks frame requires 5 bytes of data. Setting ticks to 0!");
                          break;
                      }
                      // read 32 bit signed int
                      memcpy_v(&encoder_count, &data[1], 4);
                      // transmit a message if in debug mode
                      #ifdef DEBUG_PRINTS
                      uart_size = sprintf((char*) uart_buffer, "Encoder ticks set to %l \r", encoder_count);
                      UART_status = HAL_UART_Transmit(&huart1, uart_buffer, uart_size, UART_PRINT_TIMEOUT);
                      #endif
                      break;
                  case REQUEST_SET_POLARITY:
                      if(hddr.DLC < 2) {
                          // bad frame
                          break;
                      }
                      // read 1 bit bool
                      memcpy_v(&encoder_inverted, &data[1], 1);

                      #ifdef DEBUG_PRINTS
                      uart_size = sprintf((char*) uart_buffer, "Encoder inverted set to %l \r", encoder_inverted);
                      UART_status = HAL_UART_Transmit(&huart1, uart_buffer, uart_size, UART_PRINT_TIMEOUT);
                      #endif

                      break;
                  case REQUEST_SET_FEEDBACK_PERIOD:
                      if(hddr.DLC < 3) {
                          // bad frame
                          break;
                      }
                      // read 16 bit unsigned integer
                      memcpy(&CAN_outgoing_message_period_ms, &data[1], 2);
                      break;
                  case REQUEST_NONE:
                  default:
                      // do nothing
                      break;
              }
          }
      }
    }


    // check if it's time to send a frame
    if(CAN_should_update(current_time_ms)) {
        Frame frame;

        // fill data in frame
        frame.type = FRAME_TYPE_TICKS;
        frame.error_type = ERROR_TYPE_NONE;
        frame.ticks = encoder_count;

        #ifdef DEBUG_CAN_MESSAGE
        frame.type = FRAME_TYPE_DEBUG;
        #endif

        bool can_send_success = send_CAN_update(&hcan, &frame, CAN_id);

        HAL_GPIO_WritePin(
          CAN_TRAFFIC_LED_GPIO_Port, CAN_TRAFFIC_LED_Pin,
          (can_send_success && CAN_connected)
        );
    }



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef can_filter;

  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterBank = 0;
  can_filter.FilterActivation = CAN_FILTER_ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &can_filter);
  /* USER CODE END CAN_Init 2 */

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

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CAN_TRAFFIC_LED_Pin|EEPROM_LED_Pin|Encoder_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MEM_SDA_Pin|MEM_SCL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CAN_ID_1_Pin CAN_ID_2_Pin CAN_ID_4_Pin */
  GPIO_InitStruct.Pin = CAN_ID_1_Pin|CAN_ID_2_Pin|CAN_ID_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_TRAFFIC_LED_Pin EEPROM_LED_Pin Encoder_LED_Pin */
  GPIO_InitStruct.Pin = CAN_TRAFFIC_LED_Pin|EEPROM_LED_Pin|Encoder_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_SENSE_Pin */
  GPIO_InitStruct.Pin = POWER_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(POWER_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_ID_8_Pin CAN_ID_16_Pin CAN_ID_32_Pin CAN_ID_64_Pin 
                           CAN_ID_128_Pin ENC_B_Pin */
  GPIO_InitStruct.Pin = CAN_ID_8_Pin|CAN_ID_16_Pin|CAN_ID_32_Pin|CAN_ID_64_Pin 
                          |CAN_ID_128_Pin|ENC_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MEM_SDA_Pin MEM_SCL_Pin */
  GPIO_InitStruct.Pin = MEM_SDA_Pin|MEM_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_X_Pin */
  GPIO_InitStruct.Pin = ENC_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ENC_X_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_A_Pin */
  GPIO_InitStruct.Pin = ENC_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ENC_A_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

// Called when power to the module drops. Save settings and encoder ticks
void panic() {
    // turn off leds
    HAL_GPIO_WritePin(CAN_TRAFFIC_LED_GPIO_Port, CAN_TRAFFIC_LED_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin(Encoder_LED_GPIO_Port    , Encoder_LED_Pin    , GPIO_PIN_RESET );
    HAL_GPIO_WritePin(EEPROM_LED_GPIO_Port     , EEPROM_LED_Pin     , GPIO_PIN_RESET );

    // save stuff to EEPROM
    uint8_t buffer[4];
    memset(buffer, 0, 4);
    memcpy_v(buffer, &encoder_count, 4);
    i2c_send_continuous_bytes(ENCODER_TICKS_LOCATION, buffer, 4);

    memset(buffer, 0, 4);
    memcpy_v(buffer, &encoder_inverted, 2);
    i2c_send_continuous_bytes(ENCODER_POLARITY_LOCATION, buffer, 2);

    i2c_send_byte(FEEDBACK_PERIOD_LOCATION, encoder_inverted);
}

void debug(UART_HandleTypeDef* huart, char* message) {
    char c;
    uint16_t len = 0;
    while(c != '\r') {
        c = message[len++];
    }
    HAL_UART_Transmit(huart, (uint8_t*)message, len, UART_PRINT_TIMEOUT);
    char newline[] = {'\n'};
    HAL_UART_Transmit(huart, newline, 1, UART_PRINT_TIMEOUT);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == ENC_A_Pin || GPIO_Pin == ENC_B_Pin) {
      bool a_high = HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin);
      bool b_high = HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin);
      increment_encoder_from_GPIO(a_high, b_high);
      HAL_GPIO_TogglePin(Encoder_LED_GPIO_Port, Encoder_LED_Pin);
  } else if (GPIO_Pin == ENC_X_Pin) {
    // figure out what this does and why we need it
  }
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

  // this very bad

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
