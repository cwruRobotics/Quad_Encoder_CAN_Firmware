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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    TickFrameType = 0x00,
    ErrorFrameType = 0xFF
} FrameType;

typedef enum {
    None = 0x00
} ErrorType;

typedef struct {
    uint8_t type;
    uint8_t error_type;
    uint32_t ticks;
} Frame;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define FRAME_LEN_MAX 127

#define DEBUG_PRINTS
#ifdef DEBUG_PRINTS
#define DEBUG(message) debug(&huart1, message)
#else
#define DEBUG(message)
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_FilterTypeDef hcan_filter;

uint8_t CAN_ID;
HAL_StatusTypeDef CAN_status;
static uint8_t tx_buffer[FRAME_LEN_MAX];
static uint8_t rx_buffer[FRAME_LEN_MAX];
static uint8_t uCurrentTrim_val;

uint8_t usart_buffer[1024] = {};
HAL_StatusTypeDef USART_status;

_Bool enabled = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void debug(UART_HandleTypeDef* huart, char* message);
_Bool send_CAN_update(CAN_HandleTypeDef* hcan, Frame* frame);
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
  /* USER CODE BEGIN 2 */

  // start CAN
  HAL_CAN_Start(&hcan);

  // check CAN ID from jumpers
  CAN_ID = 0b00000000;
  CAN_ID += (HAL_GPIO_ReadPin(GPIOA, CAN_ID_1_Pin  ) << 0);
  CAN_ID += (HAL_GPIO_ReadPin(GPIOA, CAN_ID_2_Pin  ) << 1);
  CAN_ID += (HAL_GPIO_ReadPin(GPIOA, CAN_ID_4_Pin  ) << 2);
  CAN_ID += (HAL_GPIO_ReadPin(GPIOB, CAN_ID_8_Pin  ) << 3);
  CAN_ID += (HAL_GPIO_ReadPin(GPIOB, CAN_ID_16_Pin ) << 4);
  CAN_ID += (HAL_GPIO_ReadPin(GPIOB, CAN_ID_32_Pin ) << 5);
  CAN_ID += (HAL_GPIO_ReadPin(GPIOB, CAN_ID_64_Pin ) << 6);
  CAN_ID += (HAL_GPIO_ReadPin(GPIOB, CAN_ID_128_Pin) << 7);

  DEBUG("Using CAN ID: \r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t can_frame_available = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
    if(can_frame_available > 0) {
      CAN_RxHeaderTypeDef hddr;
      uint8_t data[8];
      CAN_status = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &hddr, data);

      // read messages
      if(CAN_status == HAL_OK && hddr.RTR == CAN_RTR_REMOTE && hddr.StdId == CAN_ID) {

      }
    }

    if(enabled) {
        // do something
    }

    GPIO_PinState test = HAL_GPIO_ReadPin(GPIOA, CAN_ID_2_Pin);
    if(test) {
      HAL_GPIO_WritePin(GPIOA, CAN_TRAFFIC_LED_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(GPIOA, CAN_TRAFFIC_LED_Pin, GPIO_PIN_RESET);
    }

//    HAL_GPIO_TogglePin(GPIOA, CAN_TRAFFIC_LED_Pin);
    HAL_GPIO_TogglePin(GPIOA, EEPROM_LED_Pin);
    HAL_GPIO_TogglePin(GPIOA, Encoder_LED_Pin);
    DEBUG("Test");
    HAL_Delay(500);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
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
  hcan_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  hcan_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  hcan_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  hcan_filter.FilterIdHigh = 0x0000;
  hcan_filter.FilterIdLow = 0x0000;
  hcan_filter.FilterMaskIdHigh = 0x0000;
  hcan_filter.FilterMaskIdLow = 0x0000;
  hcan_filter.FilterBank = 0;
  hcan_filter.FilterActivation = CAN_FILTER_ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &hcan_filter);
  /* USER CODE END CAN_Init 2 */

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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_SENSE_Pin */
  GPIO_InitStruct.Pin = POWER_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(POWER_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_ID_8_Pin CAN_ID_16_Pin CAN_ID_32_Pin CAN_ID_64_Pin 
                           CAN_ID_128_Pin */
  GPIO_InitStruct.Pin = CAN_ID_8_Pin|CAN_ID_16_Pin|CAN_ID_32_Pin|CAN_ID_64_Pin 
                          |CAN_ID_128_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MEM_SDA_Pin MEM_SCL_Pin */
  GPIO_InitStruct.Pin = MEM_SDA_Pin|MEM_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_X_Pin CAN_A_Pin CAN_B_Pin */
  GPIO_InitStruct.Pin = CAN_X_Pin|CAN_A_Pin|CAN_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void debug(UART_HandleTypeDef* huart, char* message) {
    char c;
    uint16_t len = 0;
    while(c != '\r') {
        c = message[len++];
    }
    HAL_UART_Transmit(huart, (uint8_t*)message, len, 500);
    char newline[] = {'\n'};
    HAL_UART_Transmit(huart, newline, 1, 500);
}

_Bool send_CAN_message(CAN_HandleTypeDef* hcan, Frame* frame) {
    CAN_TxHeaderTypeDef hddr;
    uint32_t mailbox;
    uint8_t data[8];
    hddr.StdId = CAN_ID;
    hddr.IDE = CAN_ID_STD;
    hddr.RTR = CAN_RTR_DATA;
    hddr.DLC = 8;
    data[0] = frame->type;
    data[1] = frame->error_type;
    data[2] = frame->ticks;
    return HAL_CAN_AddTxMessage(hcan, &hddr, data, &mailbox) == HAL_OK;
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
