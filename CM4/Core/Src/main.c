/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "openamp.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RPMSG_SERVICE_NAME              "openamp_demo"
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

static volatile int message_received;
volatile unsigned int received_data;
static struct rpmsg_endpoint rp_endpoint;
char data[100];
int i = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
//static void MX_FDCAN1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv);
unsigned int receive_message(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//FDCAN_TxHeaderTypeDef   TxHeader;
//FDCAN_RxHeaderTypeDef   RxHeader;
//uint8_t  TxData[12];
//uint8_t  RxData[12];
//int indx = 0;


//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
//  {
//    /* Retreive Rx messages from RX FIFO0 */
//    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
//    {
//    /* Reception Error */
//    Error_Handler();
//    }
//    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
//    {
//      /* Notification Error */
//      Error_Handler();
//    }
//  }
//}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
//  MX_FDCAN1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  int32_t status = 0;

  /* Initialize the mailbox use notify the other core on new message */
  MAILBOX_Init();

  /* Initialize OpenAmp and libmetal libraries */
  if (MX_OPENAMP_Init(RPMSG_REMOTE, NULL)!= HAL_OK)
    Error_Handler();

  /* Create an endpoint for rmpsg communication */
  status = OPENAMP_create_endpoint(&rp_endpoint, RPMSG_SERVICE_NAME, RPMSG_ADDR_ANY, rpmsg_recv_callback, NULL);
  if (status < 0)
  {
    Error_Handler();
  }

//  if(HAL_FDCAN_Start(&hfdcan1)!= HAL_OK)
//   {
// 	  Error_Handler();
//   }
//   if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
//   {
//     /* Notification Error */
//     Error_Handler();
//   }
//
//     TxHeader.Identifier = 0x11;
//     TxHeader.IdType = FDCAN_STANDARD_ID;
//     TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//     TxHeader.DataLength = FDCAN_DLC_BYTES_12;
//     TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//     TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
//     TxHeader.FDFormat = FDCAN_FD_CAN;
//     TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//     TxHeader.MessageMarker = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  for(int i=0;i<12;i++)
//	  {
		  receive_message();

//		  TxData[i] = data;

		  sprintf(data,"%u\n\r",received_data);

		  HAL_UART_Transmit(&huart3, data, strlen(data), 100);
//	  }
//
//	  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData)!= HAL_OK)
//	  {
//	   Error_Handler();
//	  }
//
//	  HAL_Delay (1000);
//
//	  i++;
  }
  /* USER CODE END 3 */
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_FDCAN1_Init(void)
//{
//
//  /* USER CODE BEGIN FDCAN1_Init 0 */
//
//  /* USER CODE END FDCAN1_Init 0 */
//
//  /* USER CODE BEGIN FDCAN1_Init 1 */
//
//  /* USER CODE END FDCAN1_Init 1 */
//  hfdcan1.Instance = FDCAN1;
//  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
//  hfdcan1.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
//  hfdcan1.Init.AutoRetransmission = ENABLE;
//  hfdcan1.Init.TransmitPause = DISABLE;
//  hfdcan1.Init.ProtocolException = DISABLE;
//  hfdcan1.Init.NominalPrescaler = 1;
//  hfdcan1.Init.NominalSyncJumpWidth = 13;
//  hfdcan1.Init.NominalTimeSeg1 = 86;
//  hfdcan1.Init.NominalTimeSeg2 = 13;
//  hfdcan1.Init.DataPrescaler = 25;
//  hfdcan1.Init.DataSyncJumpWidth = 1;
//  hfdcan1.Init.DataTimeSeg1 = 2;
//  hfdcan1.Init.DataTimeSeg2 = 1;
//  hfdcan1.Init.MessageRAMOffset = 0;
//  hfdcan1.Init.StdFiltersNbr = 1;
//  hfdcan1.Init.ExtFiltersNbr = 0;
//  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
//  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_12;
//  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
//  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
//  hfdcan1.Init.RxBuffersNbr = 0;
//  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
//  hfdcan1.Init.TxEventsNbr = 0;
//  hfdcan1.Init.TxBuffersNbr = 0;
//  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
//  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
//  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_12;
//  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN FDCAN1_Init 2 */
//
//  FDCAN_FilterTypeDef sFilterConfig;
//
//  sFilterConfig.IdType = FDCAN_STANDARD_ID;
//  sFilterConfig.FilterIndex = 0;
//  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
//  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//  sFilterConfig.FilterID1 = 0x11;
//  sFilterConfig.FilterID2 = 0x11;
//  sFilterConfig.RxBufferIndex = 0;
//  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
//  {
//    /* Filter configuration Error */
//    Error_Handler();
//  }
//
//  /* USER CODE END FDCAN1_Init 2 */
//
//}

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
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data,
               size_t len, uint32_t src, void *priv)
{
  received_data = *((unsigned int *) data);
  message_received=1;

  return 0;
}

unsigned int receive_message(void)
{
  while (message_received == 0)
  {
    OPENAMP_check_for_message();
  }
  message_received = 0;

  return 0;
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
