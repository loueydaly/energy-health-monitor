/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "EnergyProcessing.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float v;
    float i;
} SampleMsg_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NOMINAL_V    230.0f
#define MAX_I        16.0f
#define V_TOLERANCE  23.0f
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
osMessageQueueId_t SampleQueueHandle;
osThreadId_t       ProcessingTaskHandle;
osThreadId_t       CommTaskHandle;

EnergyMetrics g_metrics;
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
CAN_TxHeaderTypeDef tx_header;
uint32_t tx_mailbox;

const osThreadAttr_t procTask_attr = { .name = "ProcTask", .priority = (osPriority_t) osPriorityAboveNormal, .stack_size = 512 * 4 };
const osThreadAttr_t commTask_attr = { .name = "CommTask", .priority = (osPriority_t) osPriorityNormal, .stack_size = 256 * 4 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
void StartProcessingTask(void *argument);
void StartCommTask(void *argument);
float calculate_quality(const EnergyMetrics *m, uint8_t *qv, uint8_t *qpf, uint8_t *qi, uint8_t *grade);

/* USER CODE BEGIN 0 */
/* Logic for vApplicationIdleHook is handled in freertos.c to avoid duplicate definition errors */
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();

  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  osKernelInitialize();

  SampleQueueHandle = osMessageQueueNew(10, sizeof(SampleMsg_t), NULL);
  ProcessingTaskHandle = osThreadNew(StartProcessingTask, NULL, &procTask_attr);
  CommTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attr);

  osKernelStart();
  /* USER CODE END 2 */

  while (1) {}
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        if (rxHeader.StdId == 0x100) {
            SampleMsg_t msg;
            int16_t v_raw = (int16_t)(((uint16_t)rxData[0] << 8) | rxData[1]);
            int16_t i_raw = (int16_t)(((uint16_t)rxData[2] << 8) | rxData[3]);
            msg.v = (float)v_raw / 100.0f;
            msg.i = (float)i_raw / 100.0f;
            osMessageQueuePut(SampleQueueHandle, &msg, 0U, 0U);
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    }
}

void StartProcessingTask(void *argument) {
    SampleMsg_t sample;
    energy_init();
    for(;;) {
        if (osMessageQueueGet(SampleQueueHandle, &sample, NULL, osWaitForever) == osOK) {
            energy_add_sample(sample.v, sample.i);
            if (energy_is_buffer_full()) {
                g_metrics = energy_get_metrics();
                energy_check_all_anomalies(&g_metrics);
                osThreadFlagsSet(CommTaskHandle, 0x01);
            }
        }
    }
}

void StartCommTask(void *argument) {
    uint8_t tx_data[8];
    tx_header.StdId = 0x200;
    tx_header.DLC = 8;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    uint8_t q_v, q_pf, q_i, grade;

    for(;;) {
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        float score = calculate_quality(&g_metrics, &q_v, &q_pf, &q_i, &grade);
        uint16_t q_scaled = (uint16_t)roundf(score * 100.0f);
        tx_data[0] = (q_scaled >> 8) & 0xFF;
        tx_data[1] = q_scaled & 0xFF;
        tx_data[2] = q_v;
        tx_data[3] = q_pf;
        tx_data[4] = q_i;
        tx_data[5] = grade;
        tx_data[6] = (uint8_t)roundf(fminf(fabsf(g_metrics.PF), 1.0f) * 100.0f);
        uint8_t flags = 0x01;
        if (g_metrics.I_rms > MAX_I) flags |= 0x02;
        if (fabsf(g_metrics.V_rms - NOMINAL_V) > V_TOLERANCE) flags |= 0x04;
        tx_data[7] = flags;
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
        }
    }
}

float calculate_quality(const EnergyMetrics *m, uint8_t *qv, uint8_t *qpf, uint8_t *qi, uint8_t *grade) {
    float v_diff = fabsf(m->V_rms - NOMINAL_V);
    float q_v = fmaxf(0.0f, 100.0f - ((v_diff / V_TOLERANCE) * 100.0f));
    float q_pf = fabsf(m->PF) * 100.0f;
    float q_i = (m->I_rms > MAX_I) ? 0.0f : 100.0f;
    float score = (0.40f * q_v) + (0.40f * q_pf) + (0.20f * q_i);
    *grade = (score >= 90.0f) ? 3 : (score >= 75.0f ? 2 : (score >= 60.0f ? 1 : 0));
    *qv = (uint8_t)q_v; *qpf = (uint8_t)q_pf; *qi = (uint8_t)q_i;
    return score;
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  HAL_PWREx_EnableOverDrive();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  HAL_CAN_Init(&hcan1);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
/* USER CODE END 4 */
