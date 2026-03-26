/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/tcp.h"
#include <string.h>  // For memcpy
#include <stdlib.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define START_VAL 0xAAAA
#define END_VAL   0xBB

typedef struct __attribute__((packed)) {
    // BLOCK 0 (10 Bytes)
    uint16_t start_marker;
    uint8_t  id_number;
    uint16_t serial_no;
    uint8_t  crc;
    uint32_t packed_time;

    // BLOCK 1 (4 Bytes)
    uint8_t  type;
    union {
        struct { uint16_t input_number; uint8_t status; } digital;
        struct { uint16_t channel_value; uint8_t channel_number; } analog;
    } data;

    // BLOCK 2 (2 Bytes)
    uint8_t  shift_checksum;
    uint8_t  end_marker;
} GenericPacket_t; // data packet structure

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_RECORDS_PER_FRAME 20
#define RECORD_SIZE 16

#define ID_ANALOG  0x01 // id for analog records
#define ID_DIGITAL 0x00 // id for digital records

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

typedef struct __attribute__((packed)) {
    GenericPacket_t records[MAX_RECORDS_PER_FRAME];
} BulkFrame_t;

float last_analog_voltage = -1.0f;
uint8_t digital_state = 0;
uint16_t last_digital_state = 0;
uint16_t last_sent_analog = 0;
uint32_t last_flush_tick = 0;
/* USER CODE END PV */

#define ANALOG_THRESHOLD 205

struct tcp_pcb *test_pcb; // structure to manage the connection
ip_addr_t DestIPaddr;     // variable to store your CPU's IP

volatile uint8_t analog_ready = 0; // flag
volatile uint8_t digital_ready = 0; // flag

__attribute__((section(".RAM_D2"))) uint16_t adc_buffer[1]; // Memory alignment for H7 DMA

BulkFrame_t bulkBuffer __attribute__((section(".LwipSection"), aligned(32)));

uint16_t global_serial = 0;

uint16_t record_count = 0;

uint16_t current_event_idx = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

//err_t send_analog(struct tcp_pcb *tpcb, uint16_t analog_data);
//void send_analog_record(uint16_t val, uint8_t chan_num);
//void send_digital_record(uint16_t pin_num, uint8_t state);
//err_t send_digital(struct tcp_pcb *tpcb, uint8_t digital_data);

void send_digital_record(uint16_t pin_num, uint8_t state);
void send_analog_record(uint16_t val, uint8_t chan_num);
void flush_bulk_buffer(void);
uint8_t calculate_block0_crc(GenericPacket_t *p);
uint8_t calculate_shift_checksum(GenericPacket_t *p);

void Add_Event_To_Frame(uint8_t type, uint16_t id, uint16_t val);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TCP_Client_Init(void) {
    test_pcb = tcp_new();

    if (test_pcb != NULL) {
        IP4_ADDR(&DestIPaddr, 192, 168, 31, 133);
        tcp_connect(test_pcb, &DestIPaddr, 5005, NULL);
    }
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

//  /* Enable I-Cache */
//  SCB_EnableICache();
//
//  /* Enable D-Cache */
//  SCB_EnableDCache();

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
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_LWIP_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(200);

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_buffer, 1);
  HAL_Delay(200);
  HAL_TIM_Base_Start(&htim2); // TIM2 TRGO triggers ADC2

  HAL_TIM_Base_Start_IT(&htim3); // TIM3 IRQ triggers Callback

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  SCB_InvalidateDCache();

//  HAL_Delay(3000);
  TCP_Client_Init();

  while (1)
  {
	  MX_LWIP_Process();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  if (digital_ready && test_pcb != NULL) {
//		  digital_ready = 0;
//		  if (digital_state!= last_digital_state){
//			  send_digital(test_pcb, digital_state);
//			  last_digital_state = digital_state;
//		  }
//
//	  }

//	  if (analog_ready && test_pcb != NULL) {
//		  analog_ready = 0;
//
//		  uint16_t current_analog = adc_buffer[0];
//
//
//		  uint16_t diff = (current_analog > last_analog_voltage) ?
//						  (current_analog - last_analog_voltage) :
//						  (last_analog_voltage - current_analog);
//
//		  if (diff > ANALOG_THRESHOLD) {
//			  send_analog(test_pcb, current_analog);
//			  last_analog_voltage = current_analog;
//		  }
//
//	  }

	  if (test_pcb != NULL && test_pcb->state == ESTABLISHED) {
		  if (digital_ready){
			  digital_ready = 0;
			  uint16_t current_dig = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
			  if (current_dig != last_digital_state) {
				  send_digital_record(3, (uint8_t)current_dig); // Input ID 3
				  last_digital_state = current_dig;
			  }
		  }

		  if (analog_ready){
			  analog_ready=0;
			  uint16_t current_analog = adc_buffer[0];


			  uint16_t diff = (current_analog > last_analog_voltage) ?
							  (current_analog - last_analog_voltage) :
							  (last_analog_voltage - current_analog);

			  if (diff > ANALOG_THRESHOLD) {
				  send_analog_record(current_analog, 1);
				  last_analog_voltage = current_analog;
			  }
		  }

		  if (record_count > 0 && (HAL_GetTick() - last_flush_tick) > 50) {
		          flush_bulk_buffer();
		      }
	  } else{
		  TCP_Client_Init();
		  HAL_Delay(3000);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Simple XOR-based CRC for Block 0
uint8_t calculate_block0_crc(GenericPacket_t *p) {
    uint8_t *ptr = (uint8_t*)p;
    uint8_t crc = 0;
    // Cover first 5 bytes (Start, ID, Serial) - skip CRC byte itself
    for(int i=0; i < 5; i++) crc ^= ptr[i];
    return crc;
}

// Shift Checksum: Sum of all bytes shifted by 1
uint8_t calculate_shift_checksum(GenericPacket_t *p) {
    uint8_t *ptr = (uint8_t*)p;
    uint32_t sum = 0;
    for(int i=0; i < 14; i++) { // Sum everything except Checksum and End
        sum += ptr[i];
    }
    return (uint8_t)((sum >> 1) & 0xFF);
}

void flush_bulk_buffer() {
    if (record_count == 0) return;

    if (test_pcb != NULL && test_pcb->state == ESTABLISHED) {
        uint16_t bytes_to_send = record_count * RECORD_SIZE;

        if (tcp_sndbuf(test_pcb) >= bytes_to_send) {
            tcp_write(test_pcb, &bulkBuffer, bytes_to_send, TCP_WRITE_FLAG_COPY);
            tcp_output(test_pcb);

            record_count = 0; // Empty the bucket
            last_flush_tick = HAL_GetTick();
        }
    }
}

void add_to_bulk_buffer(GenericPacket_t *new_record) {
    // Copy the record into the next available slot in the bucket
    memcpy(&bulkBuffer.records[record_count], new_record, RECORD_SIZE);
    record_count++;

    // If bucket is full, send immediately
    if (record_count >= MAX_RECORDS_PER_FRAME) {
        flush_bulk_buffer();
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        digital_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
        __DMB();
        digital_ready = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC2) {
    	__DMB();
        analog_ready = 1;
    }
}


void prepare_generic_header(GenericPacket_t *p) {
    p->start_marker = 0xAAAA;
    p->id_number = 1;
    p->serial_no = global_serial++;
    p->packed_time = HAL_GetTick();
    p->end_marker = 0xBB;
}

void send_digital_record(uint16_t pin_num, uint8_t state) {
    GenericPacket_t temp;
    prepare_generic_header(&temp);
    temp.type = 0x00;
    temp.data.digital.input_number = pin_num;
    temp.data.digital.status = state;

    // Add validation bytes
    temp.crc = calculate_block0_crc(&temp);
    temp.shift_checksum = calculate_shift_checksum(&temp);

    add_to_bulk_buffer(&temp);
}

void send_analog_record(uint16_t val, uint8_t chan_num) {
    GenericPacket_t temp;
    prepare_generic_header(&temp);
    temp.type = 0x01;
    temp.data.analog.channel_value = val;
    temp.data.analog.channel_number = chan_num;

    temp.crc = calculate_block0_crc(&temp);
    temp.shift_checksum = calculate_shift_checksum(&temp);

    add_to_bulk_buffer(&temp);
}


/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
