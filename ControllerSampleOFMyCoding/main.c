/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Components/otm8009a/otm8009a.h"
#include "stm32469i_discovery_sdram.h"
#include "stm32469i_discovery_qspi.h"
#include "stdio.h"
#include "convert.h"
#include "NRF24L01.h"
#include <string.h>
#include "RP_LIDAR_A1M8.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* DISPLAY */
#define LCD_ORIENTATION_LANDSCAPE 0x01
/* NRF24L0+ */
#define NRF24L01_TIME_INTERVAL 50
/* UART */
#define UART_RX_DATA_SIZE 127
/* LSM303 */
#define LSM303_ACC_RESOLUTION 2.0
#define G 9.81

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

DSI_HandleTypeDef hdsi;

I2C_HandleTypeDef hi2c1;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for TouchGFXTask */
osThreadId_t TouchGFXTaskHandle;
const osThreadAttr_t TouchGFXTask_attributes = {
  .name = "TouchGFXTask",
  .stack_size = 8192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* Task variables */
uint8_t MAIN_TASK_STATE = 0;

/* UART variables */
uint8_t IS_UART_SYNC = 0;
uint8_t UART_RX_COUNTER;

/* LIDAR variables */
uint8_t Quality[360]; //360
uint8_t InversedStartFlag[360];
uint8_t StartFlag[360];
uint8_t CheckBit[360];
float Angle[360];
float Distance[360];

uint8_t LIDARDataToProcess[80];
uint16_t LIDAR_Measurement_pointer = 0;

/* Vehicle state variables */
uint8_t vehicle_state = 0;
uint8_t currentlyGoingForwardFlag;
uint8_t currentlyGoingBackwardFlag;
uint8_t resetButtonStateOnF469i;

/* Vehicle max speed variable */
uint8_t max_speed = 25;
/* Max PWM value, set via slider */
extern uint8_t max_PWM;

/* Vehicle current PWM level */
uint8_t currentPWMLevel;

/* Vehicle angular velocity */
float leftEngineAngularVelocity;
float rightEngineAngularVelocity;

float leftEngineAngularVelocitySTABLE;
float rightEngineAngularVelocitySTABLE;

/* Vehicle travel distance */
float leftWheelTravelDistance;
float rightWheelTravelDistance;

/* ESP32 variables */
uint8_t firstDataByte;
uint8_t lastDataByte;
uint8_t ESP32_UART_RX_BUFFER2[UART_RX_DATA_SIZE];

/* nRF24 variables */
uint8_t TxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
uint8_t nRF24DataToTransmit[6];
uint16_t NRF24_CONVERTED_DATA_READY_TO_USE_IN_VIEW[4];

/* HCSR04 variables */
uint16_t HCSR04Measurements[6]; // 0 - FR, 1 - FC, 2 - FL, 3 - BL, 4 - BC, 5 - BR

/* Limit switch variables */
uint8_t LIMIT_SWITCH_FRONT_LEFT; 
uint8_t LIMIT_SWITCH_FRONT_RIGHT; 
uint8_t LIMIT_SWITCH_BACK_LEFT; 
uint8_t LIMIT_SWITCH_BACK_RIGHT; 

/* Gyroscope variables */
uint8_t rawGyroscopeData[6];
uint16_t convertedGyroscopeData[3];
uint16_t convertedGyroscopeDataSTABLE[3];

/* Accelerometer variables */
uint8_t rawAccelerometerData[6];
int16_t convertedAccelerometerData[3];
float accelerationSIunits[3]; // [m/s2]
float accelerationSIunitsSTABLE[3];

/* Magnetometer variables */
uint8_t rawMagnetometerData[6];
int16_t convertedMagnetometerData[3];
float magnetometerSIunits[3];
float magnetometerSIunitsSTABLE[3];

/* Temperature variables */
uint8_t rawTemperatureData[2];
int16_t convertedTemperatureData;
float temperatureCelsiusUnits;
float temperatureCelsiusUnitsSTABLE;

/* HAL variables */
uint32_t tickstart;

/* Joystick variables */
uint16_t joystick[2];
uint8_t joystickConvertedData[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA2D_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_CRC_Init(void);
void TouchGFX_Task(void *argument);
void MainTaskFunction(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_DMA2D_Init();
  MX_DSIHOST_DSI_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_QUADSPI_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
  MX_TouchGFX_Init();
  /* USER CODE BEGIN 2 */

  /* Begining of time counting in main loop */
  tickstart = HAL_GetTick();

  /* ADC Contionus mode conversion init */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) joystick, 2);

  /* nRF24 init */
  NRF24_Init();
  NRF24_TxMode(TxAddress, 100);

  /* Receive first packet of ESP32 data via UART DMA */
  HAL_UART_Receive_DMA(&huart6, ESP32_UART_RX_BUFFER2, UART_RX_DATA_SIZE);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of TouchGFXTask */
  TouchGFXTaskHandle = osThreadNew(TouchGFX_Task, NULL, &TouchGFXTask_attributes);

  /* creation of MainTask */
  MainTaskHandle = osThreadNew(MainTaskFunction, NULL, &MainTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */
  /* Activate XRES active low */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_RESET);

  HAL_Delay(20); /* wait 20 ms */

  /* Desactivate XRES */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET);

  /* Wait for 10ms after releasing XRES before sending commands */
  HAL_Delay(10);
  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_LPCmdTypeDef LPCmd = {0};
  DSI_CmdCfgTypeDef CmdCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  PLLInit.PLLNDIV = 125;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV2;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV1;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 28;
  PhyTimings.ClockLaneLP2HSTime = 33;
  PhyTimings.DataLaneHS2LPTime = 15;
  PhyTimings.DataLaneLP2HSTime = 25;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_ENABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_ENABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_ENABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_ENABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_ENABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_ENABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_ENABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_ENABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_ENABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_ENABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_ENABLE;
  LPCmd.LPMaxReadPacket = DSI_LP_MRDP_ENABLE;
  LPCmd.AcknowledgeRequest = DSI_ACKNOWLEDGE_ENABLE;
  if (HAL_DSI_ConfigCommand(&hdsi, &LPCmd) != HAL_OK)
  {
    Error_Handler();
  }
  CmdCfg.VirtualChannelID = 0;
  CmdCfg.ColorCoding = DSI_RGB565;
  CmdCfg.CommandSize = 200;
  CmdCfg.TearingEffectSource = DSI_TE_EXTERNAL;
  CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  CmdCfg.HSPolarity = DSI_HSYNC_ACTIVE_LOW;
  CmdCfg.VSPolarity = DSI_VSYNC_ACTIVE_LOW;
  CmdCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.VSyncPol = DSI_VSYNC_FALLING;
  CmdCfg.AutomaticRefresh = DSI_AR_DISABLE;
  CmdCfg.TEAcknowledgeRequest = DSI_TE_ACKNOWLEDGE_ENABLE;
  if (HAL_DSI_ConfigAdaptedCommandMode(&hdsi, &CmdCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */

  /* USER CODE END DSIHOST_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  HAL_I2C_DeInit(&hi2c1);
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 1;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 2;
  hltdc.Init.AccumulatedVBP = 2;
  hltdc.Init.AccumulatedActiveW = 202;
  hltdc.Init.AccumulatedActiveH = 482;
  hltdc.Init.TotalWidth = 203;
  hltdc.Init.TotalHeigh = 483;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 200;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 480;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0xC0000000;
  pLayerCfg.ImageWidth = 200;
  pLayerCfg.ImageHeight = 480;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */
  __HAL_LTDC_DISABLE(&hltdc);
  DSI_LPCmdTypeDef LPCmd;

  HAL_DSI_Start(&hdsi);
  OTM8009A_Init(OTM8009A_FORMAT_RBG565, LCD_ORIENTATION_LANDSCAPE);
  HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_DISPOFF, 0x00);

  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_DISABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_DISABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_DISABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_DISABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_DISABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_DISABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_DISABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_DISABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_DISABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_DISABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_DISABLE;
  HAL_DSI_ConfigCommand(&hdsi, &LPCmd);

  HAL_LTDC_SetPitch(&hltdc, 800, 0);
  __HAL_LTDC_ENABLE(&hltdc);
  /* USER CODE END LTDC_Init 2 */

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
  hqspi.Init.ClockPrescaler = 1;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 27;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_5_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */
  BSP_QSPI_DeInit();
  if(BSP_QSPI_Init() != QSPI_OK)
  {
    Error_Handler();
  }
  if(BSP_QSPI_EnableMemoryMappedMode() != QSPI_OK)
  {
    Error_Handler();
  }
  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_2;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  BSP_SDRAM_DeInit();
  if(BSP_SDRAM_Init() != SDRAM_OK)
  {
    Error_Handler();
  }
  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RENDER_TIME_GPIO_Port, RENDER_TIME_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VSYNC_FREQ_GPIO_Port, VSYNC_FREQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NRF24_CE_Pin|NRF24_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PK3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : RENDER_TIME_Pin */
  GPIO_InitStruct.Pin = RENDER_TIME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RENDER_TIME_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VSYNC_FREQ_Pin */
  GPIO_InitStruct.Pin = VSYNC_FREQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(VSYNC_FREQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PH7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF24_CE_Pin NRF24_CSN_Pin */
  GPIO_InitStruct.Pin = NRF24_CE_Pin|NRF24_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART6)
	{
		/* Check if UART packet is sync */
		if(ESP32_UART_RX_BUFFER2[0] == 42 && ESP32_UART_RX_BUFFER2[UART_RX_DATA_SIZE - 1] == 35){
			IS_UART_SYNC = 1;
		} else {
			IS_UART_SYNC = 0;
		}

		/* Copy LIDAR data to LIDAR buffer */
		memcpy(LIDARDataToProcess, &ESP32_UART_RX_BUFFER2[13], 80);

		HAL_UART_Receive_DMA(&huart6, ESP32_UART_RX_BUFFER2, UART_RX_DATA_SIZE);

		UART_RX_COUNTER++;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_TouchGFX_Task */
/**
  * @brief  Function implementing the TouchGFXTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TouchGFX_Task */
__weak void TouchGFX_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MainTaskFunction */
/**
* @brief Function implementing the MainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MainTaskFunction */
void MainTaskFunction(void *argument)
{
  /* USER CODE BEGIN MainTaskFunction */
  /* Infinite loop */
  for(;;)
  {

	  switch(MAIN_TASK_STATE){

	  case 0:
	  {
	  /* --- CONVERT LIDAR DATA PACKET TASK BEGIN --- */

	  /* Convert LIDAR data */
	  uint8_t LIDARDataToProcessPointer = 0;

	  for(int i = 0; i < 16; i++){

	  uint8_t temp = 0;
	  uint16_t temp16 = 0;
	  uint16_t temp16_2 = 0;

	  Quality[LIDAR_Measurement_pointer] = LIDARDataToProcess[LIDARDataToProcessPointer] >> 2;

	  temp = LIDARDataToProcess[LIDARDataToProcessPointer];
	  temp = temp << 6;
	  InversedStartFlag[LIDAR_Measurement_pointer] = temp >> 7;

	  temp = 0;

	  temp = LIDARDataToProcess[LIDARDataToProcessPointer];
	  temp = temp << 7;
	  StartFlag[LIDAR_Measurement_pointer] = temp >> 7;

	  temp = 0;

	  temp = LIDARDataToProcess[LIDARDataToProcessPointer+1] << 7;
	  CheckBit[LIDAR_Measurement_pointer] = temp >> 7;

	  temp = 0;
	  temp16 = LIDARDataToProcess[LIDARDataToProcessPointer+1] >> 1;
	  temp16_2 = LIDARDataToProcess[LIDARDataToProcessPointer+2];
	  temp16_2 = temp16_2 << 7;
	  temp16_2 |= temp16;
	  Angle[LIDAR_Measurement_pointer] = temp16_2 / 64.0;

	  temp16 = 0;
	  temp16_2 = 0;
	  temp16 = LIDARDataToProcess[LIDARDataToProcessPointer+4];
	  temp16 = temp16 << 8;
	  temp16_2 = LIDARDataToProcess[LIDARDataToProcessPointer+3];
	  temp16 |= temp16_2;
	  Distance[LIDAR_Measurement_pointer] = temp16 / 4.0;

	  LIDAR_Measurement_pointer++;
	  LIDARDataToProcessPointer += 5;

	  if(LIDAR_Measurement_pointer == 360){
		  LIDAR_Measurement_pointer = 0;
	  }

	  }

	  /* --- CONVERT LIDAR DATA PACKET TASK END --- */
	  MAIN_TASK_STATE++;
	  break;
	  }

	  case 1:
	  {
	  /* --- PREPARE nRF24L0+ DATA PACKET TASK BEGIN --- */

	  /* Convert joystick data to uint8_t table */
	  convertU16tableToU8table(joystick, joystickConvertedData, 2, 4);

	  /* Prepare data to send via nRF24L0+ */
	  for(int i = 0; i < 4; i++){
	  nRF24DataToTransmit[i] = joystickConvertedData[i];
	  }
	  nRF24DataToTransmit[4] = max_PWM;
	  nRF24DataToTransmit[5] = resetButtonStateOnF469i;

	  /* --- PREPARE nRF24L0+ DATA PACKET TASK END --- */
	  MAIN_TASK_STATE++;
	  break;
	  }

	  case 2:
	  {
	  /* --- SEND nRF24L0+ DATA PACKET TASK BEGIN --- */

	  /* Set trigger to initiate every NRF24L01_TIME_INTERVAL [miliseconds] */
	  if((HAL_GetTick() - tickstart) > NRF24L01_TIME_INTERVAL)
	  {
	  		 if(NRF24_Transmit(nRF24DataToTransmit) == 1)
	  		 {
	  			/* Could be LED blinking or other action  */
	  		 }
	  		 tickstart += NRF24L01_TIME_INTERVAL;
	  }

	  /* --- SEND nRF24L0+ DATA PACKET TASK END --- */
	  MAIN_TASK_STATE++;
	  break;
	  }

	  case 3:
	  {
	  /* --- CONVERT HCSR04 DATA PACKET TASK BEGIN --- */
	  uint16_t value = 0; // variable created just to hold temporary converted value
	  uint8_t converted_data_index = 0;
	  uint8_t first_data_index = 1;
	  uint8_t second_data_index = 2;

      for(int i = 0; i < 6; i++){
	  value = (uint16_t) ESP32_UART_RX_BUFFER2[first_data_index];
	  value = value << 8;
	  value |= (uint16_t) ESP32_UART_RX_BUFFER2[second_data_index];
	  HCSR04Measurements[converted_data_index] = value;
	  first_data_index += 2;
	  second_data_index += 2;
	  converted_data_index++;
	  value = 0;
      }
	  /* --- CONVERT HCSR04 DATA PACKET TASK END --- */
	  MAIN_TASK_STATE++;
	  break;
	  }

	  case 4:
	  {
	  /* --- CONVERT ENCODERS DATA PACKET TASK BEGIN --- */

	  /* Assign and covnvert travel distance */
	  float travelDistance100[2];
	  uint16_t value;

	  value = (uint16_t) ESP32_UART_RX_BUFFER2[103];
	  value = value << 8;
	  value |= (uint16_t) ESP32_UART_RX_BUFFER2[104];
	  travelDistance100[0] = value;
	  value = 0;

	  value = (uint16_t) ESP32_UART_RX_BUFFER2[105];
	  value = value << 8;
	  value |= (uint16_t) ESP32_UART_RX_BUFFER2[106];
	  travelDistance100[1] = value;
	  value = 0;

	  leftWheelTravelDistance = travelDistance100[0] / 100;
	  rightWheelTravelDistance = travelDistance100[1] / 100;

	  /* Assign movement flags */
	  currentlyGoingForwardFlag = ESP32_UART_RX_BUFFER2[107];
	  currentlyGoingBackwardFlag = ESP32_UART_RX_BUFFER2[108];

	  /* Assign current PWM level */
	  currentPWMLevel = ESP32_UART_RX_BUFFER2[109];

	  /* Assign and convert angular velocity */
	  uint16_t angularVelocity10[2];

	  value = (uint16_t) ESP32_UART_RX_BUFFER2[110];
	  value = value << 8;
	  value |= (uint16_t) ESP32_UART_RX_BUFFER2[111];
	  angularVelocity10[0] = value;
	  value = 0;

	  value = (uint16_t) ESP32_UART_RX_BUFFER2[112];
	  value = value << 8;
	  value |= (uint16_t) ESP32_UART_RX_BUFFER2[113];
	  angularVelocity10[1] = value;
	  value = 0;

	  /* Make it again one digit precise */
	  leftEngineAngularVelocity = angularVelocity10[0];
	  rightEngineAngularVelocity = angularVelocity10[1];

	  leftEngineAngularVelocity = leftEngineAngularVelocity / 10;
	  rightEngineAngularVelocity = rightEngineAngularVelocity / 10;

	  leftEngineAngularVelocitySTABLE = leftEngineAngularVelocity;
	  rightEngineAngularVelocitySTABLE = rightEngineAngularVelocity;

	  /* --- CONVERT ENCODERS DATA PACKET TASK END --- */
	  MAIN_TASK_STATE++;
	  break;
	  }

	  case 5:
	  {
	  /* --- CONVERT LIMIT SWITCH DATA PACKET TASK BEGIN --- */

	  /* Assign limit switch data */
	  LIMIT_SWITCH_FRONT_LEFT = ESP32_UART_RX_BUFFER2[97];
	  LIMIT_SWITCH_FRONT_RIGHT = ESP32_UART_RX_BUFFER2[98];
	  LIMIT_SWITCH_BACK_LEFT = ESP32_UART_RX_BUFFER2[99];
	  LIMIT_SWITCH_BACK_RIGHT = ESP32_UART_RX_BUFFER2[100];

	  /* --- CONVERT LIMIT SWITCH DATA PACKET TASK END --- */
	  MAIN_TASK_STATE++;
	  break;
	  }

	  case 6:
	  {
	  /* --- CONVERT VEHICLE STATE DATA PACKET TASK BEGIN --- */

	  /* Assign vehicle state and max speed */
	  vehicle_state = ESP32_UART_RX_BUFFER2[101];
	  max_speed = ESP32_UART_RX_BUFFER2[102];

	  /* --- CONVERT VEHICLE STATE DATA PACKET TASK END --- */
	  MAIN_TASK_STATE++;
	  break;
	  }

	  case 7:
	  {
	  /* --- CONVERT MAGNETOMETER AND TEMPERATURE DATA PACKET TASK BEGIN --- */

	  /* Assign magnetometer data */
	  uint8_t magnetometer_table_pointer = 0;
	  for(int i = 114; i < 120; i++){

		 rawMagnetometerData[magnetometer_table_pointer] = ESP32_UART_RX_BUFFER2[i];
		 magnetometer_table_pointer++;
	  }
	  /* Convert raw magnetometer data */
	  convertedMagnetometerData[0] = ((rawMagnetometerData[1] << 8) | rawMagnetometerData[0]);
	  convertedMagnetometerData[1] = ((rawMagnetometerData[3] << 8) | rawMagnetometerData[2]);
	  convertedMagnetometerData[2] = ((rawMagnetometerData[5] << 8) | rawMagnetometerData[4]);

	  /* Convert to SI units (gauss) */
	  magnetometerSIunits[0] = (convertedMagnetometerData[0] * 49.152f) / 32678.0f;
	  magnetometerSIunits[1] = (convertedMagnetometerData[1] * 49.152f) / 32678.0f;
	  magnetometerSIunits[2] = (convertedMagnetometerData[2] * 49.152f) / 32678.0f;

	  /* Assign converted magnetometer data */
	  magnetometerSIunitsSTABLE[0] = magnetometerSIunits[0];
	  magnetometerSIunitsSTABLE[1] = magnetometerSIunits[1];
	  magnetometerSIunitsSTABLE[2] = magnetometerSIunits[2];

	  /* Assign temperature data */
	  rawTemperatureData[0] = ESP32_UART_RX_BUFFER2[93];
	  rawTemperatureData[1] = ESP32_UART_RX_BUFFER2[94];

	  /* Convert raw temperature data */
	  convertedTemperatureData = ((int16_t) ((rawTemperatureData[1] << 8) | rawTemperatureData[0]));

	  /* Convert to Celsius units */
	  temperatureCelsiusUnits = (((float) convertedTemperatureData / 64.0f) / 4.0f) + 25.0f;

	  /* Assign converted temperature data */
	  temperatureCelsiusUnitsSTABLE = temperatureCelsiusUnits;

	  /* --- CONVERT MAGNETOMETER AND TEMPERATURE DATA PACKET TASK END --- */
	  MAIN_TASK_STATE++;
	  break;

	  }

	  case 8:
	  {
	  /* --- CONVERT ACCELEROMETER DATA PACKET TASK BEGIN --- */

	  /* Assign raw accelerometer data */
      uint8_t accelerometer_table_pointer = 0;
	  for(int i = 120; i < 126; i++){

		 rawAccelerometerData[accelerometer_table_pointer] = ESP32_UART_RX_BUFFER2[i];
		 accelerometer_table_pointer++;
	  }

	  convertedAccelerometerData[0] = ((rawAccelerometerData[1] << 8) | rawAccelerometerData[0]);
	  convertedAccelerometerData[1] = ((rawAccelerometerData[3] << 8) | rawAccelerometerData[2]);
	  convertedAccelerometerData[2] = ((rawAccelerometerData[5] << 8) | rawAccelerometerData[4]);

	  /* Convert data to SI uints [m/s2] */
	  accelerationSIunits[0] = (convertedAccelerometerData[0] * 2.0f) / 32678.0f;
	  accelerationSIunits[1] = (convertedAccelerometerData[1] * 2.0f) / 32678.0f;
	  accelerationSIunits[2] = (convertedAccelerometerData[2] * 2.0f) / 32678.0f;

	  /* Assign converted accelerometer data */
	  accelerationSIunitsSTABLE[0] = accelerationSIunits[0];
	  accelerationSIunitsSTABLE[1] = accelerationSIunits[1];
	  accelerationSIunitsSTABLE[2] = accelerationSIunits[2];

	  /* --- CONVERT ACCELEROMETER DATA PACKET TASK END --- */

	  MAIN_TASK_STATE++;
	  break;
	  }

	  case 9:
	  {
	  /* --- ASSIGN CONVERTED DATA PACKET TASK BEGIN --- */

	  /* Assign first data byte */
	  firstDataByte = ESP32_UART_RX_BUFFER2[0];

	  /* Assign last UART data byte */
	  lastDataByte = (uint16_t) ESP32_UART_RX_BUFFER2[UART_RX_DATA_SIZE - 1];

	  /* Get nRF24L0+ data ready to display on the screen */
	  NRF24_CONVERTED_DATA_READY_TO_USE_IN_VIEW[0] = joystick[0];
	  NRF24_CONVERTED_DATA_READY_TO_USE_IN_VIEW[1] = joystick[1];
	  NRF24_CONVERTED_DATA_READY_TO_USE_IN_VIEW[2] = nRF24DataToTransmit[4];
	  NRF24_CONVERTED_DATA_READY_TO_USE_IN_VIEW[3] = nRF24DataToTransmit[5];

	  /* --- ASSIGN ASSIGN CONVERTED DATA PACKET TASK END --- */
	  MAIN_TASK_STATE++;
	  break;
	  }

	  default:
	  MAIN_TASK_STATE = 0;
	  break;
	  }

	  }
    osDelay(1);
  /* USER CODE END MainTaskFunction */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
