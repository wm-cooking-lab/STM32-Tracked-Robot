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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	// robot immobile
	ETAT_ZERO,

	// avancer, trois vitesses possibles
	ETAT_A1,
	ETAT_A2,
	ETAT_A3,

	// reculer, trois vitesses possibles
	ETAT_R1,
	ETAT_R2,
	ETAT_R3,

	// pivoter vers la gauche, trois vitesses possibles
	ETAT_G1,
	ETAT_G2,
	ETAT_G3,

	// pivoter vers la droite, trois vitesses possibles
	ETAT_D1,
	ETAT_D2,
	ETAT_D3
} Etat;

typedef enum
{
	DIR_GAUCHE,
	DIR_DROITE
} Direction;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GAUCHE TIM_CHANNEL_4
#define DROITE TIM_CHANNEL_1

#define HTIM_GAUCHE &htim3
#define HTIM_DROITE &htim4

#define DIRECTION_ARRIERE 0
#define DIRECTION_AVANT 1

#define CARACT_AVANT 'A'
#define CARACT_ARRIERE 'R'
#define CARACT_GAUCHE 'G'
#define CARACT_DROITE 'D'
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define abs(x) ((x<0)?-(x):x)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Constantes à paramétrer
const int FREQ_VERIF_BATT_MS = 5000;
const int DELAI_REBOND_MS = 500;

// Constantes déterminées expérimentalement
const int K_p_G = 30;
const int K_p_D = 30;
const int K_i_G = 10;
const int K_i_D = 10;
const int V_MAX = 170/5;
const int REF_TOP_D = 6144;
const int REF_TOP_G = 6170;
const int REF_MUL = 100;

// Vitesse de base qui sera multipliée par 1, 2 ou 3
const char V_0 = 10; // cm/s

// Messages à envoyer au terminal en temps voulu
const unsigned char message_batterie[] = "Batterie faible.\r\n";
const unsigned char message_go[] = "C'est parti !\r\n";
const unsigned char message_stop[] = "Arrêt d'urgence !\r\n";

// Variables globales
volatile char go = 0;
volatile char T_batt = 0;
volatile uint8_t caract_recu;

// Variables d'état
volatile Etat etat_present = ETAT_ZERO;
volatile Etat etat_futur = ETAT_ZERO;

// Vitesses en cm/s
volatile char consigne_G = 0;
volatile char consigne_D = 0;
volatile unsigned int vitesse_G_cms, vitesse_D_cms;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
unsigned int calculer_PWM(int vitesse, Direction dir);
void reset_PWM(void);
int diff_top(int vitesse, Direction dir);
int cm_s(int diff_top, Direction dir);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	// Démarrage des timers
	HAL_TIM_Base_Start_IT(&htim6); // T_batt
	HAL_TIM_PWM_Start(&htim2, GAUCHE);
	HAL_TIM_PWM_Start(&htim2, DROITE);
	HAL_TIM_Encoder_Start(HTIM_GAUCHE, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(HTIM_DROITE, TIM_CHANNEL_ALL);
	HAL_UART_Receive_IT(&huart3, &caract_recu, 1); // Armement pour premier caractère

	reset_PWM(); // Moteurs en position de sécurité
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(T_batt > FREQ_VERIF_BATT_MS/100)
		{
			T_batt = 0;
			// On éteint la LED, elle sera rallumée immédiatement par le watchdog en cas de batterie faible
			HAL_GPIO_WritePin(Alert_Batt_GPIO_Port, Alert_Batt_Pin, GPIO_PIN_RESET);
			HAL_ADC_Start(&hadc1); // Le watchdog se débrouille ensuite
		}
		if(go) // Fonctionnement normal
		{
			// Modification des consignes en fonction de l'état présent
			if(etat_present != etat_futur)
			{
				// Consommation de l'état futur
				// Afin de ne pas boucler inutilement sur la même consigne
				etat_present = etat_futur;
				switch(etat_present)
				{
					case ETAT_ZERO:
						consigne_G = 0;
						consigne_D = 0;
						break;

					case ETAT_A1:
						consigne_G = V_0;
						consigne_D = V_0;
						HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_AVANT);
						HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_AVANT);
						break;

					case ETAT_A2:
						consigne_G = 2*V_0;
						consigne_D = 2*V_0;
						HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_AVANT);
						HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_AVANT);
						break;

					case ETAT_A3:
						consigne_G = 3*V_0;
						consigne_D = 3*V_0;
						HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_AVANT);
						HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_AVANT);
						break;

					case ETAT_R1:
						consigne_G = 1*V_0;
						consigne_D = 1*V_0;
						HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_ARRIERE);
						HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_ARRIERE);
						break;

					case ETAT_R2:
						consigne_G = 2*V_0;
						consigne_D = 2*V_0;
						HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_ARRIERE);
						HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_ARRIERE);
						break;

					case ETAT_R3:
						consigne_G = 3*V_0;
						consigne_D = 3*V_0;
						HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_ARRIERE);
						HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_ARRIERE);
						break;

					case ETAT_G1:
						consigne_G = V_0;
						consigne_D = V_0;
						HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_ARRIERE);
						HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_AVANT);
						break;

					case ETAT_G2:
						consigne_G = 2*V_0;
						consigne_D = 2*V_0;
						HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_ARRIERE);
						HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_AVANT);
						break;

					case ETAT_G3:
						consigne_G = 3*V_0;
						consigne_D = 3*V_0;
						HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_ARRIERE);
						HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_AVANT);
						break;

					case ETAT_D1:
						consigne_G = V_0;
						consigne_D = V_0;
						HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_AVANT);
						HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_ARRIERE);
						break;

					case ETAT_D2:
						consigne_G = 2*V_0;
						consigne_D = 2*V_0;
						HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_AVANT);
						HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_ARRIERE);
						break;

					case ETAT_D3:
						consigne_G = 3*V_0;
						consigne_D = 3*V_0;
						HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_AVANT);
						HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_ARRIERE);
						break;
				}
			}
		}
		else // Arrêt d'urgence
		{
			// Moteurs en position de sécurité
			reset_PWM();
		}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_14;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 4095;
  AnalogWDGConfig.LowThreshold = 3723;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 160000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 123-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65041-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Alert_Batt_GPIO_Port, Alert_Batt_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BP_Start_Pin */
  GPIO_InitStruct.Pin = BP_Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BP_Start_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Alert_Batt_Pin */
  GPIO_InitStruct.Pin = Alert_Batt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Alert_Batt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Direction_Gauche_Pin */
  GPIO_InitStruct.Pin = Direction_Gauche_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Direction_Gauche_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Direction_Droite_Pin */
  GPIO_InitStruct.Pin = Direction_Droite_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Direction_Droite_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Callabck analog watchdog
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
	// La première mesure déclenche toujours le callback
	static char premiere_mesure_batterie = 1;
	if(hadc == &hadc1) // Vbatt est sur l'ADC1
	{
		if(premiere_mesure_batterie) premiere_mesure_batterie = 0;
		else
		{
			// Allumage de la LED d'alerte
			HAL_GPIO_WritePin(Alert_Batt_GPIO_Port, Alert_Batt_Pin, GPIO_PIN_SET);
			HAL_UART_Transmit(&huart2, (unsigned char*)message_batterie, sizeof(message_batterie), HAL_MAX_DELAY);
		}
	}
}

// Callback bouton start
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t dernier_appui = 0;
	if((GPIO_Pin == BP_Start_Pin) && (HAL_GetTick() - dernier_appui > DELAI_REBOND_MS))
	{
		dernier_appui = HAL_GetTick(); // Mise à jour du dernier moment où le bouton a été pressé
		if(go) // arrêt d'urgence
		{
			go = 0;
			HAL_UART_Transmit(&huart2, (unsigned char*)message_stop, sizeof(message_stop), HAL_MAX_DELAY);
		}
		else // départ
		{
			go = 1;
			HAL_UART_Transmit(&huart2, (unsigned char*)message_go, sizeof(message_go), HAL_MAX_DELAY);
		}
	}
}

// Callback timer6
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Variables statiques (la valeur est conservée d'un appel à l'autre)
	// G/D : gauche/droite
	// 0/1 : passé/présent
	static int top_0G = 0;
	static int top_0D = 0;
	static int top_1G = 0;
	static int top_1D = 0;

	// Sommes des erreurs
	static int somme_G = 0;
	static int somme_D = 0;

	if(htim == &htim6)
	{
		// Pour la batterie
		T_batt++;

		// Transition depuis l'appel précédent
		top_0G = top_1G;
		top_0D = top_1D;

		// Acquisition depuis les capteurs
		top_1G = __HAL_TIM_GET_COUNTER(HTIM_GAUCHE);
		top_1D = __HAL_TIM_GET_COUNTER(HTIM_DROITE);

		unsigned int vitesse_G = abs(top_1G - top_0G);
		unsigned int vitesse_D = abs(top_1D - top_0D);
		if (vitesse_G > 65000 || vitesse_D > 65000) return; //dépassement

		// Uniquement pour visualisation SWV
		vitesse_G_cms = cm_s(vitesse_G, DIR_GAUCHE);
		vitesse_D_cms = cm_s(vitesse_D, DIR_DROITE);

		// Calculs d'asservissement
		int epsilon_G = diff_top(consigne_G, DIR_GAUCHE) - vitesse_G;
		int epsilon_D = diff_top(consigne_D, DIR_DROITE) - vitesse_D;

		somme_G += epsilon_G;
		somme_D += epsilon_D;

		int commande_G = (K_p_G*epsilon_G/REF_MUL + K_i_G*somme_G/REF_MUL);
		int commande_D = (K_p_D*epsilon_D/REF_MUL + K_i_D*somme_D/REF_MUL);

		// Conversion de la commande (top/delta) en valeur de PWM
		unsigned int PWM_G = calculer_PWM(commande_G, DIR_GAUCHE);
		unsigned int PWM_D = calculer_PWM(commande_D, DIR_DROITE);

		// Envoi des commandes
		__HAL_TIM_SET_COMPARE(&htim2, GAUCHE, PWM_G);
		__HAL_TIM_SET_COMPARE(&htim2, DROITE, PWM_D);
	}
}

// Callback Bluetooth UART3
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		// Machine d'états
		switch(etat_present)
		{
			case ETAT_ZERO:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_A1;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_R1;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_D1;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_G1;
						break;
				}
				break;

			case ETAT_A1:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_A2;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_ZERO;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_D1;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_G1;
						break;
				}
				break;

			case ETAT_A2:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_A3;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_A1;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_D1;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_G1;
						break;
				}
				break;

			case ETAT_A3:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_A3;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_A2;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_D1;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_G1;
						break;
				}
				break;

			case ETAT_R1:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_ZERO;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_R2;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_D1;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_G1;
						break;
				}
				break;

			case ETAT_R2:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_R1;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_R3;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_D1;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_G1;
						break;
				}
				break;

			case ETAT_R3:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_R2;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_R3;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_D1;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_G1;
						break;
				}
				break;

			case ETAT_G1:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_A1;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_R1;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_ZERO;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_G2;
						break;
				}
				break;

			case ETAT_G2:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_A1;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_R1;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_G1;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_G3;
						break;
				}
				break;

			case ETAT_G3:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_A1;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_R1;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_G2;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_G3;
						break;
				}
				break;

			case ETAT_D1:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_A1;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_R1;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_D2;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_ZERO;
						break;
				}
				break;

			case ETAT_D2:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_A1;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_R1;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_D3;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_D1;
						break;
				}
				break;

			case ETAT_D3:
				switch(caract_recu)
				{
					case CARACT_AVANT:
						etat_futur = ETAT_A1;
						break;

					case CARACT_ARRIERE:
						etat_futur = ETAT_R1;
						break;

					case CARACT_DROITE:
						etat_futur = ETAT_D3;
						break;

					case CARACT_GAUCHE:
						etat_futur = ETAT_D2;
						break;
				}
				break;
		}
	}
	HAL_UART_Receive_IT(&huart3, &caract_recu, 1); // Réarmement pour la suite
	return;
}

unsigned int calculer_PWM(int vitesse, Direction dir) // top/delta
{
	uint32_t arr_timer = TIM2->ARR;
	long valeur = arr_timer*vitesse;
	/* Si toutes les opérations sont sur la même ligne,
	   les calculs intermédiaires sont faits sur 32 bits
	   et le résultat dépasse */
	valeur /= diff_top(V_MAX, dir);

	unsigned int r = valeur;

	// Écrêtage de la valeur aux limites du PWM
	if (valeur < 0) r = 0;
	if (valeur > (long) arr_timer) r = arr_timer;

	return r;
}

int diff_top(int vitesse, Direction dir) // cm/s
{
	long r = (dir==DIR_GAUCHE) ? REF_TOP_G : REF_TOP_D;
	r *= vitesse;
	r /= V_MAX;
	r /= REF_MUL;
	return (int)r;
}

int cm_s(int diff_top, Direction dir)
{
	long r = V_MAX * REF_MUL;
	r *= diff_top;
	r /= (dir==DIR_GAUCHE) ? REF_TOP_G : REF_TOP_D;
	return (int)r;
}

void reset_PWM(void)
{
	// Initialisation des PWM à zéro
	__HAL_TIM_SET_COMPARE(&htim2, GAUCHE, calculer_PWM(0, DIR_GAUCHE));
	__HAL_TIM_SET_COMPARE(&htim2, DROITE, calculer_PWM(0, DIR_DROITE));

	// Initialisation des directions en avant
	HAL_GPIO_WritePin(Direction_Gauche_GPIO_Port, Direction_Gauche_Pin, DIRECTION_AVANT);
	HAL_GPIO_WritePin(Direction_Droite_GPIO_Port, Direction_Droite_Pin, DIRECTION_AVANT);
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
