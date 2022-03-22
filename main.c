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
#include <stdio.h>
#include "lcd1602.h"
#include "control_hardware.h"
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
ADC_HandleTypeDef hadc;

/* USER CODE BEGIN PV */
const GPIO_TypeDef *LedPort[5] = {LED1_GPIO_Port,LED2_GPIO_Port,
		LED3_GPIO_Port,LED4_GPIO_Port,LED5_GPIO_Port};

const uint16_t LedPin[5] = {LED1_Pin, LED2_Pin, LED3_Pin,
		LED4_Pin, LED5_Pin
};

char lcd_buff[40];
uint32_t led_ring_data[12];
int buzDoneFlag = 1, overHeat = 0;
char modeName[6][10] = {"OVER HEAT","SAFE LOCK","OFF      "
		,"ON(NONE) ","AUTO ADJ ","ON       "};

int fireTemp = 0;
int mainTemp = 20;
int warnningTemp = 20;
int autoModeFlag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t read_adc(uint8_t x)
{
  uint16_t adc[2];

  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, 1000);
  adc[0]=HAL_ADC_GetValue(&hadc);
  HAL_ADC_PollForConversion(&hadc, 1000);
  adc[1]=HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);

  return adc[x];
}

uint8_t Get_SW(void){
	int swA = HAL_GPIO_ReadPin(SW_A_GPIO_Port,SW_A_Pin);
	int swB = HAL_GPIO_ReadPin(SW_B_GPIO_Port,SW_B_Pin);
	int swAuto = HAL_GPIO_ReadPin(SW_AUTO_GPIO_Port,SW_AUTO_Pin);

	static int oldA = 1;
	static int oldB = 1;
	static int oldAuto = 1;
	int value = 0;

	if(!swA & oldA) value = 10;
	if(!swB & oldB) value = 20;
	if(!swAuto & oldAuto) value = 50;

	oldA = swA;
	oldB = swB;
	oldAuto = swAuto;

	return value;
}


uint8_t State_Set(){
	static int lockFlag = 1;

	if( mainTemp > 300 | overHeat){   //과열 모드
		overHeat = 1;
		return 0;
	}
	buzDoneFlag = 0;

	if (!HAL_GPIO_ReadPin(SW_LOCK_GPIO_Port, SW_LOCK_Pin)) {
		fireTemp = 0;
		lockFlag = 1;
		return 1;
	} else {
		if (HAL_GPIO_ReadPin(SW_ON_GPIO_Port, SW_ON_Pin))
			lockFlag = 0;
	}
	if (lockFlag == 0) {
		if (!HAL_GPIO_ReadPin(SW_ON_GPIO_Port, SW_ON_Pin)){
			if(read_adc(0) > 3900){
				if(autoModeFlag) return 4;
				return 5;
			}
			fireTemp = 1;
			return 3;
		}
		fireTemp = 0;
		return 2;
	}



}

void Buz_Action(int ver){
	int nowTick = HAL_GetTick();
	static int fir = 1;
	static int systick = 0;

	if(fir){
		fir = 0;
		systick = nowTick;
	}
	if (ver){
		if (nowTick - systick < 100)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
		else if (nowTick - systick < 200)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
		else if (nowTick - systick < 300)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
		else if (nowTick - systick < 400)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
		else if (nowTick - systick > 1000)
			fir = 1;
	}
	if(!ver){
		if (nowTick - systick < 100)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
		else if (nowTick - systick < 200)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
		else if (nowTick - systick < 300)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
		else if (nowTick - systick < 400)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
		else if (nowTick - systick < 500)
					HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
		else if (nowTick - systick < 600)
					HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
		else if (nowTick - systick > 1000){
			fir = 1;
			buzDoneFlag = 1;
		}

	}
}

void Temp_Add(int fireTemp){
	int nowTick = HAL_GetTick();
	static int fir = 1;
	static int systick = 0;

	if(fir){
		fir = 0;
		systick = nowTick;
	}

	if(nowTick - systick > 1000 - fireTemp*100 & mainTemp < 999){
		fir = 1;
		mainTemp++;
	}
}

void Temp_Decade(int Temp){
	int nowTick = HAL_GetTick();
	static int fir = 1;
	static int systick = 0;
	static int delayNum = 0;
	const int TempDate[7] = {10,15,20,40,100,200,300};
	const int delayTerm[8] = {2900,2200,1600,1100,700,400,200,100};

	if(fir){
		fir = 0;
		systick = nowTick;
	}
	for(int i = 0; i < 7; i++){
		if(Temp-20 < TempDate[i]){
			delayNum = delayTerm[i];
			break;
		}
		delayNum = delayTerm[7];
	}

	if(nowTick - systick > delayNum){
		fir = 1;
		if(mainTemp > 20) mainTemp--;
	}

}

void Auto_ADJ(void){
	const int autoTemp_V[5] = {80,100,140,180,220};
	static int a = 0;
	int adcCDS = read_adc(0);
	int adcRES = read_adc(1);

	for(int i = 0; i < 5; i++){
		if(adcRES < (4100 / 5) * (i+1)){
			HAL_GPIO_WritePin(LedPort[a], LedPin[a], GPIO_PIN_RESET);
			a = i;
			if(autoTemp_V[a]-mainTemp > 1) fireTemp = 9;
			else if( autoTemp_V[a]-mainTemp < -1) fireTemp =1;
			break;
		}
	}
	for(int i = 0; i < 5; i++){
		if( a != i)
			HAL_GPIO_WritePin(LedPort[i], LedPin[i], GPIO_PIN_SET);
	}



}

void On_ADJ(void){
	static int a = 0;
	int adcCDS = read_adc(0);
	int adcRES = read_adc(1);

	for(int i = 0; i < 9; i++){
		if(adcRES < (4100 / 9) * (i+1)){
			fireTemp = i+1;
			break;
		}
	}
	if(adcRES >= 4095) fireTemp = 9;
}

void LedRingAction(int fireNum){
	const uint32_t Fire_led_date[10][12] ={
			{0,},{13,0,0,0,13,0,0,0,13,0,0,0},
			{76,0,0,0,76,0,0,0,76,0,0,0},
			{255,0,0,0,255,0,0,0,255,0,0,0},
			{255,0,13,0,255,0,13,0,255,0,13,0},
			{255,0,76,0,255,0,76,0,255,0,76,0},
			{255,0,255,0,255,0,255,0,255,0,255,0},
			{255,13,255,13,255,13,255,13,255,13,255,13},
			{255,76,255,76,255,76,255,76,255,76,255,76},
			{255,255,255,255,255,255,255,255,255,255,255,255}
	};

	for(int i = 0; i < 12; i++){
		led_ring_data[i] = Fire_led_date[fireNum][i];
	}
	HAL_Delay(1);
	led_ring_update(led_ring_data);

}

void IdleMain(void){
	static int fir = 1;
	static int displayFire;
	int key = Get_SW();
	int nowTick = HAL_GetTick();
	int sysTick;
	int stateNum = State_Set();
	if(fir){
		fir = 0;
		sysTick = nowTick;
	}

	Temp_Decade(mainTemp);
	if(fireTemp != 0)Temp_Add(fireTemp);

	lcd_gotoxy(1,1);
	sprintf(lcd_buff,"TEMP:%03d%cC  %c:%01d",mainTemp,0xdf,1,fireTemp);
	lcd_puts(lcd_buff);

	lcd_gotoxy(1,2);
	sprintf(lcd_buff,"[%s][%03d]",modeName[stateNum], warnningTemp);
	lcd_puts(lcd_buff);

	if(key == 10 & warnningTemp < 280 & stateNum > 3) warnningTemp += 20;
	if(key == 20 & warnningTemp > 20 & stateNum > 3) warnningTemp -= 20;
	if(key == 50)autoModeFlag = !autoModeFlag;


	if(stateNum != 4)LED1_GPIO_Port -> BSRR = LedPin[0] | LedPin[1] | LedPin[2] | LedPin[3] | LedPin[4];


	if(stateNum == 4)
		Auto_ADJ();
	else if(stateNum == 5){
		On_ADJ();
	}

	if(warnningTemp != 20 & mainTemp > warnningTemp & stateNum == 5)
		Buz_Action(1);
	else HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);

	if(overHeat){
		displayFire = 0;
		fireTemp = 0;
		while(!buzDoneFlag){
			Buz_Action(0);
		}
	}

	LedRingAction(displayFire);
	if(stateNum == 0 & mainTemp < 150 & HAL_GPIO_ReadPin(SW_ON_GPIO_Port,SW_ON_Pin))
		overHeat = 0;

	if(displayFire != fireTemp & nowTick - sysTick >= 100){
		fir = 1;
		if(displayFire > fireTemp) displayFire -= 1;
		else if(displayFire < fireTemp) displayFire += 1;
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
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  LcdInit();
  led_ring_update(led_ring_data);

  lcd_puts("\fSmart Gas Range\n             XXX");
  lcd_cgram(1,0);
  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  IdleMain();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RING_Pin|BUZZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|LCD_RW_Pin|LCD_EN_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW_A_Pin SW_B_Pin */
  GPIO_InitStruct.Pin = SW_A_Pin|SW_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_ON_Pin SW_AUTO_Pin SW_LOCK_Pin */
  GPIO_InitStruct.Pin = SW_ON_Pin|SW_AUTO_Pin|SW_LOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RING_Pin */
  GPIO_InitStruct.Pin = LED_RING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED_RING_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin
                           LED5_Pin BUZZ_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin|BUZZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_RW_Pin LCD_EN_Pin LCD_D4_Pin
                           LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_RW_Pin|LCD_EN_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
