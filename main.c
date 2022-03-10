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
#define led_ring_rgb(red, green, blue) ((uint32_t)((uint32_t) blue | (uint32_t) red << 16 | (uint32_t) green << 8))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

/* USER CODE BEGIN PV */
char lcd_buff[40];
char modeName[6][10] = {"OVER HEAT","SAFE LOCK","OFF      ","ON(NONE) ","AUTO ADJ ","ON       "};
uint32_t led_ring_data[12];
uint16_t FireTemp = 0, MainTemp = 20;
uint8_t autoFlag = 0, ovHeatFlag = 0, ovHeatbuz = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t read_adc(uint8_t x) {
	uint16_t adc[2];

	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1000);
	adc[0] = HAL_ADC_GetValue(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1000);
	adc[1] = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);

	return adc[x];
}

uint8_t readSW(){
	uint8_t SW4 = HAL_GPIO_ReadPin(SW_A_GPIO_Port,SW_A_Pin);
	uint8_t SW5 = HAL_GPIO_ReadPin(SW_B_GPIO_Port, SW_B_Pin);
	uint8_t SW_auto = HAL_GPIO_ReadPin(SW_AUTO_GPIO_Port,SW_AUTO_Pin);
	static uint8_t old4 = 1, old5 = 1, oldAuto = 1;
	uint8_t state = 0;

	if(!old4 & SW4) state = 41;
	if(!old5 & SW5) state = 51;
	if(!oldAuto & SW_auto) state = 100;

	old4 = SW4;
	old5 = SW5;
	oldAuto = SW_auto;

	return state;
}

void autoSetting(int autoM) {
	static int tampCC[] = {80,100,140,180,220};

	GPIO_TypeDef *LED[5] = { LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port,
			LED4_GPIO_Port, LED5_GPIO_Port };
	uint16_t Port_Pin[5] = { LED1_Pin, LED2_Pin, LED3_Pin, LED4_Pin, LED5_Pin };
	uint16_t adc = read_adc(1);
	uint8_t a = 0;

	if (autoM == 5 | (autoM == 4 & autoFlag > 0)) {
		for (int i = 1; i <= 5; i++) {
			if (adc <= (4095 / 5) * i) {
				HAL_GPIO_WritePin(LED[i - 1], Port_Pin[i - 1], GPIO_PIN_RESET);
				a = i - 1;
				break;
			}
		}
		for(int i = 0; i < 5; i++) if(i != a) LED1_GPIO_Port->BSRR = Port_Pin[i];
		if(tampCC[a] - MainTemp > 0)FireTemp = 9;
		else FireTemp = 1;
	}
	if (autoM == 6 & autoFlag <= 0){
		LED1_GPIO_Port->BSRR = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin;
		for (int i = 1; i <= 10; i++){
			if(adc <= (4095/9) * i){
				FireTemp = i;
				break;
			}
		}
	}
}
uint8_t stateSet(){
	uint8_t key = readSW();

	if(MainTemp >= 300 | ovHeatFlag) {
		ovHeatFlag = 1;
		return 1;
	}
	if(!ovHeatFlag){
		if(!HAL_GPIO_ReadPin(SW_LOCK_GPIO_Port, SW_LOCK_Pin)) {
			FireTemp = 0;
			return 2;
		}
		else if(!HAL_GPIO_ReadPin(SW_ON_GPIO_Port,SW_ON_Pin)){
			FireTemp = 0;
			return 3;
		}else{
			if(read_adc(0) < 3000) {
				FireTemp = 1;
				BUZZ_GPIO_Port -> BRR = BUZZ_Pin;
				return 4;
			}
			else if(autoFlag > 0) return 5;
			else return 6;
		}
	}
}

void display_power(uint8_t power)
{
	uint8_t i;
	const uint8_t data[10][12]={{0,0,0,0,0,0,0,0,0,0,0,0},
                              {13,0,0,0,13,0,0,0,13,0,0,0},
                              {76,0,0,0,76,0,0,0,76,0,0,0},
                              {255,0,0,0,255,0,0,0,255,0,0,0},
                              {255,0,13,0,255,0,13,0,255,0,13,0},
                              {255,0,76,0,255,0,76,0,255,0,76,0},
                              {255,0,255,0,255,0,255,0,255,0,255,0},
                              {255,13,255,13,255,13,255,13,255,13,255,13},
                              {255,76,255,76,255,76,255,76,255,76,255,76},
                              {255,255,255,255,255,255,255,255,255,255,255,255}};

	for (i = 0; i < 12; i++) led_ring_data[i] = data[power][i];
	led_ring_update(led_ring_data);
}

void BuzAction(int a){
	static int fir = 1;
	static int systick = 0;
	if(fir){
		fir = 0;
		systick = HAL_GetTick();
	}
	if (a) {
		if (HAL_GetTick() - systick < 100)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
		else if (HAL_GetTick() - systick < 200)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
		else if (HAL_GetTick() - systick < 300)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
		else if (HAL_GetTick() - systick < 400)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
		else if (HAL_GetTick() - systick >= 1000)
			fir = 1;
	}
	if (!a) {
		if (HAL_GetTick() - systick < 100)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
		else if (HAL_GetTick() - systick < 200)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
		else if (HAL_GetTick() - systick < 300)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
		else if (HAL_GetTick() - systick < 400)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
		else if (HAL_GetTick() - systick < 500)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
		else if (HAL_GetTick() - systick < 600)
			HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
		else if (HAL_GetTick() - systick >= 1000){
			fir = 1;
			ovHeatbuz = 1;
		}

	}

}

void FireAdd(void){
	static int fir = 1;
	static int addTerm;
	static int systick =0;

	if(FireTemp > 0 & fir){
		fir = 0;
		for(int i = 1; i <= 9; i++){
			if(FireTemp == i){
				addTerm = 1000 - i*100;
				systick = HAL_GetTick();
				break;
			}else addTerm = 0;
		}

	}
	if(addTerm < (HAL_GetTick() - systick) & !fir){
		fir = 1;
		MainTemp++;
	}
}

void FireDecade(void){
	static int fir = 1;
	static int decadeTrem;
	static int systick = 0;

	if(MainTemp > 0 & fir == 1){
		fir = 0;
		if(MainTemp-20 < 10) decadeTrem = 2900;
		else if(MainTemp-20 < 15) decadeTrem = 2200;
		else if(MainTemp-20 < 20) decadeTrem = 1600;
		else if(MainTemp-20 < 40) decadeTrem = 1100;
		else if(MainTemp-20 < 100) decadeTrem = 700;
		else if(MainTemp-20 < 200) decadeTrem = 400;
		else if(MainTemp-20 < 300) decadeTrem = 200;
		else decadeTrem = 100;
		systick = HAL_GetTick();
	}
	if(decadeTrem < (HAL_GetTick() - systick) & !fir & (MainTemp-20 > 0) ){
		fir = 1;
		MainTemp--;
	}
}

void gasRange(void){
	static int modenum = 0xff;
	static int setTemp = 20;
	static int old_fireTemp;
	static int fir = 1;
	static int buzFlag = 1;
	int key = readSW();
	modenum = stateSet();

	if(fir){
		fir = 0;
		old_fireTemp = FireTemp;
	}

	lcd_gotoxy(1,1);
	sprintf(lcd_buff, "TEMP:%03d%cC  %c:%d",MainTemp, 0xdf, 1,old_fireTemp);
	lcd_puts(lcd_buff);
	lcd_gotoxy(1,2);
	sprintf(lcd_buff,"[%s][%03d]",modeName[modenum-1], setTemp);
	lcd_puts(lcd_buff);

	autoSetting(modenum);

	if(key == 41 & setTemp < 280) setTemp+= 20;
	if(key == 51 & setTemp > 20 ) setTemp-= 20;
	if(key == 100) {
		autoFlag = !autoFlag;
		LED1_GPIO_Port->BSRR = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin|LED5_Pin;
	}

	FireAdd();
	FireDecade();

	if(setTemp != 20 & MainTemp > setTemp & stateSet() == 6)
		BuzAction(1);

	if(modenum == 1){
		while(!ovHeatbuz) BuzAction(0);
		FireTemp = 0;
		if(MainTemp <= 150 & HAL_GPIO_ReadPin(SW_ON_GPIO_Port,SW_ON_Pin))
			ovHeatbuz = 0, ovHeatFlag = 0;
	}

	HAL_Delay(100);
	display_power(old_fireTemp);
	if(old_fireTemp != FireTemp){
		if(old_fireTemp < FireTemp) old_fireTemp++;
		else if(old_fireTemp > FireTemp) old_fireTemp--;
	}

}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	for(int i = 0 ; i < 12; i++){
		led_ring_data[i] = led_ring_rgb(0,0x0f,0xff);
	}
	led_ring_update(led_ring_data);
	lcd_puts("\fSmart Gas Range\n             XXX");
	HAL_Delay(2000);
	lcd_cgram(1, 0);
	LcdSendByte(0, 1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		gasRange();

		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED_RING_Pin | BUZZ_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin | LED5_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			LCD_RS_Pin | LCD_RW_Pin | LCD_EN_Pin | LCD_D4_Pin | LCD_D5_Pin
					| LCD_D6_Pin | LCD_D7_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : SW_A_Pin SW_B_Pin */
	GPIO_InitStruct.Pin = SW_A_Pin | SW_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : SW_ON_Pin SW_AUTO_Pin SW_LOCK_Pin */
	GPIO_InitStruct.Pin = SW_ON_Pin | SW_AUTO_Pin | SW_LOCK_Pin;
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
	GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin | LED5_Pin
			| BUZZ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_RS_Pin LCD_RW_Pin LCD_EN_Pin LCD_D4_Pin
	 LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
	GPIO_InitStruct.Pin = LCD_RS_Pin | LCD_RW_Pin | LCD_EN_Pin | LCD_D4_Pin
			| LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin;
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
void Error_Handler(void) {
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
