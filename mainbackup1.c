/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib_lcd.h"
#include "string.h"
#include <stdlib.h>
#include <stdio.h>
static rgb_lcd data;
static SHT_31 sht;
//static const uint8_t SHT31_ADDR = 0x40;
//static const uint8_t REG_TEMP = 0x11;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//I2C_HandleTypeDef hi2c1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
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
  uint8_t buf[12];
  uint8_t buf1[2];
  HAL_StatusTypeDef ret;
  float temp_c;
  float hydro_c;
  float lum_c;
  static uint8_t lum[5];
  int retmes;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  sht31_init();
  lcd_init(&hi2c1,&data);
  /* USER CODE BEGIN 2 */

  clearlcd();

lcd_position(&hi2c1,0,0);
lcd_print(&hi2c1,"     HELLO!     ");
HAL_Delay(3000);
lcd_position(&hi2c1,0,0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  retmes = sht31_read(&sht);
		if(retmes){
			reglagecouleur(4,4,4);
			  lcd_print(&hi2c1,"T");
			  lcd_write(&hi2c1,0b11011111);
			  lcd_print(&hi2c1,"=              ");
			  lcd_position(&hi2c1,0,1);
			  lcd_print(&hi2c1,"H");
			  lcd_write(&hi2c1,0b00100101);
			  lcd_print(&hi2c1,"=              ");
			temp_c = 100*(sht.temp);
			hydro_c = 100*(sht.hydro);
				buf[0]= (int)temp_c/1000;
				buf[1]= (int)(temp_c - buf[0]*1000)/100;
				buf[2]= (int)(temp_c - buf[0]*1000 - buf[1]*100)/10;
				buf[3]= (int)(temp_c - buf[0]*1000 - buf[1]*100 - buf[2]*10);
				lcd_position(&hi2c1,4,0);
				lcd_write(&hi2c1,0x3F & (buf[0])+0x30);
				lcd_write(&hi2c1,0x3F & (buf[1])+0x30);
				lcd_print(&hi2c1,".");
				lcd_write(&hi2c1,0x3F & (buf[2])+0x30);
				lcd_write(&hi2c1,0x3F & (buf[3])+0x30);
				if(sht.temp>50)
					{reglagecouleur(0xFF,0,0);
					lcd_print(&hi2c1," ALARM");}
				buf[4]= (int)hydro_c/1000;
				buf[5]= (int)(hydro_c - buf[4]*1000)/100;
				buf[6]= (int)(hydro_c - buf[4]*1000 - buf[5]*100)/10;
				buf[7]= (int)(hydro_c - buf[4]*1000 - buf[5]*100 - buf[6]*10);
				lcd_position(&hi2c1,4,1);
				lcd_write(&hi2c1,0x3F & (buf[4])+0x30);
				lcd_write(&hi2c1,0x3F & (buf[5])+0x30);
				lcd_print(&hi2c1,".");
				lcd_write(&hi2c1,0x3F & (buf[6])+0x30);
				lcd_write(&hi2c1,0x3F & (buf[7])+0x30);
				if(sht.hydro>100)
					{reglagecouleur(0xFF,0,0);
					lcd_print(&hi2c1," ALARM");}

				  }
		if(retmes == -1 )
		  {reglagecouleur(0xFF,0,0);
		  lcd_position(&hi2c1,0,1);
		  lcd_print(&hi2c1,"  erreur trans");
		  }
		if(retmes == -2 )
	  	  {reglagecouleur(0xFF,0,0);
	  	  lcd_position(&hi2c1,0,1);
	  	  lcd_print(&hi2c1,"  erreur recep");
		  }
		  HAL_Delay(2000);

    /* USER CODE BEGIN 3 */
}
}
  /* USER CODE END 3 */


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
