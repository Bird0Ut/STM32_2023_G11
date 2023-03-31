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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "calcullux.h"
#include "lib_lcd.h"
#include "string.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
static rgb_lcd data;
static SHT_31 sht;
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

/* USER CODE BEGIN PV */
uint8_t buf[15];
  uint8_t buf1[15];
  //HAL_StatusTypeDef ret;
  float temp_c;
  float hydro_c;
  uint16_t lum_c;
  uint16_t lum_c2;
  unsigned long lux;
  static uint8_t lum[15];
  int retmes;
  int compteur;
  int aux;
  int affichagechanel = 0;
  unsigned long aux2;

const int positionMin = 750; //-45 deg
const int positionnul = 1600; //0 deg
const int positionMax = 2400; //+45 deg

uint8_t ButtonPressed=0;
uint16_t readValue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int calcullum(int nb,unsigned long *lumi)
   {
   	if(TLS2561_read(&lum) == -1) //fonction de lecture des valeurs de chanels A et B du TLS2561
   			return -1; // cas pour capteur defaillant ou débranché
   			else
   			{
   				lum_c = (lum[0]+lum[1]*256); //conversion des channels A:lum_c et B:lum_c2
   				lum_c2 = (lum[2]+lum[3]*256);
   				aux2 = CalculateLux(0,2, lum_c,
   						lum_c2,1); // conversion de la mesure en LUX tel que : iGain = 0,integration time =402ms,Chan A,Chan B,package type CS
   					*lumi=aux2;
   				return 0;
   			}

   }

   int mesure(void){
		  //=========================================partie temp et humid ==============================================

		  	   	      	  retmes = sht31_read(&sht);//lecture de l'humidité et de la température dans la struct sht
		  	   	      	  if(retmes){ // mesure OK
		  	   	      	lcd_position(&hi2c1,0,0); //réglage de couleur écran en fonction du mode de fonctionnement puis affichage mesure temperature et humidité
		  	   	      	if(ButtonPressed==0)
		  	   	      		reglagecouleur(4,4,4);
		  	   	      	else if(ButtonPressed==1)
		  	   	      		reglagecouleur(0,2,10);
		  	   	      	else if(ButtonPressed==2)
		  	   			  	reglagecouleur(6,0,6);
		  	   	      	else if(ButtonPressed==3)
		  	   			  	reglagecouleur(8,4,0);
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
		  	   	      	  				lcd_position(&hi2c1,3,0);
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
		  	   	      	  				lcd_position(&hi2c1,3,1);
		  	   	      	  				lcd_write(&hi2c1,0x3F & (buf[4])+0x30);
		  	   	      	  				lcd_write(&hi2c1,0x3F & (buf[5])+0x30);
		  	   	      	  				lcd_print(&hi2c1,".");
		  	   	      	  				lcd_write(&hi2c1,0x3F & (buf[6])+0x30);
		  	   	      	  				lcd_write(&hi2c1,0x3F & (buf[7])+0x30);
		  	   	      	  				if(sht.hydro>100)
		  	   	      	  					{reglagecouleur(0xFF,0,0);
		  	   	      	  					lcd_print(&hi2c1," ALARM");}

		  	   	      	  				  }
		  	   	      	  		if(retmes == -1 ) // retour fonction sur erreur de transmission
		  	   	      	  		  {reglagecouleur(0xFF,0,0);
		  	   	      	  		  lcd_position(&hi2c1,0,1);
		  	   	      	  		  lcd_print(&hi2c1,"  erreur trans");
		  	   	      	  		  }
		  	   	      	  		if(retmes == -2 ) // retour fonction sur erreur de reception
		  	   	      	  	  	  {reglagecouleur(0xFF,0,0);
		  	   	      	  	  	  lcd_position(&hi2c1,0,1);
		  	   	      	  	  	  lcd_print(&hi2c1,"  erreur recep");
		  	   	      	  		  }
		  	   	      	  //===============================================================================================================

		  	   	      	  		HAL_Delay(5);

		  	   	      	  //=================================== parti lum ==================================================================

		  	   	      	  		if(calcullum(1,&lux) == -1) // fonction de mesure+conversion lumiere en LUX dans la variable lux
		  	   	      	  			{reglagecouleur(0xFF,0,0); // retour fonction sur erreur capteur
		  	   	      	  			lcd_position(&hi2c1,0,1);
		  	   	      	  			lcd_print(&hi2c1," erreur TLS2561");
		  	   	      	  			aux = 3;

		  	   	      	  			}
		  	   	      	  		else // mesure OK
		  	   	      	  			{

		  	   	      	  			//============================================================= affichage chanels ======================
		  	   	      	  			if(affichagechanel) // gestion des affichages de channels du TLS2561 pour debug, activé:affichagechanel=1, désactivé affichagechanel=0
		  	   	      	  			{
		  	   	      	  			lum_c /= 65.536;
		  	   	      	  			lum_c2 /= 65.536;

		  	   	      	  			buf[8]= (int)lum_c/1000;
		  	   	      	  			buf[9]= (int)(lum_c - buf[8]*1000)/100;
		  	   	      	  			buf[10]= (int)(lum_c - buf[8]*1000 - buf[9]*100)/10;
		  	   	      	  			buf[11]= (int)(lum_c - buf[8]*1000 - buf[9]*100 - buf[10]*10);

		  	   	      	  			buf1[8]= (int)lum_c2/1000;
		  	   	      	  			buf1[9]= (int)(lum_c2 - buf1[8]*1000)/100;
		  	   	      	  			buf1[10]= (int)(lum_c2 - buf1[8]*1000 - buf1[9]*100)/10;
		  	   	      	  			buf1[11]= (int)(lum_c2 - buf1[8]*1000 - buf1[9]*100 - buf1[10]*10);

		  	   	      	  			if(retmes){
		  	   	      	  			lcd_position(&hi2c1,9,0);
		  	   	      	  			lcd_print(&hi2c1,"C0=");
		  	   	      	  			lcd_write(&hi2c1,0x3F & (buf[8])+0x30);
		  	   	      	  			lcd_write(&hi2c1,0x3F & (buf[9])+0x30);
		  	   	      	  			lcd_write(&hi2c1,0x3F & (buf[10])+0x30);
		  	   	      	  			lcd_write(&hi2c1,0x3F & (buf[11])+0x30);
		  	   	      	  			lcd_position(&hi2c1,9,1);
		  	   	      	  			lcd_print(&hi2c1,"C1=");
		  	   	      	  			lcd_write(&hi2c1,0x3F & (buf1[8])+0x30);
		  	   	      	  			lcd_write(&hi2c1,0x3F & (buf1[9])+0x30);
		  	   	      	  			lcd_write(&hi2c1,0x3F & (buf1[10])+0x30);
		  	   	      	  			lcd_write(&hi2c1,0x3F & (buf1[11])+0x30);
		  	   	      	  			}
		  	   	      	  			}
		  	   	      	  			//========================================================================================================
		  	   	      	  			if(!affichagechanel) // affichage LUX
		  	   	      	  			{
		  	   	      	  				buf[8]= (int)lux/100000;
		  	   	      	  				buf[9]= (int)(lux - buf[8]*100000)/10000;
		  	   	      	  				buf[10]= (int)(lux - buf[8]*100000 - buf[9]*10000)/1000;
		  	   	      	  				buf[11]= (int)(lux - buf[8]*100000 - buf[9]*10000 - buf[10]*1000)/100;
		  	   	      	  				buf[12]= (int)(lux - buf[8]*100000 - buf[9]*10000 - buf[10]*1000 - buf[11]*100)/10;
		  	   	      	  				buf[13]= (int)(lux - buf[8]*100000 - buf[9]*10000 - buf[10]*1000 - buf[11]*100 - buf[12]*10);
		  	   	      	  				if(retmes)
		  	   	      	  				{
		  	   	      	  					lcd_position(&hi2c1,9,0);
		  	   	      	  					lcd_print(&hi2c1,"Lux =   ");
		  	   	      	  					lcd_position(&hi2c1,9,1);
		  	   	      	  					lcd_write(&hi2c1,0x3F & (buf[8])+0x30);
		  	   	      	  					lcd_write(&hi2c1,0x3F & (buf[9])+0x30);
		  	   	      	  					lcd_write(&hi2c1,0x3F & (buf[10])+0x30);
		  	   	      	  					lcd_write(&hi2c1,0x3F & (buf[11])+0x30);
		  	   	      	  					lcd_write(&hi2c1,0x3F & (buf[12])+0x30);
		  	   	      	  					lcd_write(&hi2c1,0x3F & (buf[13])+0x30);
		  	   	      	  				}

		  	   	      	  			}


		  	   	      	  			}
		  	   	      	  		if(aux > 0) // reinit si erreur capteur TLS2561
		  	   	      	  			{
		  	   	      	  			TLS2561_init();
		  	   	      	  			aux-=1;
		  	   	      	  			}
		  	   	      	  //====================================================================================================================
   return 1;
   }
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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // initialisation capteurs SHT31 et TLS2561 et écran LCD
     sht31_init();
     TLS2561_init();
     lcd_init(&hi2c1,&data);

     clearlcd(); // LCD vierge couleur RGB : 0xFF,0xFF ,0xFF

   lcd_position(&hi2c1,0,0);
   lcd_print(&hi2c1,"     HELLO!     ");
   HAL_Delay(2000);


  HAL_TIM_Base_Start_IT(&htim3); // start timer pour interrupt mesure

  //partie lilian

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,positionnul);// initialisation à la poition 0 degré
  HAL_Delay(500);
  HAL_GPIO_EXTI_IRQHandler ( GPIO_PIN_9 );  // Réinitialiser l'interruption PIN9
  GPIO_PinState PinState;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(compteur>5)
		  {if(mesure());
		  	  compteur=0;}
	  	  //==================Mod de fonctionnement========================================
	  	      			lcd_position(&hi2c1,15,0);
	  	  //===================mod Manuel=======================================================
	  	      		  		if(ButtonPressed==0){
	  	      		  lcd_print(&hi2c1,"M");

	  	      	     	  HAL_ADC_Start(&hadc1);
	  	      	     	      HAL_ADC_PollForConversion(&hadc1,1000);
	  	      	     	      readValue = HAL_ADC_GetValue(&hadc1);
	  	      	     	      readValue = readValue/2.5;
	  	      	     	      if (readValue >1600)
	  	      	     	    	  readValue = 1600;
	  	      	     	      __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,(750+readValue));
	  	      	     	      HAL_ADC_Stop(&hadc1);
	  	      		  		}
	  	   //===================mod Automatique======================================================
	  	      		  		if(ButtonPressed==1){
	  	      		  			lcd_print(&hi2c1,"A");

	  	      		 	  for(int i = positionMin; i < positionMax; i += 10) {
	  	      			    		  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,i);
	  	      			    		  HAL_Delay(20);
	  	      			    	  	  }
	  	      		  		}
	  	   //===================mod Nuit======================================================
	  	      		  		if(ButtonPressed==2){
	  	      		  			lcd_print(&hi2c1,"N");

	  		    		  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,positionMin);
	  		  	      			    		HAL_Delay(20);}
	  	   //===================mod Jour======================================================
	  	      		  		if(ButtonPressed==3){
	  	      		   			lcd_print(&hi2c1,"J");

		  		    		  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,positionMax);
		  		  	      			    	HAL_Delay(20);}

	  	      	  		  HAL_Delay(5);

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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		compteur++;

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
  /*//if(GPIO_Pin == PUSH_BUTTON_Pin)
    GPIO_PinState PinState;
    PinState = HAL_GPIO_ReadPin( PA9_GPIO_Port, GPIO_PIN_9);
   // PinState = HAL_GPIO_ReadPin( PUSH_BUTTON_GPIO_Port, PUSH_BUTTON_Pin);
    if ( PinState == GPIO_PIN_RESET )
      ButtonPressed = 1;
    else
      ButtonPressed = 0;*/
	if (HAL_GPIO_ReadPin( PA9_GPIO_Port, GPIO_PIN_9) == GPIO_PIN_SET)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		reglagecouleur(0x00,0xFF,0x00);
		if(ButtonPressed<3)
			ButtonPressed += 1;
		else
			ButtonPressed = 0;

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
