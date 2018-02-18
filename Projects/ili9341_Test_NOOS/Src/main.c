/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

//TODO
// 1. SPI should be setup for SPI3 (this is the only one on the STM32F3
// 2. Pin mapping needs to be sorted for all IO.
//    SPI_LCD_CS = PD2 (CN7-4)
//    SPI_LCD_CLK = PC10 (CN7-1)
//	  SPI_LCD_MOSI = PC12 (CN7-3)
//	  SPI_LCD_MISO = PC11 (CN7-2)
//    SPI_LCD_DC = PA15 (CN7-17)
//	  SPI_LCD_RST = PA13 (CN7-23)
// 3. Review timer 1 use


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

#include "lcd_spi.h"

//#include "tim.h"
//#include "usart.h"
//#include "gpio.h"


#include "ILI9341_Touchscreen.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#include "snow_tiger.h"

#include "SEGGER_SYSVIEW.h"
#include "string.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);


int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  LCD_SPI_Init();
  //MX_TIM1_Init();


  //SEGGER_SYSVIEW_Conf();
  //SEGGER_SYSVIEW_Start(); // start SystemView
  //setvbuf(stdin, NULL, _IONBF, 0);

  ILI9341_Init();//initial driver setup to drive ili9341

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

//----------------------------------------------------------PERFORMANCE TEST
		ILI9341_Fill_Screen(WHITE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		ILI9341_Draw_Text("FPS TEST, 40 loop 2 screens", 10, 10, BLACK, 1, WHITE);
		HAL_Delay(2000);
		ILI9341_Fill_Screen(WHITE);

		uint32_t Timer_Counter = 0;
		uint32_t j = 0;
		uint16_t i = 0;

		for(j = 0; j < 2; j++)
		{
			//HAL_TIM_Base_Start(&htim1);
			for(i = 0; i < 10; i++)
			{
				ILI9341_Fill_Screen(RED);
				ILI9341_Fill_Screen(DARKGREEN);
			}

			//20.000 per second!
			//HAL_TIM_Base_Stop(&htim1);
			//Timer_Counter += __HAL_TIM_GET_COUNTER(&htim1);
			//__HAL_TIM_SET_COUNTER(&htim1, 0);
		}
		Timer_Counter = 444;

		char counter_buff[30];
		ILI9341_Fill_Screen(WHITE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		sprintf(counter_buff, "Timer counter value: %d", Timer_Counter*2);
		ILI9341_Draw_Text(counter_buff, 10, 10, BLACK, 1, WHITE);

		double seconds_passed = 2*((float)Timer_Counter / 20000);
		sprintf(counter_buff, "Time: %.3f Sec", seconds_passed);
		ILI9341_Draw_Text(counter_buff, 10, 30, BLACK, 2, WHITE);

		double timer_float = 20/(((float)Timer_Counter)/20000);	//Frames per sec

		sprintf(counter_buff, "FPS:  %.2f", timer_float);
		ILI9341_Draw_Text(counter_buff, 10, 50, BLACK, 2, WHITE);
		double MB_PS = timer_float*240*320*2/1000000;
		sprintf(counter_buff, "MB/S: %.2f", MB_PS);
		ILI9341_Draw_Text(counter_buff, 10, 70, BLACK, 2, WHITE);
		double SPI_utilized_percentage = (MB_PS/(6.25 ))*100;		//50mbits / 8 bits
		sprintf(counter_buff, "SPI Utilized: %.2f", SPI_utilized_percentage);
		ILI9341_Draw_Text(counter_buff, 10, 90, BLACK, 2, WHITE);
		HAL_Delay(10000);


		static uint16_t x = 0;
		static uint16_t y = 0;

		char Temp_Buffer_text[40];

//----------------------------------------------------------COUNTING MULTIPLE SEGMENTS
		ILI9341_Fill_Screen(WHITE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		ILI9341_Draw_Text("Counting multiple segments at once", 10, 10, BLACK, 1, WHITE);
		HAL_Delay(2000);
		ILI9341_Fill_Screen(WHITE);


		for(i = 0; i <= 10; i++)
		{
		sprintf(Temp_Buffer_text, "Counting: %d", i);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 2, WHITE);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 30, BLUE, 2, WHITE);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 50, RED, 2, WHITE);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 70, GREEN, 2, WHITE);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 90, BLACK, 2, WHITE);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 110, BLUE, 2, WHITE);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 130, RED, 2, WHITE);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 150, GREEN, 2, WHITE);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 170, WHITE, 2, BLACK);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 190, BLUE, 2, BLACK);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 210, RED, 2, BLACK);
		}

		HAL_Delay(1000);

//----------------------------------------------------------COUNTING SINGLE SEGMENT
		ILI9341_Fill_Screen(WHITE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		ILI9341_Draw_Text("Counting single segment", 10, 10, BLACK, 1, WHITE);
		HAL_Delay(2000);
		ILI9341_Fill_Screen(WHITE);

		for(i = 0; i <= 100; i++)
		{
		sprintf(Temp_Buffer_text, "Counting: %d", i);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 3, WHITE);
		}

		HAL_Delay(1000);

//----------------------------------------------------------ALIGNMENT TEST
		ILI9341_Fill_Screen(WHITE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		ILI9341_Draw_Text("Rectangle alignment check", 10, 10, BLACK, 1, WHITE);
		HAL_Delay(2000);
		ILI9341_Fill_Screen(WHITE);

		ILI9341_Draw_Hollow_Rectangle_Coord(50, 50, 100, 100, BLACK);
		ILI9341_Draw_Filled_Rectangle_Coord(20, 20, 50, 50, BLACK);
		ILI9341_Draw_Hollow_Rectangle_Coord(10, 10, 19, 19, BLACK);
		HAL_Delay(1000);

//----------------------------------------------------------LINES EXAMPLE
		ILI9341_Fill_Screen(WHITE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
		ILI9341_Draw_Text("Horizontal and Vertical lines", 10, 20, BLACK, 1, WHITE);
		HAL_Delay(2000);
		ILI9341_Fill_Screen(WHITE);


		HAL_Delay(1000);

//----------------------------------------------------------INDIVIDUAL PIXEL EXAMPLE

		ILI9341_Fill_Screen(WHITE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		ILI9341_Draw_Text("Slow draw by selecting", 10, 10, BLACK, 1, WHITE);
		ILI9341_Draw_Text("and adressing pixels", 10, 20, BLACK, 1, WHITE);
		HAL_Delay(2000);
		ILI9341_Fill_Screen(WHITE);


		x = 0;
		y = 0;
		while (y < 240)
		{
		while ((x < 320) && (y < 240))
		{

			if(x % 2)
			{
				ILI9341_Draw_Pixel(x, y, BLACK);
			}

			x++;
		}

			y++;
			x = 0;
		}

		x = 0;
		y = 0;


		while (y < 240)
		{
		while ((x < 320) && (y < 240))
		{

			if(y % 2)
			{
				ILI9341_Draw_Pixel(x, y, BLACK);
			}

			x++;
		}

			y++;
			x = 0;
		}
		HAL_Delay(2000);


//----------------------------------------------------------565 COLOUR EXAMPLE, Grayscale
		ILI9341_Fill_Screen(WHITE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		ILI9341_Draw_Text("Colour gradient", 10, 10, BLACK, 1, WHITE);
		ILI9341_Draw_Text("Grayscale", 10, 20, BLACK, 1, WHITE);
		HAL_Delay(2000);


		for(i = 0; i <= (320); i++)
		{
			uint16_t Red = 0;
			uint16_t Green = 0;
			uint16_t Blue = 0;

			Red = i/(10);
			Red <<= 11;
			Green = i/(5);
			Green <<= 5;
			Blue = i/(10);



			uint16_t RGB_color = Red + Green + Blue;
			ILI9341_Draw_Rectangle(i, x, 1, 240, RGB_color);

		}
		HAL_Delay(2000);

//----------------------------------------------------------IMAGE EXAMPLE, Snow Tiger
		ILI9341_Fill_Screen(WHITE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		ILI9341_Draw_Text("RGB Picture", 10, 10, BLACK, 1, WHITE);
		ILI9341_Draw_Text("TIGER", 10, 20, BLACK, 1, WHITE);
		HAL_Delay(2000);
		//ILI9341_Draw_Image((const char*)snow_tiger, SCREEN_VERTICAL_2);
		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
		HAL_Delay(10000);


//----------------------------------------------------------TOUCHSCREEN EXAMPLE
		ILI9341_Fill_Screen(WHITE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		ILI9341_Draw_Text("Touchscreen", 10, 10, BLACK, 2, WHITE);
		ILI9341_Draw_Text("Touch to draw", 10, 30, BLACK, 2, WHITE);
		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);



  }
  /* USER CODE END 3 */


}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
    //HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
