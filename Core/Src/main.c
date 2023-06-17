
#include "main.h"
#include "string.h"
#include "stdio.h"


#define RS_PORT GPIOA	//D7
#define RS_PIN GPIO_PIN_8

#define D7_PORT GPIOA	//D8
#define D7_PIN GPIO_PIN_9

#define D6_PORT GPIOC	//D6
#define D6_PIN GPIO_PIN_7

#define D5_PORT GPIOB	//D10
#define D5_PIN GPIO_PIN_6

#define D4_PORT GPIOA	//D11
#define D4_PIN GPIO_PIN_7

#define E_PORT GPIOA	//D12
#define E_PIN GPIO_PIN_6


typedef struct{
	GPIO_TypeDef * port;
	uint16_t pin;

}data_gpio;

typedef struct{
	GPIO_TypeDef* rs_port;
	uint16_t  rs_pin;

	GPIO_TypeDef * en_port;
	uint16_t  en_pin;

	data_gpio data[4];
}LCD_handler;



UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);


void lcd_write(LCD_handler lcd, uint8_t data)
{
	for(uint8_t i =0; i<4;i++)
	{
		HAL_GPIO_WritePin(lcd.data[i].port, lcd.data[i].pin, (data >> i)&0x01);
	}
	HAL_GPIO_WritePin(lcd.en_port, lcd.en_pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(lcd.en_port, lcd.en_pin, GPIO_PIN_RESET);
}

void lcd_WriteCommand(LCD_handler lcd, uint8_t data)
{
	HAL_GPIO_WritePin(lcd.rs_port, lcd.rs_pin,GPIO_PIN_RESET);

	lcd_write(lcd,((data&0xF0)>>4));
	lcd_write(lcd,(data&0x0F));
}

void lcd_WriteData(LCD_handler lcd, uint8_t data)
{
	HAL_GPIO_WritePin(lcd.rs_port, lcd.rs_pin,GPIO_PIN_SET);

	lcd_write(lcd,((data&0xF0)>>4));
	lcd_write(lcd,(data&0x0F));
}

void init_lcd_gpio(LCD_handler lcd)
{
	GPIO_InitTypeDef gpio;
	for(int i=0;i<4;i++)
	{
		gpio.Mode = GPIO_MODE_OUTPUT_PP;
		gpio.Speed= GPIO_SPEED_FREQ_LOW;
		gpio.Pin = lcd.data[i].pin;
		HAL_GPIO_Init(lcd.data[i].port, &gpio);
	}

	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Speed= GPIO_SPEED_FREQ_LOW;
	gpio.Pin = lcd.en_pin;
	HAL_GPIO_Init(lcd.en_port, &gpio);

	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Speed= GPIO_SPEED_FREQ_LOW;
	gpio.Pin = lcd.rs_pin;
	HAL_GPIO_Init(lcd.rs_port, &gpio);
}


void lcd_WriteString(LCD_handler lcd,char a[])
{
	for(int i=0;i<strlen(a);i++)
	{
		lcd_WriteData(lcd,a[i]);
		HAL_Delay(5);
	}
}

void int_to_string(int n, char* str) {
    sprintf(str, "%d", n);
}

int main(void)
{

  HAL_Init();


  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();

  //INIT

  data_gpio pins[4]={
		  {D4_PORT,D4_PIN},
		  {D5_PORT,D5_PIN},
		  {D6_PORT,D6_PIN},
		  {D7_PORT,D7_PIN}};
  LCD_handler lcd;

  for (int i = 0; i < 4; i++) {
	  lcd.data[i] = pins[i];
  }
  lcd.en_pin = E_PIN;
  lcd.en_port = E_PORT;
  lcd.rs_pin = RS_PIN;
  lcd.rs_port =RS_PORT;

  init_lcd_gpio(lcd);


  lcd_WriteCommand(lcd,0x33);
  lcd_WriteCommand(lcd,0x32);
  lcd_WriteCommand(lcd,0x28);

  lcd_WriteCommand(lcd,0x08);
  lcd_WriteCommand(lcd,0x01);
  lcd_WriteCommand(lcd,0x04 | 0x02);



  //Display on
  lcd_WriteCommand(lcd,0x0F);
  HAL_Delay(10);

  lcd_WriteString(lcd,"Bonjour");
  HAL_Delay(2000);
  //lcd_WriteCommand(lcd, 0x01);//Clear Display
  //lcd_WriteCommand(lcd,0x02);// Cursor to the begining
  lcd_WriteCommand(lcd, 0x14);
  lcd_WriteCommand(lcd, 0x14);
  lcd_WriteCommand(lcd, 0x14);
  lcd_WriteCommand(lcd, 0xD4+4);

  //80 1st line
  //C0 2nd line
  //94 3rd line
  //D4 4rd line
  int x=0,y=0,z=0;
  char X[5];
  char Y[5];
  char Z[5];
  while (1)
  {
	  lcd_WriteCommand(lcd, 0x01);

	  int_to_string(x, X);
	  int_to_string(y, Y);
	  int_to_string(z, Z);

	  lcd_WriteString(lcd,"X : ");
	  lcd_WriteString(lcd,X);
	  lcd_WriteCommand(lcd, 0xC0);
	  lcd_WriteString(lcd,"Y : ");
	  lcd_WriteString(lcd,Y);
	  lcd_WriteCommand(lcd, 0x94);
	  lcd_WriteString(lcd,"Z : ");
	  lcd_WriteString(lcd,Z);

	  x=x+2;
	  y=y+4;
	  z=z+10;

	  HAL_Delay(1000);

  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();




  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);


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
