/*
 * lcd.c
 *
 *  Created on: 19 juin 2023
 *      Author: ahmet
 */

#include "lcd.h"
#include "string.h"

const int LINE[4] = {0x00,0x40,0x14,0x54};

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
	HAL_Delay(5);
}

void lcd_WriteData(LCD_handler lcd, uint8_t data)
{
	HAL_GPIO_WritePin(lcd.rs_port, lcd.rs_pin,GPIO_PIN_SET);

	lcd_write(lcd,((data&0xF0)>>4));
	lcd_write(lcd,(data&0x0F));
	HAL_Delay(5);
}

void lcd_init_gpio(LCD_handler lcd)
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

void lcd_init(LCD_handler lcd)
{

	lcd_WriteCommand(lcd,0x33);
	lcd_WriteCommand(lcd,0x32);
	lcd_WriteCommand(lcd, FUNCTION_SET | 0x08); //DL=0 4bit, N=1 2line mode, F =X 5x8 font
	lcd_WriteCommand(lcd,DISPLAY_ON_OFF);		//D=0 display off, C=0 cursor off B=0 cursor pos off
	lcd_WriteCommand(lcd,CLEAR_DISPLAY);
	lcd_WriteCommand(lcd,ENTRY_MODE_SET | 0x02);//I/D=1 increment, S=0 display shift off
	lcd_WriteCommand(lcd,DISPLAY_ON_OFF | 0x07);//D=1 display on, C=1 cursor on B=1 cursor pos on
}

void lcd_WriteString(LCD_handler lcd,char a[])
{
	for(int i=0;i<strlen(a);i++)
	{
		lcd_WriteData(lcd,a[i]);
	}
}

void lcd_SetCursor(LCD_handler lcd,int line,int row)
{
	if((line >-1 && line <5) || (row >-1 && row < 20))
	{
		lcd_WriteCommand(lcd, SET_DDRAM | (LINE[line] + row));
	}
}
