/*
 * lcd.h
 *
 *  Created on: 19 juin 2023
 *      Author: ahmet
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f4xx.h"

#define CLEAR_DISPLAY 0x01
#define RETURN_HOME 0x02
#define ENTRY_MODE_SET 0x04
#define DISPLAY_ON_OFF 0x08
#define CURSOR_DISPLAY_SHIFT 0x10
#define FUNCTION_SET 0x20
#define SET_CGRAM 0x40
#define SET_DDRAM 0x80

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


//80 1st line
//C0 2nd line
//94 3rd line
//D4 4rd line

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


void lcd_write(LCD_handler lcd, uint8_t data);
void lcd_WriteCommand(LCD_handler lcd, uint8_t data);
void lcd_WriteData(LCD_handler lcd, uint8_t data);
void lcd_init_gpio(LCD_handler lcd);
void lcd_init(LCD_handler lcd);
void lcd_WriteString(LCD_handler lcd,char a[]);
void lcd_SetCursor(LCD_handler lcd,int line,int row);




#endif /* INC_LCD_H_ */
