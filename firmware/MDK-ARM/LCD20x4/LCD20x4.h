#pragma once

#include "main.h"


#define LCD_I2C_ADDR  (0x27<<1) 
#define LCD_RS_PIN (1<<0)
#define LCD_RW_PIN (1<<1)
#define LCD_E_PIN (1<<2)
#define LCD_BL_PIN (1<<3)
#define LCD_D4_PIN (1<<4)
#define LCD_D5_PIN (1<<5)
#define LCD_D6_PIN (1<<6)
#define LCD_D7_PIN (1<<7)

void LCD_Init(void);
void lcd_send_nibble(uint8_t data, uint8_t mode);
void lcd_send_byte(uint8_t data, uint8_t mode);
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t col, uint8_t row);
void LCD_PrintChar (char ch);
void LCD_PrintString(const char *str);
void LCD_BacklightOn(void);
void LCD_BacklightOff(void);
void Error_Handler(void);

extern uint8_t lcd_backlight_state;
extern char lcd_buffer[20];
