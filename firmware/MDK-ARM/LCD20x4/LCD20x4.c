#include "LCD20x4.h"
#include "main.h"  
#include "i2c1.h"

	
uint8_t lcd_backlight_state = 1 ;
char lcd_buffer[20];

void LCD_Init(void) 
	{
    HAL_Delay(50); // Ожидание после подачи питания

    // Специальная последовательность инициализации 4-битного режима
    lcd_send_nibble(0x03, 0);
    HAL_Delay(5);
    lcd_send_nibble(0x03, 0);
    HAL_Delay(1);
    lcd_send_nibble(0x03, 0);
    HAL_Delay(1);
    lcd_send_nibble(0x02, 0); // Переход на 4-битный интерфейс
    HAL_Delay(1);

    // Теперь отправляем команды полными байтами
    lcd_send_byte(0x28, 0); // Function Set: 4-bit, 2 lines, 5x8 dots
    lcd_send_byte(0x0C, 0); // Display ON, Cursor OFF, Blink OFF
    lcd_send_byte(0x01, 0); // Clear Display
    HAL_Delay(2); // Команда очистки требует больше времени
    lcd_send_byte(0x06, 0); // Entry Mode Set: Increment, No shift
}
	
void lcd_send_nibble(uint8_t data, uint8_t mode)
{
	uint8_t packet = 0;
	if (mode) packet |= LCD_RS_PIN;
	if (lcd_backlight_state) packet |= LCD_BL_PIN;
	packet |= (data & 0x0F) << 4;
	HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,&packet,1, HAL_MAX_DELAY);
	HAL_Delay(1);
	packet |= LCD_E_PIN;
	HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,&packet,1,HAL_MAX_DELAY);
	HAL_Delay(1);
	packet &= ~LCD_E_PIN;
  HAL_I2C_Master_Transmit(&hi2c1,LCD_I2C_ADDR,&packet,1, HAL_MAX_DELAY);
	HAL_Delay(1);
}

void lcd_send_byte(uint8_t data, uint8_t mode)
{
	lcd_send_nibble(data >> 4, mode);
	lcd_send_nibble(data & 0x0F,mode);
}

void LCD_Clear(void) 
{
    lcd_send_byte(0x01, 0);
	  LCD_SetCursor(0, 0);
    HAL_Delay(2);
}
void LCD_SetCursor(uint8_t col, uint8_t row) 
{
	const uint8_t row_offset[] = {0x00,0x40,0x14,0x54};
	if (row>3) row=3;
	lcd_send_byte(0x80|col + row_offset[row],0);
	}

void LCD_PrintChar (char ch)
{
	lcd_send_byte(ch,1);
}
void LCD_PrintString(const char *str)
{
	while(*str) {
		LCD_PrintChar(*str++);
	}
}
 void LCD_BacklightOn(void)
 {
	 lcd_backlight_state=1;
	 lcd_send_byte(0,0);	 
 }
  
void LCD_BacklightOff(void)
 {
	 lcd_backlight_state=0;
	 lcd_send_byte(0,0);	 
 }
 
 