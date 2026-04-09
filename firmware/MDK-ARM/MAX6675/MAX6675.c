#include "MAX6675.h"
#include "spi1.h"

float temp = 0.0f;

void read_max6675 (void)
{
	uint8_t raw_temp_data[2] = {0};
	uint16_t raw_data = 0;
	uint16_t temp_data = 0;
	
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	
	HAL_Delay(1);
				
	HAL_SPI_Receive(&hspi1,&raw_temp_data[0],2,HAL_MAX_DELAY);
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	
	raw_data = (raw_temp_data[0] << 8 | raw_temp_data[1]);
	
	if (raw_data & 0x04) Error_Handler();
	else temp_data = ((raw_data>>3) & 0x0FFF);
		
	temp = temp_data*0.25;
	
}