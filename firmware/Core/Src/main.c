#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "gpio.h"
#include "i2c1.h"
#include "spi1.h"
#include "tim.h"
#include "pid_controller.h"
#include "flash.h"
#include "idwg.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "MAX6675.h"
#include "LCD20x4.h"
#include "Keypad4x4.h"
#include "PC_program_commands.h"

const char* startup_msg = "\r\nReflow Controller Ready\r\n";

volatile SystemState_t current_state = STATE_IDLE;
SystemState_t previous_state = STATE_IDLE;

// Буферы для USB коммуникации

volatile uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE] = {0};
volatile uint8_t usb_rx_len = 0;
char usb_tx_buffer[USB_TX_BUFFER_SIZE] = {0};

// Флаги и переменны для расчета ПИД
float setpoint_temp = 30.0f;
static uint16_t on_steps = 0;
static uint16_t tim3_counter = 0;
static uint8_t need_new_pid_calc = 0;

// Буфер усреднения температуры
static float temp_buffer[5] = {0};
static uint8_t temp_index = 0;
static uint8_t temp_count = 0;

float temp_raw = 0.0f;     // Последнее измерение MAX6675
float temp_avg = 0.0f;     // Усреднённая температура

volatile uint8_t temp_read_needed = 0;
volatile uint8_t temp_ready = 0;   // Становится 1 когда набрано 5 измерений

void add_temperature_sample(float t)
{
    temp_buffer[temp_index] = t;
    temp_index = (temp_index + 1) % 3;

    if (temp_count < 3) temp_count++;

    if (temp_count == 3)
    {
        float sum = 0.0f;
        for (uint8_t i = 0; i < 3; i++) sum += temp_buffer[i];

        temp_avg = sum / 3.0f;
        temp_ready = 1;
    }
    else
    {
        temp_ready = 0;
    }
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

void OTG_FS_IRQHandler(void)
{

  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);

}
// Обновление уставки по профилю
void update_profile_setpoint(void)
{
    if (!profile_running || profile_point_count == 0) return;
    
    float elapsed_time = (HAL_GetTick() - profile_start_time) / 1000.0f; // в секундах
	
    // Если вышли за последнюю точку — ставим на последнюю
    if (elapsed_time >= (uint32_t)temperature_profile[profile_point_count - 1].time_sec) {
        current_profile_point = profile_point_count - 1;
        setpoint_temp = temperature_profile[current_profile_point].temperature;
        stop_profile();
        return;
    }
	  // Проверяем, чтобы current_profile_point не вылез за границы
    if (current_profile_point >= profile_point_count) current_profile_point = profile_point_count - 1;
    
    // Находим текущую точку профиля
    while (current_profile_point < profile_point_count - 1 &&
			     elapsed_time >= temperature_profile[current_profile_point + 1].time_sec) current_profile_point++; 
    
    // Интерполяция между точками
    if (current_profile_point < profile_point_count - 1) 
		{
        float t1 = temperature_profile[current_profile_point].time_sec;
        float t2 = temperature_profile[current_profile_point + 1].time_sec;
        float temp1 = temperature_profile[current_profile_point].temperature;
        float temp2 = temperature_profile[current_profile_point + 1].temperature;

        if (t2 > t1) {
            float ratio = (elapsed_time - t1) / (t2 - t1);
            setpoint_temp = temp1 + (temp2 - temp1) * ratio;
        } else setpoint_temp = temp1;

    } 
		else setpoint_temp = temperature_profile[profile_point_count - 1].temperature;
     // Если профиль завершен
    if (elapsed_time >= temperature_profile[profile_point_count - 1].time_sec) {
        stop_profile();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{   
		  static uint32_t temp_timer = 0;
      temp_timer++;
        
      if (temp_timer >= 20) 
		  {  // Устанавливаем флаг каждые 200 мс
          temp_timer = 0;
          temp_read_needed = 1;
				
      }
				
		  static uint32_t pid_control_counter = 0;
		  pid_control_counter++;
		
		  if (pid_control_counter >= pid_calls_countaty) 
		  {
          pid_control_counter = 0;  
          need_new_pid_calc = 1;
      }
      // Управление счетчиком ШИМ
      static uint16_t local_tim3_counter = 0;
      local_tim3_counter++;
      if (local_tim3_counter >= steps_in_window) 
			{
          local_tim3_counter = 0;
      }
      tim3_counter = local_tim3_counter;  // Копируем в общую переменную
			
			// Обновление уставки по профилю
      if (profile_running && current_state == STATE_RUNNING_PROFILE) 
			{
          static uint32_t last_profile_update = 0;
          uint32_t current_time = HAL_GetTick();
          if (current_time - last_profile_update >= 1000) 
					{ // Обновляем каждую секунду
              last_profile_update = current_time;
              profile_update_needed = 1;
          }
      }
	 }
}

int main(void)
{
  HAL_Init();
	
  SystemClock_Config();
	
  GPIO_Init();
	TIM3_Init();
  I2C1_Init();
  SPI1_Init();
	USB_DEVICE_Init();
	IWDG_Init();
  LCD_Init();
	
	// Иниализируем ПИД контроллер стандартными значениями
	PID_Standart_Init (&heater);

	// Загружаем PID коэффициенты
	Flash_Read_PIDCoef_Storage(&heater);
	
  if (isnan(heater.Kp) || isnan(heater.Ki) || isnan(heater.Kd)) 
	{
    PID_Standart_Init (&heater);
  }
		// Загружаем сохраненный профиль
  Flash_Read_Profile_Storage(&flash_profile);
	
  Flash_Profile_Init(temperature_profile);
			
	CDC_Transmit_FS((uint8_t*)startup_msg, strlen(startup_msg));
  HAL_Delay(10);  // Дать время на отправку
	
	LCD_Clear();
	LCD_SetCursor(0, 0);
	LCD_PrintString("System Ready");
  HAL_Delay(1000);

  while (1)
  {	
		// Чтение температуры по флагу из прерывания
    if (temp_read_needed) {
        temp_read_needed = 0;
			  read_max6675();
			  temp_raw = temp;      // копируем для ясности
        add_temperature_sample(temp_raw);
    }
		//Обработка команд с клавиатуры
		char key = read_keypad();
		process_keypress(key);
		//Обработка команд от программы
		if (cmd_ready) 
		{
    cmd_ready = 0;
    process_usb_command(pending_cmd);
    }
		
		// Сброс ПИД при смене состояния
    if (previous_state != current_state) {
        PID_Clear(&heater);
        previous_state = current_state;  // Обновляем предыдущее состояние
    }
		   
		    if (current_state == STATE_IDLE)
		    {
			      //LCD_Clear();
					  HAL_GPIO_WritePin(TPC_GPIO_Port,TPC_Pin, GPIO_PIN_RESET);
	          LCD_SetCursor(0, 0);
	          LCD_PrintString("'A' to set temp");
			      LCD_SetCursor(0, 2);
	          LCD_PrintString("'D' to start profile");
					  LCD_SetCursor(0, 3);
					  float displayed_temp = temp_avg;
  				  snprintf(lcd_buffer, sizeof(lcd_buffer), "Temp:%.1fC", displayed_temp);
  					LCD_PrintString(lcd_buffer);
		    }
				
			  if (current_state == STATE_RUNNING_SETPOINT || current_state == STATE_RUNNING_PROFILE)
		    {	
			      //static uint16_t last_tim3_counter = 0;
	          static uint8_t power_percentage = 0;
			      if (profile_update_needed) 
						{
                profile_update_needed = 0;
                update_profile_setpoint();
            }
		        if (need_new_pid_calc == 1) 
		        {
                need_new_pid_calc = 0;
													
                float current_temp = temp_avg;
            			
                if (temp_ready) 
						    {
                    float pid_output = PID_Calculate(&heater, setpoint_temp, current_temp, dt);
                    power_percentage = (uint8_t)pid_output;
                }	
					  }
						else power_percentage = 0; // Пока нет 5 измерений - отключаем нагрев или используем минимальную мощность

            on_steps = (power_percentage * steps_in_window)/100;  
		
		        if (tim3_counter < on_steps) 
		   			{ 
		            HAL_GPIO_WritePin(TPC_GPIO_Port,TPC_Pin, GPIO_PIN_SET);
			          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		        }
		        else 
		        {
		            HAL_GPIO_WritePin(TPC_GPIO_Port,TPC_Pin, GPIO_PIN_RESET);
			          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		        }
												
             if (current_state == STATE_RUNNING_SETPOINT)
						 {
		             LCD_SetCursor(0, 0);
							   snprintf(lcd_buffer, sizeof(lcd_buffer), "Set_temp:%.1fC", setpoint_temp);
  							 LCD_PrintString(lcd_buffer);
							 
							   float displayed_temp;
							   if(temp_ready) displayed_temp = temp_avg;
  							 LCD_SetCursor(0, 1);
  							 snprintf(lcd_buffer, sizeof(lcd_buffer), "Temp:%.1fC", displayed_temp);
  							 LCD_PrintString(lcd_buffer);
		
							   LCD_SetCursor(0, 2);
							 	 snprintf(lcd_buffer, sizeof(lcd_buffer), "PWR:%2d%% CNT:%4d", power_percentage, tim3_counter);
  							 LCD_PrintString(lcd_buffer);

							 	 LCD_SetCursor(0, 3);  
  							 snprintf(lcd_buffer, sizeof(lcd_buffer), "ON:%4d/%4d", on_steps, steps_in_window);
  							 LCD_PrintString(lcd_buffer);
						 }
						 if (current_state == STATE_RUNNING_PROFILE)
						 {
						     LCD_SetCursor(0, 0);
  							 snprintf(lcd_buffer, sizeof(lcd_buffer), "Point:%d", current_profile_point);
							   LCD_PrintString(lcd_buffer);
						
							   float displayed_temp;
							   if(temp_ready) displayed_temp = temp_avg;
							   LCD_SetCursor(0, 1);
							   snprintf(lcd_buffer, sizeof(lcd_buffer), "Temp:%.1fC", displayed_temp);
							   LCD_PrintString(lcd_buffer);
		
				         LCD_SetCursor(0, 2);
							   uint32_t current_profile_time = (HAL_GetTick() - profile_start_time)/1000;
							   snprintf(lcd_buffer, sizeof(lcd_buffer), "CurTime:%d",current_profile_time);
							   LCD_PrintString(lcd_buffer);

						     LCD_SetCursor(0, 3);  
						     snprintf(lcd_buffer, sizeof(lcd_buffer), "Spoint:%.1fC ", setpoint_temp);
						     LCD_PrintString(lcd_buffer);
						 }
	     }
		HAL_IWDG_Refresh(&hiwdg);
   } 
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}


void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(100);
  }

}