#include "Keypad4x4.h"
#include "LCD20x4.h"
#include "PC_program_commands.h"

char keymap[ROWS][COLS] = {
  {'1','2','3','A'},
	{'4','5','6','B'},
	{'7','8','9','C'},
	{'*','0','#','D'},
	
};

char input_buffer[3]={0};
uint8_t input_index = 0;

 char read_keypad (void) 
 {
	 GPIO_TypeDef* row_ports[ROWS] = {Row1_GPIO_Port,Row2_GPIO_Port,Row3_GPIO_Port,Row4_GPIO_Port};
	 uint16_t row_pins[ROWS] = {Row1_Pin,Row2_Pin,Row3_Pin,Row4_Pin};
	 
	 GPIO_TypeDef* col_ports[COLS] = {Col1_GPIO_Port,Col2_GPIO_Port,Col3_GPIO_Port,Col4_GPIO_Port};
	 uint16_t col_pins[COLS] = {Col1_Pin,Col2_Pin,Col3_Pin,Col4_Pin};
	 
	 for (int row=0; row <ROWS; row++) 
	 {
		 HAL_GPIO_WritePin(row_ports[row],row_pins[row],GPIO_PIN_RESET);
		 HAL_Delay(1);
		 for (int col=0; col <COLS;col++) 
		 {
			 if(HAL_GPIO_ReadPin(col_ports[col],col_pins[col]) == GPIO_PIN_RESET)
			 {
			 HAL_Delay(150); //Антидребезг
				   if (HAL_GPIO_ReadPin(col_ports[col], col_pins[col]) == GPIO_PIN_RESET)
           {
				       while(HAL_GPIO_ReadPin(col_ports[col],col_pins[col]) == GPIO_PIN_RESET)
					     {
				       	   HAL_GPIO_WritePin(row_ports[row],row_pins[row],GPIO_PIN_SET);
						       return keymap[row][col];
				       }
			   }
			 }
		 }
		 HAL_GPIO_WritePin(row_ports[row],row_pins[row],GPIO_PIN_SET);
	 }
	  	 return 0;
 }
 
 void process_keypress(char key)
 {
 if (key==0) return;
	 switch (current_state) 
	 {
		 case STATE_IDLE:
			 
			   profile_running = 0;
		 
		     if(key=='A') 
			   {   
					   current_state = STATE_SET_TEMP;
					   LCD_Clear();
					   LCD_PrintString("Required temp: dC");
					   
			   }
				 if(key=='D') 
			   {   
					   start_profile();
				 }
		 break;
				 
		 case STATE_SET_TEMP:

			   	   if(key>='0' && key<='9')
						 {
							   if(input_index < 3 )
								 {  
								     input_buffer[input_index] = key;
							       LCD_SetCursor(15 + input_index,0);
							       LCD_PrintChar(key);
									   input_index++;
									}
						     							    
						 }
						 else if (key == 'C')
						 {
						     HAL_Delay(300);
                 float new_setpoint_temp = 0;
                 for (int i = 0; i < input_index; i++) new_setpoint_temp = new_setpoint_temp * 10 + (input_buffer[i] - '0');
                 if (new_setpoint_temp < 0 || new_setpoint_temp > 1500) 
								 {
								     LCD_Clear();
									   LCD_PrintString("Error: Temp 0-1500C");
									   HAL_Delay(2000);
									   input_index = 0;
									   LCD_Clear();
									   LCD_PrintString("Required temp: dC");
									   break;
                }
                setpoint_temp = new_setpoint_temp;
								input_index = 0;
								LCD_Clear();
								LCD_PrintString("New SPT has been set");	
								HAL_Delay(1000);
								LCD_Clear();					
							  current_state = STATE_RUNNING_SETPOINT;		 
						}
						else if (key == 'B')
						{
					      input_index = 0;
							  for (int i = 0; i < input_index; i++) input_buffer[i] = 0;
								LCD_Clear();
					      LCD_PrintString("Required temp: dC");
						}
						else if (key == 'D') // Назад
						{   
							   LCD_Clear();
							   input_index = 0; 
						     current_state = STATE_IDLE;
						}
				    break;
			
		 case STATE_RUNNING_SETPOINT:	
			    setpoint_running = 1;
		      if(key=='B') 
			    {   
						 setpoint_running = 0;
					   current_state = STATE_IDLE;
					   LCD_Clear();
					   LCD_PrintString("SETPOINT STOPPED");
						 HAL_Delay(1000);
					}
          break;			 
     case STATE_RUNNING_PROFILE:
					 profile_running = 1;
           profile_start_time = HAL_GetTick();
		 		   if(key=='B') 
			     {   
						 current_state = STATE_IDLE;
						 profile_running = 0;
						 setpoint_temp = 30.0f; // Возвращаем к безопасной температуре
					   current_profile_point = 0;
						 send_state();
					   LCD_Clear();
					   LCD_PrintString("PROFILE STOPPED");
						 HAL_Delay(1000);
					 }
				   break;
   }
}
