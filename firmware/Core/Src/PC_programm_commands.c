#include "main.h"
#include "PC_program_commands.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "MAX6675.h"
#include "flash.h"
#include "LCD20x4.h"

ProfilePoint_t temperature_profile[MAX_PROFILE_POINTS];
uint8_t profile_point_count = 0;
uint8_t current_profile_point = 0;
uint32_t setpoint_start_time = 0;
uint32_t profile_start_time = 0;
uint8_t profile_running = 0;
uint8_t setpoint_running = 0;
uint8_t profile_update_needed = 0;

static void trim_crlf(char *s);

// Обработка команд от PC
void process_usb_command(char* command)
{		
	
    if (strncmp(command, "SET_PID:", 8) == 0) {
        // Формат: SET_PID:kp,ki,kd
        float kp, ki, kd;
			  int freq, window;
			
        if (sscanf(command + 8, "%d,%d,%f,%f,%f", &freq, &window, &kp, &ki, &kd) == 5) 
				{
					  heater.pid_calc_freq = freq;
					  heater.window_time_ms = window;
            heater.Kp = kp;
            heater.Ki = ki;
            heater.Kd = kd;
					
					  Init_Pid_Variables(&heater);
            // Отправляем отчет
            snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "PID_SET:%d,%d,%.3f,%.3f,%.3f\r\n", freq, window, kp, ki, kd);
            CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
        }
    }
    else if (strncmp(command, "SAVE_PID", 8) == 0) {
        // сохранение во Flash МК
		    Flash_Write_PIDCoef_Storage(&heater);
			  snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "PID_SAVED\r\n");
        CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
    }
    else if (strncmp(command, "GET_PID", 7) == 0) {
        send_pid_values();
    }
else if (strncmp(command, "SET_PROFILE:", 12) == 0) {
    char *data = (char*)(command + 12);
    trim_crlf(data);               // убираем \r\n в конце строки
    profile_point_count = 0;
    current_profile_point = 0;

    // Разбираем строки формата "0,30;60,25;120,40"
    char *token = strtok(data, ";");
    while (token != NULL && profile_point_count < MAX_PROFILE_POINTS) {
        trim_crlf(token);          // убираем возможные символы конца строки
        float time, temp;
        if (sscanf(token, "%f,%f", &time, &temp) == 2) {
            // записываем точку без проверки температуры
            temperature_profile[profile_point_count].time_sec = time;
            temperature_profile[profile_point_count].temperature = temp;
            profile_point_count++;
        }
        token = strtok(NULL, ";");
    }

    snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "PROFILE_RECEIVED\r\n");
    CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
}
		else if (strncmp(command, "SAVE_PROFILE", 12) == 0) 
		{
    FlashProfileStorage_t flash_profile;
    flash_profile.point_count = profile_point_count;
    for (int i = 0; i < profile_point_count; i++) {
        flash_profile.points[i].time_sec = temperature_profile[i].time_sec;
        flash_profile.points[i].temperature = temperature_profile[i].temperature;
    }
    Flash_Write_Profile_Storage(&flash_profile);
    
    snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "PROFILE_FLASH_SAVED\r\n");
    CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
    }
    else if (strncmp(command, "GET_PROFILE", 11) == 0)
    {
    char temp_buf[64];
    usb_tx_buffer[0] = '\0'; // очистка

    for (int i = 0; i < profile_point_count; i++) {
        snprintf(temp_buf, sizeof(temp_buf),
                 "POINT:%d,%.1f,%.1f\r\n",
                 i,
                 temperature_profile[i].time_sec,
                 temperature_profile[i].temperature);

        // Проверка, не переполняем ли буфер
        if (strlen(usb_tx_buffer) + strlen(temp_buf) < sizeof(usb_tx_buffer)) {
            strcat(usb_tx_buffer, temp_buf);
        } else {
            // буфер почти полный — отправляем частично
            while (CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer)) == USBD_BUSY);
            HAL_Delay(5);
            usb_tx_buffer[0] = '\0';
            strcat(usb_tx_buffer, temp_buf);
        }
    }
    // отправить остаток
    while (CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer)) == USBD_BUSY);
		snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "PROFILE_SENT\r\n");
    while (CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer)) == USBD_BUSY);
    }
    else if (strncmp(command, "START_PROFILE", 13) == 0) {
        start_profile();
    }
    else if (strncmp(command, "STOP_PROFILE", 12) == 0) {
        stop_profile();
    }
    else if (strncmp(command, "TEMP_POINT_SET:", 15) == 0) {
    char* temp_str = command + 15;
    
    // ЗАМЕНЯЕМ ЗАПЯТУЮ НА ТОЧКУ перед парсингом
    char temp_buffer[32];
    strncpy(temp_buffer, temp_str, sizeof(temp_buffer) - 1);
    temp_buffer[sizeof(temp_buffer) - 1] = '\0';
    
    for (char *p = temp_buffer; *p; p++) {
        if (*p == ',') *p = '.';
    }
    float program_setpoint_temp = atof(temp_buffer);
    if (program_setpoint_temp >= 0.0f && program_setpoint_temp <= 1500.0f) {
        setpoint_temp = program_setpoint_temp;
        snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "SETPOINT_SET:%.1f\r\n", program_setpoint_temp);
        CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
    } else {
        snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "ERROR:Invalid temperature\r\n");
        CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
    }
    }
		else if (strncmp(command, "START_SETPOINT", 14) == 0) {
        start_setpoint();
    }
		else if (strncmp(command, "STOP_SETPOINT", 14) == 0) {
        stop_setpoint();
    }
    else if (strncmp(command, "GET_STATUS", 10) == 0) {
        send_telemetry();
        send_state();
    }
		else if (strncmp(command, "GET_TELEMETRY", 10) == 0) {
        send_telemetry_quiet();
    }
    else {
        // Неизвестная команда
        snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "UNKNOWN_CMD:%s\r\n", command);
        CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
    }
}

// Отправка телеметрии
void send_telemetry(void)
{
    float current_temp;

    if (temp_ready) current_temp = temp_avg;   // готовое усреднённое значение
    //else current_temp = temp;       // сырое значение, пока нет 5 измерений
    
		snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "TEMP:%.3f\n", current_temp);
    CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
    HAL_Delay(10);
	
    // Всегда отправляем setpoint (если активен)
    if (current_state == STATE_RUNNING_PROFILE || current_state == STATE_RUNNING_SETPOINT) {
        snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "SETPOINT:%.3f\n", setpoint_temp);
        CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
        HAL_Delay(5);
    }
    if (current_state == STATE_RUNNING_SETPOINT) {
        uint32_t setpoint_time = (HAL_GetTick() - setpoint_start_time) / 1000;
        snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "SETPOINT_TIME:%d\n", setpoint_time);
        CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
        HAL_Delay(2);
    }
    // Время профиля только при выполнении профиля
    if (current_state == STATE_RUNNING_PROFILE) {
        uint32_t profile_time = (HAL_GetTick() - profile_start_time) / 1000;
        snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "PROFILE_TIME:%d\n", profile_time);
        CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
        HAL_Delay(5);
    }
}

void send_telemetry_quiet(void)
{
	  float current_temp;
    char buffer[64];
    if (temp_ready) current_temp = temp_avg;
    //else current_temp = temp;
    
    // Температура
    snprintf(buffer, sizeof(buffer), "TEMP:%.3f\n", current_temp);
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
    HAL_Delay(2);
    
    // Setpoint (если активен)
    if (current_state == STATE_RUNNING_PROFILE || current_state == STATE_RUNNING_SETPOINT) {
        snprintf(buffer, sizeof(buffer), "SETPOINT:%.3f\r\n", setpoint_temp);
        CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
        HAL_Delay(2);
    }
		if (current_state == STATE_RUNNING_SETPOINT) {
        uint32_t setpoint_time = (HAL_GetTick() - setpoint_start_time) / 1000;
        snprintf(buffer, sizeof(buffer), "SETPOINT_TIME:%d\n", setpoint_time);
        CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
        HAL_Delay(2);
    }
    // Время профиля
    if (current_state == STATE_RUNNING_PROFILE) {
        uint32_t profile_time = (HAL_GetTick() - profile_start_time) / 1000;
        snprintf(buffer, sizeof(buffer), "PROFILE_TIME:%d\n", profile_time);
        CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
        HAL_Delay(2);
    }
}

// Отправка PID коэффициентов
void send_pid_values(void)
{
    snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "PID:%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", 
             heater.pid_calc_freq, heater.window_time_ms, heater.Kp, heater.Ki, heater.Kd, heater.integral, heater.prev_error, heater.output_min, heater.output_max);
    CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
}

// Отправка состояния системы
void send_state(void)
{
    const char* state_str;
    switch(current_state) {
        case STATE_IDLE: state_str = "IDLE\r\n"; break;
			  case STATE_RUNNING_SETPOINT: state_str = "RUNNING SETPOINT\r\n"; break;
        case STATE_RUNNING_PROFILE: state_str = "RUNNING PROFILE\r\n"; break;
        default: state_str = "UNKNOWN\r\n"; break;
    }
    
    snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "STATE:%s", state_str);
    CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));
}

// Управление профилем
void start_profile(void)
{
    if (current_state == STATE_IDLE || current_state == STATE_RUNNING_SETPOINT) {
        current_state = STATE_RUNNING_PROFILE;
        profile_running = 1;
        profile_start_time = HAL_GetTick();
        send_state();
				LCD_Clear();
	      LCD_PrintString("PROFILE START");
			  HAL_Delay(1000);
    }
}
void stop_profile(void)
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
void start_setpoint(void)
{ 
	if (current_state == STATE_IDLE || current_state == STATE_RUNNING_PROFILE) {
      current_state = STATE_RUNNING_SETPOINT;
      setpoint_running = 1;
		  setpoint_start_time = HAL_GetTick();
      send_state();
		  LCD_Clear();
	    LCD_PrintString("SETPOINT START");
			HAL_Delay(1000);
    }	
}
void stop_setpoint(void)
{ 
	  current_state = STATE_IDLE;
	  setpoint_running = 0;
    send_state();
	  LCD_Clear();
	  LCD_PrintString("SETPOINT STOPPED");
	  HAL_Delay(1000);
}

static void trim_crlf(char *s) {
    if (!s) return;
    size_t len = strlen(s);
    while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n' || s[len-1] == ' '))
        s[--len] = '\0';
}