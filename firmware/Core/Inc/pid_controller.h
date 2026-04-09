#pragma once 
#include "main.h"

typedef struct {
	  uint16_t pid_calc_freq;
	  uint16_t window_time_ms;
    float Kp, Ki, Kd;      // Коэффициенты
    float integral;         // Интегральная составляющая
    float prev_error;       // Предыдущая ошибка
    float output_min;       // Минимальный выход (0%)
    float output_max;       // Максимальный выход (100%)
} PID_Controller;

void PID_Standart_Init (PID_Controller* pid);
void Init_Pid_Variables(PID_Controller* pid);
void PID_Clear (PID_Controller* pid);
float PID_Calculate(PID_Controller* pid, float setpoint, float measured, float dt);

extern PID_Controller heater;
extern volatile uint16_t pid_calc_freq;
extern volatile uint16_t pid_calls_countaty;
extern volatile uint16_t steps_in_window;
extern volatile float dt;