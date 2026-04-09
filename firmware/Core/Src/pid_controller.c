#include "main.h"
#include "pid_controller.h"

// Настройки ПИД регуляции
// Объявляем переменные
volatile uint16_t pid_timer_period_ms = 10;
volatile uint16_t pid_calls_countaty;
volatile uint16_t steps_in_window;
volatile float dt;

PID_Controller heater;

// Функция инициализации PID параметров
void Init_Pid_Variables(PID_Controller* pid) 
{
	  steps_in_window = pid->window_time_ms/pid_timer_period_ms;
    pid_calls_countaty = 1000 / pid_timer_period_ms / pid->pid_calc_freq;
	  dt = 1.0f/pid->pid_calc_freq; // период расчета ПИД
}

void PID_Standart_Init (PID_Controller* pid)
{   
	  pid->pid_calc_freq = 30; // значения по умолчанию стандартные
	  pid->window_time_ms = 5000;
    pid->Kp = 1.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
	  pid->integral = 0;        // Интегральная составляющая
    pid->prev_error = 0;       // Предыдущая ошибка
    pid->output_min = 0;      // Минимальный выход (0%)
    pid->output_max =100;      // Максимальный выход (100%)
	
	  Init_Pid_Variables(pid); 
}

void PID_Clear (PID_Controller* pid)
{   
	  pid->integral = 0;        // Интегральная составляющая
    pid->prev_error = 0;       // Предыдущая ошибка
}

float PID_Calculate(PID_Controller* pid, float setpoint, float measured, float dt) 
{
    float error = setpoint - measured;
    
    // Пропорциональная составляющая
    float proportional = pid->Kp * error;
    
    // Интегральная составляющая (с защитой от насыщения)
    pid->integral += error * dt;
    if (pid->integral > 100) pid->integral = 100;
    if (pid->integral < -100) pid->integral = -100;
    float integral = pid->Ki * pid->integral;
    
    // Дифференциальная составляющая
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // Суммируем и ограничиваем выход
    float output = proportional + integral + derivative;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    
    return output;
}


