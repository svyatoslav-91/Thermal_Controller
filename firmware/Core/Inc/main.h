#pragma once 
#include "stm32f4xx_hal.h"
#include "pid_controller.h"

#define Row1_Pin GPIO_PIN_1
#define Row1_GPIO_Port GPIOA
#define Row2_Pin GPIO_PIN_2
#define Row2_GPIO_Port GPIOA
#define Row3_Pin GPIO_PIN_3
#define Row3_GPIO_Port GPIOA
#define Row4_Pin GPIO_PIN_4
#define Row4_GPIO_Port GPIOA
#define CS_MAX6675_Pin GPIO_PIN_7
#define CS_MAX6675_GPIO_Port GPIOA
#define Col1_Pin GPIO_PIN_0
#define Col1_GPIO_Port GPIOB
#define Col2_Pin GPIO_PIN_1
#define Col2_GPIO_Port GPIOB
#define Col3_Pin GPIO_PIN_5
#define Col3_GPIO_Port GPIOB
#define Col4_Pin GPIO_PIN_10
#define Col4_GPIO_Port GPIOB
#define TPC_GPIO_Port GPIOB
#define TPC_Pin GPIO_PIN_4

#define USB_RX_BUFFER_SIZE 64
#define USB_TX_BUFFER_SIZE 64

#define CMD_BUF_SIZE 128

#define MAX_PROFILE_POINTS 50

typedef enum {
    STATE_IDLE,
	  STATE_SET_TEMP,
	  STATE_RUNNING_SETPOINT,
    STATE_RUNNING_PROFILE
} SystemState_t;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern volatile uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE];
extern volatile uint8_t usb_rx_len;
extern float setpoint_temp;
extern volatile SystemState_t current_state;

extern char usb_tx_buffer[USB_TX_BUFFER_SIZE];
extern volatile uint16_t window_time_ms;
extern float setpoint_temp;
extern float temp_avg; 
extern volatile uint8_t temp_ready; 

void SystemClock_Config(void);
void Error_Handler(void);
void process_usb_command(char* command);
void update_profile_setpoint(void);



typedef struct {
    float time_sec;
    float temperature;
} ProfilePoint_t;

