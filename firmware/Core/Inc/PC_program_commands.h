#pragma once 
#include "main.h"

extern ProfilePoint_t temperature_profile[MAX_PROFILE_POINTS];
extern uint8_t profile_point_count;
extern uint8_t current_profile_point;
extern uint32_t profile_start_time;
extern uint32_t setpoint_start_time;
extern uint8_t profile_running;
extern uint8_t setpoint_running;
extern uint8_t profile_update_needed;

void process_usb_command(char* command);
void send_telemetry(void);
void send_telemetry_quiet(void);
void send_pid_values(void);
void send_state(void);
void start_profile(void);
void stop_profile(void);
void start_setpoint(void);
void stop_setpoint(void);