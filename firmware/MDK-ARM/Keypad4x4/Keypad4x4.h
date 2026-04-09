#pragma once

#include "main.h"

#define ROWS 4
#define COLS 4

typedef enum {
    MODE_NORMAL,
	  MODE_SET_TEMP,
	  MODE_RUN_PROFILE
} KeypadSettingMode;

extern char keymap[ROWS][COLS];
extern char input_buffer[3];
extern uint8_t input_index;

char read_keypad (void);
void process_keypress(char key);





