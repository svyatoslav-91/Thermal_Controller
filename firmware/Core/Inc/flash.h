#pragma once 
#include "main.h"
#include "pid_controller.h"

#define FLASH_PIDCOEF_STORAGE_ADDR 0x08040000 //Для ПИД коэффициентов
#define FLASH_PROFILE_ADDR  0x08060000  // Для термопрофиля

// Магическое число для проверки валидности данных
#define FLASH_MAGIC_NUMBER 0xDEADBEEF

// Структура для хранения профиля во Flash
typedef struct __attribute__((aligned(4)))
{
    float time_sec;
    float temperature;
} FlashProfilePoint_t;

typedef struct __attribute__((aligned(4)))
{
    uint32_t magic;                    // Магическое число для проверки
    uint32_t point_count;
    FlashProfilePoint_t points[MAX_PROFILE_POINTS];    // Максимум 20 точек
    uint32_t crc;                      // Контрольная сумма
} FlashProfileStorage_t;

// Функции
void Flash_Read_PIDCoef_Storage(PID_Controller* pid);
void Flash_Write_PIDCoef_Storage(PID_Controller* pid);
void Flash_Read_Profile_Storage(FlashProfileStorage_t* profile);
void Flash_Write_Profile_Storage(FlashProfileStorage_t* profile);
uint32_t Calculate_CRC32(const uint8_t* data, size_t length);
void Standart_PIDCoef_Init(PID_Controller *pid);
void Flash_Profile_Init (ProfilePoint_t* profile);

extern FlashProfileStorage_t flash_profile;
