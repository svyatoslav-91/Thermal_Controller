#include <string.h>
#include "flash.h"
#include "pid_controller.h"
#include "PC_program_commands.h"


FlashProfileStorage_t flash_profile;

void Flash_Read_PIDCoef_Storage(PID_Controller *pid)
{
    float pid_cons[5];
    memcpy(pid_cons, (void*)FLASH_PIDCOEF_STORAGE_ADDR, sizeof(pid_cons));

	  pid->pid_calc_freq = pid_cons[0];
	  pid->window_time_ms = pid_cons[1];
    pid->Kp = pid_cons[2];
    pid->Ki = pid_cons[3];
    pid->Kd = pid_cons[4]; 
}

void Flash_Write_PIDCoef_Storage(PID_Controller *pid)
{
	FLASH_EraseInitTypeDef EraseInitStruct = {0};
	uint32_t SectorError = 0;
	uint32_t Address = FLASH_PIDCOEF_STORAGE_ADDR;
	
  // 1 Разблокируем Flash
  HAL_FLASH_Unlock();
	// 2 Настройка структуры стирания
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.Banks = FLASH_BANK_1;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_6;
  EraseInitStruct.NbSectors = 1;
  // 3 Стереть сектор
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
      HAL_FLASH_Lock();
      return; // Ошибка стирания
  }
   // Подготовим массив только из 3-х float
    float pid_cons[5] = { pid->pid_calc_freq, pid->window_time_ms, pid->Kp, pid->Ki, pid->Kd };
    uint32_t data;

    // Записываем их как 32-битные слова
    for (int i = 0; i < 5; i++) {
        memcpy(&data, &pid_cons[i], sizeof(float)); // безопасное преобразование
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, data);
        Address += 4;
    }

    // Блокируем Flash
    HAL_FLASH_Lock();
}

// Чтение профиля из Flash
void Flash_Read_Profile_Storage(FlashProfileStorage_t* profile) 
{
    FlashProfileStorage_t* flash_data = (FlashProfileStorage_t*)FLASH_PROFILE_ADDR;
    
    // Проверяем магическое число и CRC
    if (flash_data->magic == FLASH_MAGIC_NUMBER) {
        uint32_t calculated_crc = Calculate_CRC32((uint8_t*)flash_data, 
                                                sizeof(FlashProfileStorage_t) - sizeof(uint32_t));
        
        if (calculated_crc == flash_data->crc) {
            memcpy(profile, flash_data, sizeof(FlashProfileStorage_t));
            return;
        }
    }
    
    // Если данные невалидны, возвращаем пустой профиль
    profile->magic = 0;
    profile->point_count = 0;
    profile->crc = 0;
}

// Запись профиля во Flash
void Flash_Write_Profile_Storage(FlashProfileStorage_t* profile) 
{
	  profile->magic = FLASH_MAGIC_NUMBER;
	
    // 1 Рассчитываем CRC перед записью
    profile->crc = Calculate_CRC32((uint8_t*)profile, sizeof(FlashProfileStorage_t) - sizeof(uint32_t));
       
	  FLASH_EraseInitTypeDef EraseInitStruct = {0};
	  uint32_t SectorError = 0;
	  uint32_t Address = FLASH_PROFILE_ADDR;
	
    // 2 Разблокируем Flash
    HAL_FLASH_Unlock();
	  // 3 Настройка структуры стирания
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Banks = FLASH_BANK_1;
	  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_7;
    EraseInitStruct.NbSectors = 1;
		
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK) {
        // Записываем данные
        uint32_t* src = (uint32_t*)profile;
        uint32_t* dst = (uint32_t*)FLASH_PROFILE_ADDR;
        uint32_t words = sizeof(FlashProfileStorage_t) / 4;
        
        for (uint32_t i = 0; i < words; i++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)dst, *src) != HAL_OK) {
                break;
            }
            src++;
            dst++;
        }
    }
    
    // Блокируем Flash
    HAL_FLASH_Lock();
}

// Расчет CRC32 (упрощенная версия)
uint32_t Calculate_CRC32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return ~crc;
}
void Flash_Profile_Init(ProfilePoint_t* profile)
{
    profile_point_count = flash_profile.point_count;

    if (profile_point_count > MAX_PROFILE_POINTS)
        profile_point_count = MAX_PROFILE_POINTS;

    memcpy(profile,
           flash_profile.points,
           profile_point_count * sizeof(ProfilePoint_t));
}