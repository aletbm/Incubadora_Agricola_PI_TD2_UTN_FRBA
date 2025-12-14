/*
 * epprom.c
 *
 *  Created on: Dec 14, 2025
 *      Author: Alexander
 */

#include "eeprom.h"
#include <string.h> // Necesario para memcpy

// --- GESTIÓN DE MEMORIA FLASH (PERSISTENCIA) ---

/**
 * @brief Carga la configuración del sistema desde la memoria Flash
 *
 * Esta función lee la estructura `IncubatorData_t` almacenada en la Flash
 * interna del microcontrolador y la copia a RAM.
 *
 * Antes de cargar los datos, se realiza una validación básica para detectar:
 *  - Memoria virgen (valores 0xFF)
 *  - Datos corruptos o fuera de rango
 *
 * Si la validación falla, se cargan valores de fábrica por defecto.
 *
 * Al finalizar, se actualiza el tick de arranque para el cálculo correcto
 * del tiempo de funcionamiento.
 */
void Load_Config_From_Flash(void)
{
    /* Puntero a los datos almacenados en Flash */
    IncubatorData_t *flash_data =
        (IncubatorData_t *)FLASH_USER_START_ADDR;

    /* Verificación de memoria virgen o datos inválidos */
    if (flash_data->total_days == 0xFF ||
        flash_data->stages[0].temp_target > 100.0f)
    {
        /* =========================
         *   VALORES DE FÁBRICA
         * ========================= */

        /* Duración total del proceso (días) */
        sysData.total_days = 21;

        /* -------- Etapa 0: Desarrollo -------- */
        snprintf(sysData.stages[0].name, 12, "DESARROLLO");
        sysData.stages[0].end_day     = 18;
        sysData.stages[0].temp_target = 37.7f;
        sysData.stages[0].hum_target  = 55.0f;
        sysData.stages[0].motor_on    = 1;

        /* -------- Etapa 1: Eclosión -------- */
        snprintf(sysData.stages[1].name, 12, "ECLOSION");
        sysData.stages[1].end_day     = 21;
        sysData.stages[1].temp_target = 37.2f;
        sysData.stages[1].hum_target  = 70.0f;
        sysData.stages[1].motor_on    = 0;

        /* Inicialización del estado del sistema */
        sysData.is_running      = 0;
        sysData.saved_timestamp = 0;
    }
    else
    {
        /* Copiar datos válidos desde Flash a RAM */
        memcpy(&sysData, flash_data, sizeof(IncubatorData_t));
    }

    /* Registrar tick de arranque para cálculo de tiempo */
    sysData.last_boot_tick = HAL_GetTick();
}

/**
 * @brief Guarda la configuración del sistema en memoria Flash interna
 *
 * Esta función persiste la estructura `sysData` en la memoria Flash del
 * microcontrolador. Antes de escribir, se borra el sector asignado y luego
 * se programa palabra por palabra (32 bits).
 *
 * Previo al guardado, se actualiza el tiempo acumulado de funcionamiento
 * para mantener la continuidad del proceso ante reinicios del sistema.
 *
 * El sector de Flash utilizado está definido por `FLASH_SECTOR_7` y la
 * dirección de inicio por `FLASH_USER_START_ADDR`.
 *
 * En caso de error durante el borrado del sector, la función aborta
 * la operación y bloquea nuevamente la Flash.
 */
void Save_Config_To_Flash(void)
{
    /* Desbloquear memoria Flash para permitir operaciones de borrado/escritura */
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    /* Configuración del borrado del sector de Flash */
    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = FLASH_SECTOR_7;
    EraseInitStruct.NbSectors    = 1;

    /* Borrado del sector asignado */
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    /* Dirección inicial de escritura en Flash */
    uint32_t address = FLASH_USER_START_ADDR;

    /* Puntero a los datos a guardar (cast a palabras de 32 bits) */
    uint32_t *data_ptr = (uint32_t *)&sysData;

    /* Actualización del tiempo acumulado antes de guardar */
    if (sysData.is_running) {
        uint32_t current_session_min =
            (HAL_GetTick() - sysData.last_boot_tick) / 60000U;

        sysData.saved_timestamp += current_session_min;
        sysData.last_boot_tick = HAL_GetTick();
    }

    /* Escritura de la estructura completa palabra por palabra */
    for (int i = 0; i < sizeof(IncubatorData_t); i += 4) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *data_ptr);
        address += 4;
        data_ptr++;
    }

    /* Bloquear nuevamente la Flash para evitar escrituras accidentales */
    HAL_FLASH_Lock();
}


