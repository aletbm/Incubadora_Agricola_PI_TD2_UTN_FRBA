/*
 * utils.h
 *
 *  Created on: Dec 14, 2025
 *      Author: Alexander
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>
#include "stm32f446xx.h"

typedef struct {
    char* name;          // Nombre a mostrar
    uint8_t state;       // 0 = OFF, 1 = ON (Lógico)
    GPIO_TypeDef* port;  // Puerto GPIO
    uint16_t pin;        // Pin GPIO
} OutputControl_t;

typedef struct {
    char name[12];      // "DESARROLLO"
    uint8_t end_day;    // Fin de etapa (ej. 18)
    float temp_target;  // Ej. 37.7
    float hum_target;   // Ej. 55.0
    uint8_t motor_on;   // 1 = Volteo activo
} StageConfig_t;

typedef struct {
    uint8_t total_days;      // Ciclo total (21)
    StageConfig_t stages[2]; // [0]=Desarrollo, [1]=Eclosión

    // Estado Persistente
    uint8_t is_running;         // 0=Stop, 1=Run
    uint32_t saved_timestamp;   // Minutos acumulados guardados
    uint32_t last_boot_tick;    // Referencia de tiempo (HAL_GetTick)

    // Auxiliar UI
    uint8_t current_stage_idx;  // Qué etapa estamos editando
} IncubatorData_t;

// Estructura para monitoreo en vivo (Live Expressions)
typedef struct {
    float temp_current;
    float hum_current;
    uint16_t rpm_current;
    uint8_t day_current;
    uint8_t hour_current;
    // Debug variables
    float temp_target;
    float hum_target;
} LiveStatus_t;

void toggle_output(int index);
uint8_t Get_Current_Day(void);
uint8_t Get_Current_Hour(void);
uint8_t Get_Current_Minute(void);
void Get_Active_Targets(float *temp, float *hum, uint8_t *motor_on);

extern IncubatorData_t sysData;
extern OutputControl_t test_outputs[];
extern volatile LiveStatus_t liveStatus;
extern int16_t edit_day;
extern int16_t edit_hour;
extern int16_t edit_min;

#endif /* INC_UTILS_H_ */
