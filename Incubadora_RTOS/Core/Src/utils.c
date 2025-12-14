/*
 * utils.c
 *
 *  Created on: Dec 14, 2025
 *      Author: Alexander
 */

#include "utils.h"
#include "config.h"

IncubatorData_t sysData;

OutputControl_t test_outputs[] = {
    {"COOLER", 0, COOLER_GPIO_Port, COOLER_Pin},  // Index 0: PA8
    {"HUMID.", 0, HUMIDIFICADOR_GPIO_Port, HUMIDIFICADOR_Pin}, // Index 1: PB10
    {"LUZ   ", 0, LAMPARA_GPIO_Port, LAMPARA_Pin},  // Index 2: PB4
    {"MOTOR ", 0, MOTOR_GPIO_Port, MOTOR_Pin}   // Index 3: PB5
};

// VARIABLE GLOBAL PARA LIVE EXPRESSIONS
volatile LiveStatus_t liveStatus = {0};

// Variables Temporales para Editar Tiempo
int16_t edit_day = 1;
int16_t edit_hour = 0;
int16_t edit_min = 0;

// --- HARDWARE HELPERS ---
/**
 * @brief Alterna el estado de una salida digital de prueba
 *
 * Esta función invierte (toggle) el estado lógico de una de las salidas
 * configuradas en el menú de test. El índice recibido identifica qué
 * salida controlar dentro del arreglo `test_outputs`.
 *
 * Luego de actualizar el estado interno, la función refleja el cambio
 * físicamente escribiendo el nivel correspondiente en el pin GPIO.
 *
 * Si el índice está fuera de rango, la función no realiza ninguna acción.
 *
 * @param index Índice de la salida a alternar
 */
void toggle_output(int index)
{
    if(index < 0 || index >= test_menu_size) return;

    // Lógica invertida para relés activos en bajo
    test_outputs[index].state = !test_outputs[index].state;
    // Si state es 1 (ON) -> Mandamos RESET (Low)
    // Si state es 0 (OFF) -> Mandamos SET (High)
    HAL_GPIO_WritePin(test_outputs[index].port, test_outputs[index].pin,
                      test_outputs[index].state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// --- LÓGICA DE TIEMPO ---

/**
 * @brief Obtiene el día actual del proceso en ejecución
 *
 * Esta función calcula el día actual del sistema (por ejemplo, el día de
 * incubación) a partir del tiempo total acumulado en minutos.
 *
 * El tiempo total se obtiene sumando:
 *  - El tiempo previamente guardado en memoria no volátil
 *    (`sysData.saved_timestamp`)
 *  - El tiempo transcurrido desde el último arranque del sistema
 *    (`HAL_GetTick() - sysData.last_boot_tick`)
 *
 * Si el sistema no se encuentra en ejecución, la función retorna 0.
 *
 * @return Día actual (comenzando en 1). Retorna 0 si el sistema está detenido.
 */
uint8_t Get_Current_Day(void)
{
    /* Si el sistema no está en ejecución, no hay día válido */
    if (!sysData.is_running)
        return 0;

    /* Minutos transcurridos desde el último arranque */
    uint32_t session_mins =
        (HAL_GetTick() - sysData.last_boot_tick) / 60000;

    /* Minutos totales acumulados (previos + sesión actual) */
    uint32_t total_mins =
        sysData.saved_timestamp + session_mins;

    /* Conversión de minutos a días (1440 min = 1 día) */
    uint8_t day = (total_mins / 1440) + 1;

    return day;
}

/**
 * @brief Obtiene los valores objetivo activos según el día actual del proceso
 *
 * Esta función determina los valores de consigna (setpoints) de temperatura,
 * humedad y estado del motor en función del día actual del proceso.
 *
 * Los valores se seleccionan según la etapa activa definida en la estructura
 * `sysData.stages`:
 *  - Etapa 0: Desarrollo
 *  - Etapa 1: Eclosión
 *
 * Si el sistema no está en ejecución (día = 0), los valores retornados son cero.
 *
 * @param temp      Puntero donde se almacena la temperatura objetivo (°C)
 * @param hum       Puntero donde se almacena la humedad objetivo (%)
 * @param motor_on  Puntero donde se almacena el estado del motor (0 = OFF, 1 = ON)
 */
void Get_Active_Targets(float *temp, float *hum, uint8_t *motor_on)
{
    /* Obtener el día actual del proceso */
    uint8_t day = Get_Current_Day();

    /* Sistema detenido: no hay consignas activas */
    if (day == 0) {
        *temp = 0;
        *hum  = 0;
        *motor_on = 0;
        return;
    }

    /* Etapa de desarrollo */
    if (day <= sysData.stages[0].end_day) {
        *temp     = sysData.stages[0].temp_target;
        *hum      = sysData.stages[0].hum_target;
        *motor_on = sysData.stages[0].motor_on;
    }
    else {
        /* Etapa de eclosión o final del proceso */
        *temp     = sysData.stages[1].temp_target;
        *hum      = sysData.stages[1].hum_target;
        *motor_on = sysData.stages[1].motor_on;
    }
}


/**
 * @brief Obtiene la hora actual del ciclo de incubación.
 *
 * Esta función calcula la hora actual (0–23) dentro del día del proceso
 * de incubación. El cálculo se basa en el tiempo total acumulado desde
 * el inicio del proceso, combinando:
 *  - El tiempo guardado previamente en memoria Flash
 *  - El tiempo transcurrido desde el último arranque del sistema
 *
 * El tiempo se mide en minutos a partir del contador de milisegundos
 * del sistema (HAL_GetTick()).
 *
 * @note Si el sistema no se encuentra en ejecución (sysData.is_running == 0),
 *       la función devuelve 0.
 *
 * @retval uint8_t Hora actual del día (0–23).
 */
uint8_t Get_Current_Hour(void)
{
    // Si el sistema no está ejecutando la incubación, no hay hora válida
    if (!sysData.is_running) return 0;

    // Calcular los minutos transcurridos desde el último encendido
    // HAL_GetTick() devuelve milisegundos desde el reset del microcontrolador
    uint32_t session_mins = (HAL_GetTick() - sysData.last_boot_tick) / 60000;

    // Sumar los minutos acumulados guardados en Flash
    // con los de la sesión actual
    uint32_t total_mins = sysData.saved_timestamp + session_mins;

    // Obtener la hora actual dentro del día:
    // 1440 minutos = 24 horas
    uint8_t hour = (total_mins % 1440) / 60;

    // Retornar la hora calculada
    return hour;
}


/**
 * @brief Obtiene el minuto actual del ciclo de incubación.
 *
 * Esta función calcula el minuto actual (0–59) dentro de la hora en curso
 * del proceso de incubación, utilizando el tiempo total acumulado desde
 * el inicio del sistema.
 *
 * El cálculo se realiza en minutos y se normaliza dentro del rango de una hora.
 *
 * @note Si el sistema no se encuentra en ejecución (sysData.is_running == 0),
 *       la función devuelve 0.
 *
 * @retval uint8_t Minuto actual (0–59).
 */
uint8_t Get_Current_Minute(void)
{
    // Si la incubación no está activa, devolver 0
    if (!sysData.is_running) return 0;

    // Calcular los minutos transcurridos desde el último arranque
    uint32_t session_mins = (HAL_GetTick() - sysData.last_boot_tick) / 60000;

    // Sumar minutos acumulados guardados con los de la sesión actual
    uint32_t total_mins = sysData.saved_timestamp + session_mins;

    // Retornar el minuto actual dentro de la hora (0–59)
    return (uint8_t)(total_mins % 60);
}
