/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Firmware Incubadora - Final (Lógica Control + Flash + UI Config)
  * - Integración completa de persistencia en Flash (Sector 7 F446RE).
  * - Control de Temperatura con Soft-PWM.
  * - Control de Humedad por estados.
  * - Menú de Configuración editable.
  * - Corrección de brillo LCD (Update on Change).
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "i2c-lcd.h"
#include "eeprom.h"
#include "utils.h"
#include "config.h"
#include "rele.h"
#include "dht11.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

TaskHandle_t menuTaskHandle;
TaskHandle_t debounceTaskHandle;
TaskHandle_t motorTaskHandle;
TaskHandle_t controlTaskHandle;
TaskHandle_t sensorTaskHandle;

QueueHandle_t menuQueueHandle;
SemaphoreHandle_t xDHTMutex;

// --- VARIABLES DE SISTEMA ---
volatile uint32_t motor_pulse_count = 0;
uint16_t global_rpm = 0;
volatile uint8_t is_welcome = 1;

// --- VARIABLES DE CONTROL ---
float last_valid_temp = 0.0f;
float last_valid_hum = 0.0f;
uint32_t last_dht_read_time = 0;
HumidifierState_t hum_state = HUM_STATE_IDLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM12_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Tarea principal de gestión de menú e interfaz de usuario.
 *
 * Esta tarea maneja:
 *  - Eventos del encoder rotativo y botón
 *  - Navegación entre modos de UI
 *  - Edición de parámetros del sistema
 *  - Actualización eficiente del LCD (update-on-change)
 *
 * La comunicación con las ISR se realiza mediante una cola RTOS.
 *
 * @param argument Parámetro no utilizado
 */

void StartMenuTask(void *argument)
{
    /* Mensaje de bienvenida */
    if (is_welcome) {
        LCD_ShowWelcome("TDII: INCUBADORA");
        vTaskDelay(pdMS_TO_TICKS(1000));
        is_welcome = 0;
    }

    /* Inicializa la pantalla al arrancar la tarea */
    update_display();

    MenuEvent_t event;
    uint32_t last_draw_time = 0;   // Último refresco del LCD
    int needs_update = 0;          // Flag para redibujar pantalla

    /* Variables para actualización solo ante cambios (Dashboard) */
    float last_disp_temp = -999.0f;
    float last_disp_hum  = -999.0f;
    uint16_t last_disp_rpm = 9999;
    HumidifierState_t last_disp_hum_state = HUM_STATE_IDLE;
    uint8_t last_disp_day = 255;
    uint8_t last_disp_is_running = 255;
    uint8_t last_disp_hour = 255;

    for (;;)
    {
        /* Espera evento del menú (encoder / botón)
           Timeout de 200 ms para permitir refresco periódico del dashboard */
        if (xQueueReceive(menuQueueHandle, &event, pdMS_TO_TICKS(200)) == pdPASS)
        {
            MenuEvent_t initial_event = event;

            /* -------- BOTÓN: PRESIÓN LARGA (volver atrás) -------- */
            if (initial_event == BUTTON_LONG_PRESS) {
                // Si estamos en cualquier sub-menú de edición o test, volvemos al menú principal
                if (current_ui_mode == UI_MODE_TEST_MENU ||
                    current_ui_mode == UI_MODE_CONFIG_EDIT ||
                    current_ui_mode == UI_MODE_CONFIG_GLOBAL ||
                    current_ui_mode == UI_MODE_CONFIG_TIME ||
                    current_ui_mode == UI_MODE_CONFIG_SELECT)
                {
                    current_ui_mode = UI_MODE_MAIN_MENU;
                }
                else if (current_ui_mode == UI_MODE_MAIN_MENU) {
                    current_ui_mode = UI_MODE_DASHBOARD;
                }

                needs_update = 1;
                continue; // Saltar el resto del bucle
            }

            /* -------- PROCESAMIENTO DE MOVIMIENTO -------- */
            int8_t net_movement = 0;
            int button_pressed = 0;

            if (initial_event == ENCODER_RIGHT) net_movement++;
            else if (initial_event == ENCODER_LEFT) net_movement--;
            else if (initial_event == BUTTON_PRESS) button_pressed = 1;

            /* Acumula eventos rápidos del encoder (anti-lag UI) */
            int batch_limit = 20;
            while (batch_limit-- > 0 && xQueueReceive(menuQueueHandle, &event, 0) == pdPASS)
            {
                if (event == ENCODER_RIGHT) net_movement++;
                else if (event == ENCODER_LEFT) net_movement--;
            }

            /* ================= LÓGICA DE INTERFAZ ================= */

            /* -------- DASHBOARD -------- */
            if (current_ui_mode == UI_MODE_DASHBOARD) {
                if (button_pressed) {
                    current_ui_mode = UI_MODE_MAIN_MENU;
                    selected_item = 0;
                    needs_update = 1;
                }
            }

            /* -------- MENÚ PRINCIPAL -------- */
            else if (current_ui_mode == UI_MODE_MAIN_MENU) {

                if (net_movement != 0) {
                    int16_t new_pos = selected_item + net_movement;
                    selected_item = ((new_pos % menu_size) + menu_size) % menu_size;

                    if (selected_item >= menu_top_item + LCD_ROWS)
                        menu_top_item = selected_item - (LCD_ROWS - 1);
                    else if (selected_item < menu_top_item)
                        menu_top_item = selected_item;

                    needs_update = 1;
                }

                if (button_pressed) {
                    if (selected_item == 1) { // INICIAR / PAUSAR
                        /* OPTIMIZACIÓN: Calcular y Guardar UNA sola vez */
                        if (sysData.is_running) {
                            // Pausando: Calcular tiempo transcurrido y sumar
                            uint32_t current_session_min = (HAL_GetTick() - sysData.last_boot_tick) / 60000U;
                            sysData.saved_timestamp += current_session_min;
                            sysData.is_running = 0;
                        } else {
                            // Iniciando: Resetear tick de referencia
                            sysData.is_running = 1;
                            sysData.last_boot_tick = HAL_GetTick();
                        }
                        // Guardado único para no congelar la UI dos veces
                        Save_Config_To_Flash();
                    }
                    else if (selected_item == 2) { // CONFIGURACION
                        current_ui_mode = UI_MODE_CONFIG_SELECT;
                        config_sel_index = 0;
                        config_top_index = 0;
                    }
                    else if (selected_item == 3) { // TEST
                        current_ui_mode = UI_MODE_TEST_MENU;
                        test_selected_item = 0;
                    }
                    else { // VOLVER / VER SENSORES
                        current_ui_mode = UI_MODE_DASHBOARD;
                    }
                    needs_update = 1;
                }
            }

            /* -------- MENÚ DE TEST -------- */
            else if (current_ui_mode == UI_MODE_TEST_MENU) {
                if (net_movement != 0) {
                    int16_t new_pos = test_selected_item + net_movement;
                    test_selected_item = ((new_pos % test_menu_size) + test_menu_size) % test_menu_size;

                    if (test_selected_item >= test_top_item + LCD_ROWS)
                        test_top_item = test_selected_item - (LCD_ROWS - 1);
                    else if (test_selected_item < test_top_item)
                        test_top_item = test_selected_item;

                    needs_update = 1;
                }
                if (button_pressed) {
                    toggle_output(test_selected_item);
                    needs_update = 1;
                }
            }

            /* -------- SELECCIÓN DE CONFIGURACIÓN -------- */
            else if (current_ui_mode == UI_MODE_CONFIG_SELECT) {
                if (net_movement != 0) {
                    int16_t new_pos = config_sel_index + net_movement;
                    config_sel_index = ((new_pos % config_menu_sz) + config_menu_sz) % config_menu_sz;

                    if (config_sel_index >= config_top_index + LCD_ROWS)
                        config_top_index = config_sel_index - (LCD_ROWS - 1);
                    else if (config_sel_index < config_top_index)
                        config_top_index = config_sel_index;

                    needs_update = 1;
                }

                if (button_pressed) {
                    if (config_sel_index <= 1) { // Desarrollo o Eclosión
                        sysData.current_stage_idx = config_sel_index;
                        current_ui_mode = UI_MODE_CONFIG_EDIT;
                        config_item = 0;
                        is_editing_val = 0;
                    }
                    else if (config_sel_index == 2) { // Ciclo Total
                        current_ui_mode = UI_MODE_CONFIG_GLOBAL;
                        config_item = 0;
                        is_editing_val = 0;
                    }
                    else if (config_sel_index == 3) { // Ajustar Tiempo
                        current_ui_mode = UI_MODE_CONFIG_TIME;
                        config_item = 0;
                        is_editing_val = 0;
                        // Cargar valores actuales en variables temporales
                        edit_day  = Get_Current_Day();
                        if (edit_day == 0) edit_day = 1;
                        edit_hour = Get_Current_Hour();
                        edit_min  = Get_Current_Minute();
                    }
                    else { // Volver
                        current_ui_mode = UI_MODE_MAIN_MENU;
                    }
                    needs_update = 1;
                }
            }

            /* -------- CONFIGURACIÓN: EDICIÓN DE ETAPA (LO QUE FALTABA) -------- */
            else if (current_ui_mode == UI_MODE_CONFIG_EDIT) {
                StageConfig_t *st = &sysData.stages[sysData.current_stage_idx];

                if (button_pressed) {
                    if (config_item == 4) { // [SALIR]
                        Save_Config_To_Flash();
                        current_ui_mode = UI_MODE_CONFIG_SELECT;
                    } else {
                        is_editing_val = !is_editing_val;
                    }
                    needs_update = 1;
                }

                if (net_movement != 0) {
                    if (!is_editing_val) {
                        config_item += net_movement;
                        if (config_item < 0) config_item = 4;
                        if (config_item > 4) config_item = 0;
                    } else {
                        switch(config_item) {
                            case 0: // Fin Día
                                st->end_day += net_movement;
                                if(st->end_day < 1) st->end_day = 1;
                                break;
                            case 1: // Temp
                                st->temp_target += (net_movement * 0.1f);
                                if(st->temp_target > 42.0f) st->temp_target = 42.0f; // Límites
                                if(st->temp_target < 20.0f) st->temp_target = 20.0f;
                                break;
                            case 2: // Hum
                                st->hum_target += (float)net_movement;
                                if(st->hum_target > 90.0f) st->hum_target = 90.0f;
                                if(st->hum_target < 20.0f) st->hum_target = 20.0f;
                                break;
                            case 3: // Motor
                                if(net_movement != 0) st->motor_on = !st->motor_on;
                                break;
                        }
                    }
                    needs_update = 1;
                }
            }

            /* -------- CONFIGURACIÓN: GLOBAL (LO QUE FALTABA) -------- */
            else if (current_ui_mode == UI_MODE_CONFIG_GLOBAL) {
                if (button_pressed) {
                    if (config_item == 1) { // [GUARDAR]
                        Save_Config_To_Flash();
                        current_ui_mode = UI_MODE_CONFIG_SELECT;
                    } else {
                        is_editing_val = !is_editing_val;
                    }
                    needs_update = 1;
                }

                if (net_movement != 0) {
                    if (!is_editing_val) {
                        config_item += net_movement;
                        if (config_item < 0) config_item = 1;
                        if (config_item > 1) config_item = 0;
                    } else {
                        if (config_item == 0) { // Total Días
                            sysData.total_days += net_movement;
                            if (sysData.total_days < 1) sysData.total_days = 1;
                        }
                    }
                    needs_update = 1;
                }
            }

            /* -------- CONFIGURACIÓN: TIEMPO -------- */
            else if (current_ui_mode == UI_MODE_CONFIG_TIME) {
                if (button_pressed) {
                    if (config_item == 3) { // [GUARDAR]
                        uint32_t new_total_mins =
                            ((uint32_t)(edit_day - 1) * 1440) +
                            ((uint32_t)edit_hour * 60) +
                            (uint32_t)edit_min;

                        sysData.saved_timestamp = new_total_mins;
                        sysData.last_boot_tick = HAL_GetTick();
                        Save_Config_To_Flash();
                        current_ui_mode = UI_MODE_CONFIG_SELECT;
                    } else {
                        is_editing_val = !is_editing_val;
                    }
                    needs_update = 1;
                }

                if (net_movement != 0) {
                    if (!is_editing_val) {
                        config_item += net_movement;
                        if (config_item < 0) config_item = 3;
                        if (config_item > 3) config_item = 0;
                    } else {
                        switch (config_item) {
                            case 0: // Día
                                edit_day += net_movement;
                                if (edit_day < 1) edit_day = 1;
                                break;
                            case 1: // Hora
                                edit_hour = (edit_hour + net_movement + 24) % 24;
                                break;
                            case 2: // Min
                                edit_min = (edit_min + net_movement + 60) % 60;
                                break;
                        }
                    }
                    needs_update = 1;
                }
            }
        }
        /* ================= TIMEOUT (UPDATE-ON-CHANGE) ================= */
        else
        {
            if (current_ui_mode == UI_MODE_DASHBOARD) {
                // Chequear cambios usando liveStatus
                uint8_t day  = Get_Current_Day();
                uint8_t hour = Get_Current_Hour();

                if (liveStatus.temp_current != last_disp_temp ||
                    liveStatus.hum_current  != last_disp_hum  ||
                    liveStatus.rpm_current  != last_disp_rpm  ||
                    hum_state               != last_disp_hum_state ||
                    day                     != last_disp_day ||
                    hour                    != last_disp_hour ||
                    sysData.is_running      != last_disp_is_running)
                {
                    needs_update = 1;
                }
            }
        }

        /* ================= DIBUJO CONTROLADO ================= */
        uint32_t current_time = HAL_GetTick();

        if (needs_update && (current_time - last_draw_time >= MIN_DRAW_INTERVAL_MS))
        {
            if (current_ui_mode == UI_MODE_DASHBOARD) {
                last_disp_temp = liveStatus.temp_current;
                last_disp_hum  = liveStatus.hum_current;
                last_disp_rpm  = liveStatus.rpm_current;
                last_disp_hum_state = hum_state;
                last_disp_day  = Get_Current_Day();
                last_disp_hour = Get_Current_Hour();
                last_disp_is_running = sysData.is_running;
            }

            update_display();
            last_draw_time = current_time;
            needs_update = 0;
        }
    }
}



/**
 * @brief Tarea de debounce y detección de pulsaciones del botón del encoder.
 *
 * Esta tarea implementa un debounce por integrador digital y detecta:
 *  - Pulsación corta (BUTTON_PRESS)
 *  - Pulsación larga (BUTTON_LONG_PRESS)
 *
 * Funciona mediante sondeo periódico (polling) y envía eventos a la
 * cola del menú para ser procesados por la tarea de UI.
 *
 * Ventajas del enfoque:
 *  - No bloquea interrupciones
 *  - Filtrado robusto de rebotes mecánicos
 *  - Detección confiable de pulsación larga
 *
 * @param argument Parámetro no utilizado
 */
void StartDebounceTask(void *argument)
{
    /* Integrador digital para debounce:
       incrementa cuando el botón está presionado,
       decrementa cuando está liberado */
    uint8_t integrator = 0;

    /* Estado anterior del botón:
       1 = liberado
       0 = presionado */
    uint8_t prev_state = 1;

    /* Contador de tiempo de pulsación (para long press) */
    uint32_t hold_counter = 0;

    /* Flag para evitar reenviar múltiples eventos de pulsación larga */
    uint8_t long_press_sent = 0;

    const TickType_t poll_delay = pdMS_TO_TICKS(DEBOUNCE_POLL_RATE_MS);

    for (;;)
    {
        /* Periodo de muestreo del botón */
        vTaskDelay(poll_delay);

        /* Lectura del pin del botón (activo en nivel bajo) */
        uint8_t pin_state = HAL_GPIO_ReadPin(
                                ENCODER_SW_GPIO_Port,
                                ENCODER_SW_Pin);

        /* ================= DEBOUNCE POR INTEGRADOR ================= */

        if (pin_state == GPIO_PIN_RESET) {
            /* Botón presionado → integrar hacia arriba */
            if (integrator < DEBOUNCE_THRESHOLD)
                integrator++;
        } else {
            /* Botón liberado → integrar hacia abajo */
            if (integrator > 0)
                integrator--;
        }

        /* ================= DETECCIÓN DE EVENTOS ================= */

        /* ---- Transición: LIBERADO → PRESIONADO ---- */
        if (integrator >= DEBOUNCE_THRESHOLD && prev_state == 1) {

            prev_state = 0;        // Nuevo estado: presionado
            hold_counter = 0;      // Reiniciar contador de hold
            long_press_sent = 0;   // Habilitar detección de long press

            /* Enviar evento de pulsación corta */
            MenuEvent_t event = BUTTON_PRESS;
            xQueueSend(menuQueueHandle, &event, pdMS_TO_TICKS(10));
        }

        /* ---- Botón mantenido presionado ---- */
        else if (prev_state == 0 && integrator >= DEBOUNCE_THRESHOLD) {

            hold_counter++;

            /* Detectar pulsación larga */
            if (hold_counter >= LONG_PRESS_TICKS &&
                long_press_sent == 0)
            {
                long_press_sent = 1;

                MenuEvent_t event = BUTTON_LONG_PRESS;
                xQueueSend(menuQueueHandle, &event, pdMS_TO_TICKS(10));
            }
        }

        /* ---- Transición: PRESIONADO → LIBERADO ---- */
        else if (integrator == 0 && prev_state == 0) {

            prev_state = 1;        // Volver a estado liberado
        }
    }
}


/**
 * @brief Tarea de cálculo de RPM del motor.
 *
 * Esta tarea se ejecuta periódicamente y calcula la velocidad del motor
 * en RPM a partir de los pulsos generados por un sensor o encoder.
 *
 * Funcionamiento:
 *  - La interrupción incrementa `motor_pulse_count`
 *  - Esta tarea toma un snapshot del contador de forma atómica
 *  - Calcula las RPM según el intervalo de muestreo
 *  - Actualiza variables globales y de estado en tiempo real
 *
 * El uso de una sección crítica garantiza coherencia de datos entre
 * interrupciones y contexto de tarea.
 *
 * @param argument Parámetro no utilizado
 */
void StartMotorTask(void *argument)
{
    /* Variable local para capturar los pulsos acumulados
       durante el intervalo de medición */
    uint32_t pulses_snapshot = 0;

    const TickType_t calc_delay =
        pdMS_TO_TICKS(MOTOR_CALC_INTERVAL_MS);

    for (;;)
    {
        /* Espera fija entre cálculos de RPM */
        vTaskDelay(calc_delay);

        /* ================= SECCIÓN CR�?TICA =================
           Protege el acceso a motor_pulse_count, que es
           modificada desde una ISR (EXTI / sensor de motor) */
        taskENTER_CRITICAL();
        pulses_snapshot = motor_pulse_count;  // Copia atómica
        motor_pulse_count = 0;                 // Reinicio del contador
        taskEXIT_CRITICAL();
        /* =================================================== */

        /* Cálculo de RPM:
           pulses_snapshot  → pulsos en el intervalo
           ENCODER_SLOTS    → pulsos por revolución
           MOTOR_CALC_INTERVAL_MS → ventana de tiempo en ms

           Fórmula:
           RPM = (pulsos * 60s * 1000ms) / (slots * intervalo_ms)
        */
        uint32_t calculated_rpm =
            (pulses_snapshot * 60UL * 1000UL) /
            (ENCODER_SLOTS * MOTOR_CALC_INTERVAL_MS);

        /* Guardar RPM calculada (limitada a 16 bits) */
        global_rpm = (uint16_t)calculated_rpm;

        /* Actualizar estructura de estado en tiempo real
           utilizada por el dashboard / UI */
        liveStatus.rpm_current = global_rpm;
    }
}


/**
 * @brief Tarea principal de control ambiental de la incubadora.
 *
 * Esta tarea implementa el lazo de control de:
 *  - Temperatura (lámpara calefactora + cooler)
 *  - Humedad (humidificador ultrasónico)
 *  - Actualización de variables de estado en tiempo real
 *
 * El control se basa en:
 *  - Histéresis y soft-PWM para temperatura
 *  - Máquina de estados temporizada para humedad
 *
 * Todas las salidas son Active-Low:
 *  - GPIO_PIN_RESET → dispositivo encendido
 *  - GPIO_PIN_SET   → dispositivo apagado
 *
 * @param argument Parámetro no utilizado (requerido por CMSIS-RTOS)
 */
void StartControlTask(void * argument)
{
	/**
	 * @brief Tarea de Control Avanzado: 3 Estados Térmicos + Humedad Pulsante.
	 */

	    // Variables de Objetivos
	    float target_t = 0.0f;
	    float target_h = 0.0f;
	    uint8_t motor_enabled = 0;

	    // --- TIMERS Y FLAGS DE ALARMA TEMPERATURA ---
	    uint32_t timer_tolerable_start = 0;
	    uint8_t  flag_tolerable_active = 0;

	    uint32_t timer_critical_start = 0;
	    uint8_t  flag_critical_active = 0;

	    // --- TIMERS HUMEDAD ---
	    uint32_t hum_pulse_timer = 0;   // Para el ciclo ON/OFF
	    uint32_t hum_alarm_timer = 0;   // Para la alarma de tiempo
	    uint8_t  flag_hum_alarm_active = 0;

	    // Sincronización inicial para que el DHT11 tenga datos
	    vTaskDelay(pdMS_TO_TICKS(2000));

	    for(;;)
	    {
	        uint32_t now = HAL_GetTick();

	        // 1. Obtener Objetivos (EEPROM/Flash)
	        Get_Active_Targets(&target_t, &target_h, &motor_enabled);

	        // Actualizar datos para UI
	        liveStatus.day_current  = Get_Current_Day();
	        liveStatus.hour_current = Get_Current_Hour();
	        liveStatus.temp_target  = target_t;
	        liveStatus.hum_target   = target_h;

	        // Si el sistema está pausado (Target=0), apagar todo
	        if (target_t == 0.0f) {
	            RELES_ApagarTodos();
	            flag_tolerable_active = 0; flag_critical_active = 0;
	            vTaskDelay(pdMS_TO_TICKS(1000));
	            continue;
	        }



	        float curr_t = liveStatus.temp_current;
	        float curr_h = liveStatus.hum_current;
	        float delta_t = curr_t - target_t; // Diferencia (positiva es calor)
	        float DELTA_CRITICA = (target_t*.08f); //8% DE DESVIO DEL TARGET
	        float DELTA_TOLERABLE = (target_t*.03f);//3% DE DESVIO
	        float HUM_TOLERANCIA = (target_h*.05f);//5% DE DESVIO
	        uint32_t TIEMPO_LIMITE_CRITICO = pdMS_TO_TICKS(900000); //15 MIN en ms
	        uint32_t TIEMPO_LIMITE_TOLERABLE = pdMS_TO_TICKS(3600000); //60 MIN en ms
	        uint32_t TIEMPO_ALARMA_HUMEDAD = pdMS_TO_TICKS(1800000); //30 MIN en ms
	        uint32_t HUM_TIEMPO_ON = pdMS_TO_TICKS(5000);	//5seg
			uint32_t HUM_TIEMPO_OFF = pdMS_TO_TICKS(15000);	//espera 15seg

	        // ============================================================
	        //              LOGICA DE TEMPERATURA (3 ESTADOS)
	        // ============================================================

	        uint8_t activar_cooler = 0;
	        uint8_t activar_buzzer = 0;
	        uint8_t enviar_telegram = 0;

	        // --- 1. ESTADO CRÍTICO (Prioridad Máxima) ---
	        if (delta_t >= DELTA_CRITICA) {
	            // Si acabamos de entrar a crítico, iniciamos cronómetro
	            if (!flag_critical_active) {
	                timer_critical_start = now;
	                flag_critical_active = 1;
	            }

	            // Chequeo de tiempo (15 min)		//Tengo un pseudotimer que se maneja por ticks
	            if ((now - timer_critical_start) >= TIEMPO_LIMITE_CRITICO) {
	                activar_buzzer = 1;
	                activar_cooler = 1; // Forzar enfriamiento
	                enviar_telegram = 1; // "ALERTA CRITICA: HUEVO EN PELIGRO"
	            }
	        }
	        else {
	            flag_critical_active = 0; // Salimos de zona crítica
	        }

	        // --- 2. ESTADO TOLERABLE (Prioridad Media) ---
	        // Estamos calientes, pero no críticos aún
	        if (delta_t >= DELTA_TOLERABLE && delta_t < DELTA_CRITICA) {
	            if (!flag_tolerable_active) {
	                timer_tolerable_start = now;
	                flag_tolerable_active = 1;
	            }

	            // Chequeo de tiempo (1 Hora)		//IDEM QUI
	            if ((now - timer_tolerable_start) >= TIEMPO_LIMITE_TOLERABLE) {
	                activar_buzzer = 1;
	                activar_cooler = 1; // Ayudar a bajar
	                enviar_telegram = 1; // "ALERTA: Tiempo prolongado en calor"
	            }
	        }
	        else {
	            flag_tolerable_active = 0; // Salimos de zona tolerable
	        }

	        // --- 3. ESTADO NORMAL (Control Histeresis) ---
	        // Solo actuamos sobre la Lámpara aquí. El cooler y buzzer dependen de las alarmas arriba.

	        if (curr_t < (target_t - DELTA_TOLERABLE)) {
	            encender_lampara(); // Hace frío -> Calentar
	        }
	        else if (curr_t >= target_t + DELTA_TOLERABLE) {
	            apagar_lampara();   // Llegamos al objetivo -> Inercia térmica hará el resto
	        }

	        // --- EJECUCIÓN DE ACTUADORES DE ALARMA ---
	        if (activar_buzzer) encender_buzzer();
	        else apagar_buzzer();

	        if (activar_cooler) encender_cooler();
	        else apagar_cooler();

	        /* Placeholder para Telegram (Implementar envio UART aqui) */
	        if (enviar_telegram) {
	             // Send_Telegram_Message(ALERTA_TEMP);
	             // Tip: Usar un timer o flag para no enviar 1 mensaje por segundo
	        }


	        // ============================================================
	        //              LOGICA DE HUMEDAD (PULSANTE + ALARMA)
	        // ============================================================

	        // Alarma por tiempo fuera de rango
	        if (curr_h < (target_h - HUM_TOLERANCIA) || curr_h > (target_h + HUM_TOLERANCIA)) {
	             if (!flag_hum_alarm_active) {
	                 hum_alarm_timer = now;
	                 flag_hum_alarm_active = 1;
	             }
	             if ((now - hum_alarm_timer) >= TIEMPO_ALARMA_HUMEDAD) {
	                 // Alarma simple de humedad (quizás solo mensaje telegram, sin buzzer molesto)
	                 // Send_Telegram_Message(ALERTA_HUM);
	             }
	        } else {
	             flag_hum_alarm_active = 0;
	        }

	        // Maquina de estados PULSANTE (Dosificación)
	        switch (hum_state) {
	            case HUM_STATE_IDLE:
	                // Si falta humedad
	                if (curr_h < (target_h - HUM_TOLERANCIA)) {
	                    encender_humidificador(); // ON
	                    hum_pulse_timer = now;
	                    hum_state = HUM_STATE_DOSING;
	                } else {
	                    apagar_humidificador();
	                }
	                break;

	            case HUM_STATE_DOSING:
	                // Mantener ON solo por un pulso corto
	                if ((now - hum_pulse_timer) >= HUM_TIEMPO_ON) {
	                    apagar_humidificador(); // OFF
	                    hum_pulse_timer = now;
	                    hum_state = HUM_STATE_COOLDOWN;
	                }
	                break;

	            case HUM_STATE_COOLDOWN:
	                // Esperar TIEMPO LARGO para dispersión antes de volver a inyectar
	                if ((now - hum_pulse_timer) >= HUM_TIEMPO_OFF) {
	                    hum_state = HUM_STATE_IDLE; // Listo para medir y decidir de nuevo
	                }
	                break;
	        }

	        // Control Motor (Si aplica)
	        if (motor_enabled == 0) apagar_motor();

	        vTaskDelay(pdMS_TO_TICKS(1000)); // Ciclo de control de 1 segundo
	    }
}

void StartSensorTask(void *argument){
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        /* Pauso scheduler para mantener timing fino del DHT11 */
		DHT11_GetDatos();

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM12_Init();


  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  Enable_Internal_Pullups(); // Anti-ruido I2C
  I2C_Scan();                // Escaneo
  HD44780_Init(LCD_ROWS);
  // Cargar configuración guardada
  Load_Config_From_Flash();

  DHT11_Init();
  RELES_Init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  xDHTMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  menuQueueHandle = xQueueCreate(32, sizeof(uint32_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  xTaskCreate(StartMenuTask, "Menu", 1024, NULL, osPriorityRealtime, &menuTaskHandle);
  xTaskCreate(StartDebounceTask, "Debounce", 128, NULL, osPriorityNormal, &debounceTaskHandle);
  //xTaskCreate(StartMotorTask, "Motor", 128, NULL, osPriorityNormal, &motorTaskHandle);
  //xTaskCreate(StartControlTask, "Control", 512, NULL, osPriorityNormal, &controlTaskHandle);
  xTaskCreate(StartSensorTask, "Sensor", 1024, NULL, osPriorityAboveNormal, &sensorTaskHandle);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 83;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|HUMIDIFICADOR_Pin|COOLER_Pin|DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LAMPARA_Pin|BUZZER_Pin|MOTOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_A_Pin */
  GPIO_InitStruct.Pin = ENCODER_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_B_Pin */
  GPIO_InitStruct.Pin = ENCODER_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin HUMIDIFICADOR_Pin COOLER_Pin DHT11_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|HUMIDIFICADOR_Pin|COOLER_Pin|DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_SW_Pin */
  GPIO_InitStruct.Pin = ENCODER_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENCODER_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LAMPARA_Pin BUZZER_Pin MOTOR_Pin */
  GPIO_InitStruct.Pin = LAMPARA_Pin|BUZZER_Pin|MOTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
   while(1)
   {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(50);
   }
}

/**
 * @brief Callback de interrupción externa por GPIO (EXTI).
 *
 * Esta función es llamada automáticamente por la HAL cuando ocurre
 * una interrupción externa en un pin configurado como EXTI.
 *
 * Maneja dos funcionalidades:
 *  - Conteo de pulsos del sensor de motor (RPM)
 *  - Decodificación de un encoder rotativo en cuadratura (A/B)
 *
 * @param GPIO_Pin Pin que generó la interrupción.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* ================= SENSOR DE MOTOR ================= */

    // Si la interrupción proviene del sensor de RPM del motor
    if (GPIO_Pin == MOTOR_SENSOR_PIN) {
        // Incrementar contador de pulsos (usado luego para cálculo de RPM)
        motor_pulse_count++;
        return; // Salir rápido para minimizar tiempo en ISR
    }

    /* ================= ENCODER ROTATIVO ================= */

    // Si la interrupción proviene de cualquiera de las fases del encoder
    if (GPIO_Pin == ENCODER_A_Pin || GPIO_Pin == ENCODER_B_Pin) {

        // Último estado válido del encoder (AB)
        static uint8_t last_state = 0;

        // Contador interno para acumular transiciones
        // 4 transiciones = 1 paso mecánico del encoder
        static int8_t counter = 0;

        // Leer estados actuales de las señales A y B
        uint8_t state_A = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin);
        uint8_t state_B = HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);

        // Combinar A y B en un estado de 2 bits: [A B]
        uint8_t current_state = (state_A << 1) | state_B;

        // Ignorar si el estado no cambió (rebotes o ruido)
        if (current_state == last_state) return;

        /*
         * Tabla de transición del encoder en cuadratura.
         * Index = (last_state << 2) | current_state
         * Valores:
         *  +1 -> giro horario
         *  -1 -> giro antihorario
         *   0 -> transición inválida o ruido
         */
        const int8_t transition_table[16] = {
             0, -1,  1,  0,
             1,  0,  0, -1,
            -1,  0,  0,  1,
             0,  1, -1,  0,
        };

        // Obtener la transición correspondiente
        int8_t transition =
            transition_table[(last_state << 2) | current_state];

        // Si la transición es válida
        if (transition != 0) {

            // Acumular transición
            counter += transition;

            // 4 transiciones positivas → un paso a la derecha
            if (counter >= 4) {
                MenuEvent_t event = ENCODER_RIGHT;

                // Enviar evento a la cola del menú
                osMessagePut(menuQueueHandle, (uint32_t)event, 0);

                // Ajustar contador
                counter -= 4;
            }
            // 4 transiciones negativas → un paso a la izquierda
            else if (counter <= -4) {
                MenuEvent_t event = ENCODER_LEFT;

                // Enviar evento a la cola del menú
                osMessagePut(menuQueueHandle, (uint32_t)event, 0);

                // Ajustar contador
                counter += 4;
            }
        }

        // Actualizar el último estado del encoder
        last_state = current_state;
    }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
