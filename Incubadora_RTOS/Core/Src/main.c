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
    if (is_welcome) {
    	LCD_ShowWelcome("TDII: INCUBADORA");

        /* Mantener visible */
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
           Timeout de 200 ms para permitir refresco periódico */
        if (xQueueReceive(menuQueueHandle, &event,
                          pdMS_TO_TICKS(200)) == pdPASS)
        {
            MenuEvent_t initial_event = event;

            /* -------- BOTÓN: PRESIÓN LARGA (volver atrás) -------- */
            if (initial_event == BUTTON_LONG_PRESS) {

                if (current_ui_mode == UI_MODE_TEST_MENU ||
                    current_ui_mode == UI_MODE_CONFIG_EDIT ||
                    current_ui_mode == UI_MODE_CONFIG_GLOBAL ||
                    current_ui_mode == UI_MODE_CONFIG_TIME)
                {
                    current_ui_mode = UI_MODE_MAIN_MENU;
                }
                else if (current_ui_mode == UI_MODE_CONFIG_SELECT) {
                    current_ui_mode = UI_MODE_MAIN_MENU;
                }
                else if (current_ui_mode == UI_MODE_MAIN_MENU) {
                    current_ui_mode = UI_MODE_DASHBOARD;
                }

                needs_update = 1;
                continue;
            }

            /* -------- PROCESAMIENTO DE MOVIMIENTO -------- */
            int8_t net_movement = 0;
            int button_pressed = 0;

            if (initial_event == ENCODER_RIGHT) net_movement++;
            else if (initial_event == ENCODER_LEFT) net_movement--;
            else if (initial_event == BUTTON_PRESS) button_pressed = 1;

            /* Acumula eventos rápidos del encoder (anti-lag UI) */
            int batch_limit = 20;
            while (batch_limit-- > 0 &&
                   xQueueReceive(menuQueueHandle, &event, 0) == pdPASS)
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
                    selected_item =
                        ((new_pos % menu_size) + menu_size) % menu_size;

                    if (selected_item >= menu_top_item + LCD_ROWS)
                        menu_top_item = selected_item - (LCD_ROWS - 1);
                    else if (selected_item < menu_top_item)
                        menu_top_item = selected_item;

                    needs_update = 1;
                }

                if (button_pressed) {

                    if (selected_item == 1) { // INICIAR / PAUSAR
                        if (sysData.is_running) {
                            Save_Config_To_Flash();
                            sysData.is_running = 0;
                            Save_Config_To_Flash();
                        } else {
                            sysData.is_running = 1;
                            sysData.last_boot_tick = HAL_GetTick();
                            Save_Config_To_Flash();
                        }
                    }
                    else if (selected_item == 2) {
                        current_ui_mode = UI_MODE_CONFIG_SELECT;
                        config_sel_index = 0;
                        config_top_index = 0;
                    }
                    else if (selected_item == 3) {
                        current_ui_mode = UI_MODE_TEST_MENU;
                        test_selected_item = 0;
                    }
                    else {
                        current_ui_mode = UI_MODE_DASHBOARD;
                    }

                    needs_update = 1;
                }
            }

            /* -------- MENÚ DE TEST -------- */
            else if (current_ui_mode == UI_MODE_TEST_MENU) {

                if (net_movement != 0) {
                    int16_t new_pos = test_selected_item + net_movement;
                    test_selected_item =
                        ((new_pos % test_menu_size) + test_menu_size) % test_menu_size;

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
                    config_sel_index =
                        ((new_pos % config_menu_sz) + config_menu_sz) % config_menu_sz;

                    if (config_sel_index >= config_top_index + LCD_ROWS)
                        config_top_index = config_sel_index - (LCD_ROWS - 1);
                    else if (config_sel_index < config_top_index)
                        config_top_index = config_sel_index;

                    needs_update = 1;
                }

                if (button_pressed) {
                    if (config_sel_index <= 1) {
                        sysData.current_stage_idx = config_sel_index;
                        current_ui_mode = UI_MODE_CONFIG_EDIT;
                        config_item = 0;
                        is_editing_val = 0;
                    }
                    else if (config_sel_index == 2) {
                        current_ui_mode = UI_MODE_CONFIG_GLOBAL;
                        config_item = 0;
                        is_editing_val = 0;
                    }
                    else if (config_sel_index == 3) {
                        current_ui_mode = UI_MODE_CONFIG_TIME;
                        config_item = 0;
                        is_editing_val = 0;

                        edit_day  = Get_Current_Day();
                        if (edit_day == 0) edit_day = 1;
                        edit_hour = Get_Current_Hour();
                        edit_min  = Get_Current_Minute();
                    }
                    else {
                        current_ui_mode = UI_MODE_MAIN_MENU;
                    }

                    needs_update = 1;
                }
            }

            /* -------- CONFIGURACIÓN DE TIEMPO -------- */
            else if (current_ui_mode == UI_MODE_CONFIG_TIME) {

                if (button_pressed) {
                    if (config_item == 3) {
                        uint32_t new_total_mins =
                            ((uint32_t)(edit_day - 1) * 1440) +
                            ((uint32_t)edit_hour * 60) +
                            (uint32_t)edit_min;

                        sysData.saved_timestamp = new_total_mins;
                        sysData.last_boot_tick = HAL_GetTick();
                        Save_Config_To_Flash();
                        current_ui_mode = UI_MODE_CONFIG_SELECT;
                    }
                    else {
                        is_editing_val = !is_editing_val;
                    }
                    needs_update = 1;
                }

                if (net_movement != 0) {
                    if (!is_editing_val) {
                        config_item += net_movement;
                        if (config_item < 0) config_item = 3;
                        if (config_item > 3) config_item = 0;
                    }
                    else {
                        if (config_item == 0) {
                            edit_day += net_movement;
                            if (edit_day < 1) edit_day = 1;
                        }
                        else if (config_item == 1) {
                            edit_hour = (edit_hour + net_movement + 24) % 24;
                        }
                        else if (config_item == 2) {
                            edit_min = (edit_min + net_movement + 60) % 60;
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

        if (needs_update &&
            (current_time - last_draw_time >= MIN_DRAW_INTERVAL_MS))
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
    /* ============================================================
       INICIALIZACIÓN SEGURA
       Asegura que todos los actuadores estén apagados al inicio
       (Active Low → HIGH = OFF)
       ============================================================ */
	apagar_cooler();
	apagar_humidificador();
	apagar_lampara();
	apagar_motor();
	apagar_buzzer();

    /* Variables de control */
    float target_t = 0;          // Temperatura objetivo
    float target_h = 0;          // Humedad objetivo
    uint8_t motor_enabled = 0;   // Estado del motor (etapa)
    uint32_t hum_timer_start = 0;// Temporizador máquina de humedad

    for (;;)
    {
        uint32_t now = HAL_GetTick();

        /* ============================================================
           1. OBTENER OBJETIVOS ACTIVOS SEGÚN EL D�?A DE INCUBACIÓN
           ============================================================ */
        Get_Active_Targets(&target_t, &target_h, &motor_enabled);

        /* Actualización de información en tiempo real (UI / Debug) */
        liveStatus.day_current  = Get_Current_Day();
        liveStatus.hour_current = Get_Current_Hour();
        liveStatus.temp_target  = target_t;
        liveStatus.hum_target   = target_h;

        /* Si el sistema está pausado o fuera de ciclo,
           no se ejecuta control activo */
        if (target_t == 0) {
        	vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* ============================================================
           2. LECTURA DE SENSOR (SIMULADA / REAL)
           Se ejecuta cada DHT_READ_INTERVAL_MS
           ============================================================ */
        if ((now - last_dht_read_time) >= DHT_READ_INTERVAL_MS) {

            /* SIMULACIÓN DE TEMPERATURA
               (Reemplazar por DHT11_Read / DHT22_Read) */

            // Si la lámpara está encendida → sube la temperatura
            if (estado_lampara() == RELE_ACTIVO)
                last_valid_temp += 0.2f;
            else
            	last_valid_temp -= 0.1f;

            /* Límite inferior de seguridad */
            if (last_valid_temp < 20.0f)
                last_valid_temp = 20.0f;

            /* Actualizar valores globales de estado */
//liveStatus.temp_current = last_valid_temp;
            //liveStatus.hum_current  = last_valid_hum;

            last_dht_read_time = now;
        }

        /* ============================================================
           3. CONTROL DE TEMPERATURA
           - Histéresis
           - Soft PWM en zona muerta
           ============================================================ */
        float error_temp = target_t - last_valid_temp;

        if (error_temp > 0.5f) {
            /* Muy frío → Lámpara ON, Cooler OFF */
            encender_lampara();
            apagar_cooler();
        }
        else if (error_temp > 0.0f && error_temp <= 0.5f) {
            /* Zona de regulación suave (Soft PWM 15s) */
            if ((now - last_dht_read_time) < 15000)
                encender_lampara(); // ON
            else
            	apagar_lampara();   // OFF
        }
        else {
            /* Temperatura alcanzada o excedida */
            apagar_lampara(); // Lámpara OFF

            /* Protección térmica con ventilación */
            if (last_valid_temp > (target_t + 1.0f))
                encender_cooler(); // Cooler ON
            else
                apagar_cooler();   // Cooler OFF
        }

        /* ============================================================
           4. CONTROL DE HUMEDAD
           Máquina de estados temporizada
           ============================================================ */
        switch (hum_state) {

            case HUM_STATE_IDLE:
                /* Humedad baja → iniciar dosificación */
                if (last_valid_hum < (target_h - 5.0f)) {
                    encender_humidificador(); // ON
                    hum_timer_start = now;
                    hum_state = HUM_STATE_DOSING;
                }
                break;

            case HUM_STATE_DOSING:
                /* Tiempo máximo de inyección de humedad */
                if ((now - hum_timer_start) >= HUM_DOSE_TIME_MS) {
                    apagar_humidificador(); // OFF
                    hum_timer_start = now;
                    hum_state = HUM_STATE_COOLDOWN;
                }
                break;

            case HUM_STATE_COOLDOWN:
                /* Tiempo de estabilización antes de volver a medir */
                if ((now - hum_timer_start) >= HUM_COOLDOWN_TIME_MS)
                    hum_state = HUM_STATE_IDLE;
                break;
        }

        /* ============================================================
           5. RETARDO DEL LAZO DE CONTROL
           ============================================================ */
        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
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
  xTaskCreate(StartControlTask, "Control", 512, NULL, osPriorityNormal, &controlTaskHandle);
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
