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
#include "i2c-lcd.h"
#include <stdio.h>
#include <string.h> // Necesario para memcpy
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    ENCODER_RIGHT,
    ENCODER_LEFT,
    BUTTON_PRESS,
    BUTTON_LONG_PRESS
} MenuEvent_t;

typedef struct {
    char* name;          // Nombre a mostrar
    uint8_t state;       // 0 = OFF, 1 = ON
    GPIO_TypeDef* port;  // Puerto GPIO
    uint16_t pin;        // Pin GPIO
} OutputControl_t;

typedef enum {
    UI_MODE_DASHBOARD,
    UI_MODE_MAIN_MENU,
    UI_MODE_TEST_MENU,
    UI_MODE_CONFIG_EDIT
} UIMode_t;

typedef enum {
    HUM_STATE_IDLE,      // Esperando
    HUM_STATE_DOSING,    // Inyectando humedad
    HUM_STATE_COOLDOWN   // Esperando estabilización
} HumidifierState_t;

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



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBOUNCE_POLL_RATE_MS   5
#define DEBOUNCE_THRESHOLD      4
#define LCD_ROWS 2

// --- NUEVOS DEFINES (ESTABILIDAD & MOTOR) ---
#define LONG_PRESS_MS           2000
#define LONG_PRESS_TICKS        (LONG_PRESS_MS / DEBOUNCE_POLL_RATE_MS)
#define LCD_COMMAND_DELAY       5
#define MIN_DRAW_INTERVAL_MS    50

// --- CONFIG MOTOR (PA0) ---
#define MOTOR_SENSOR_PIN        GPIO_PIN_0
#define MOTOR_SENSOR_PORT       GPIOA
#define ENCODER_SLOTS           20
#define MOTOR_CALC_INTERVAL_MS  1000

// --- CONTROL DEFINES ---
#define DHT_READ_INTERVAL_MS    30000   // Leer sensor cada 30 seg
#define CONTROL_LOOP_MS         1000    // Recalcular lógica cada 1 seg
#define HUM_DOSE_TIME_MS        10000   // Humidificador ON 10 seg
#define HUM_COOLDOWN_TIME_MS    300000  // Espera 5 min

// EEPROM --------------

#define FLASH_USER_START_ADDR 0x08060000 // Sector 7 (F446RE)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId menuTaskHandle;
osThreadId debounceTaskHandle;
osThreadId MotorTaskHandle;
osMessageQId menuQueueHandle;
/* USER CODE BEGIN PV */
// --- TUS VARIABLES ORIGINALES (Menú Simple) ---
char* menu_items[] = {"Ver Sensores", "Configuracion", "TEST"};
int8_t selected_item = 0;
const int8_t menu_size = 3;
int8_t menu_top_item = 0;

// --- VARIABLES DE SISTEMA ---
UIMode_t current_ui_mode = UI_MODE_DASHBOARD;
volatile uint32_t motor_pulse_count = 0;
uint16_t global_rpm = 0;

// Variables Config Edit
int8_t config_item = 0;
int8_t is_editing_val = 0;

// --- VARIABLES DE CONTROL ---
float current_temp = 0.0f;
float current_hum = 0.0f;
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
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
void StartMenuTask(void const * argument);
void StartDebounceTask(void const * argument);
void StartMotorTask(void const * argument);

/* USER CODE BEGIN PFP */
void I2C_Scan(void);
void Enable_Internal_Pullups(void);
void recover_lcd(void);
void update_display(void);
void render_dashboard(void);
void render_menu(void);
void render_test_menu(void);
void render_config_edit(void);
void toggle_output(int index);

void Load_Config_From_Flash(void);
void Save_Config_To_Flash(void);

uint8_t Get_Current_Day(void);
void Get_Active_Targets(float *temp, float *hum, uint8_t *motor_on);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- FUNCIONES AUXILIARES ---

void Enable_Internal_Pullups(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void recover_lcd(void) {
    HD44780_Init(2);
    HD44780_Clear();
    osDelay(50);
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("! PANIC RESET ! ");
    osDelay(1000);
    HD44780_Clear();
    update_display();
}

void I2C_Scan(void) {
    char info[] = "Escaneando bus I2C...\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)info, sizeof(info) - 1, 100);
    HAL_StatusTypeDef res;
    for(uint16_t i = 0; i < 128; i++) {
        res = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 1, 10);
        if(res == HAL_OK) {
            char msg[64];
            snprintf(msg, sizeof(msg), "-> Disp: 0x%02X\r\n", i);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        }
    }
    char end_info[] = "Fin del escaneo.\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)end_info, sizeof(end_info) - 1, 100);
}

// --- GESTIÓN DE MEMORIA FLASH (PERSISTENCIA) ---

void Load_Config_From_Flash(void) {
    IncubatorData_t *flash_data = (IncubatorData_t *)FLASH_USER_START_ADDR;

    // Chequeamos si la memoria está virgen (0xFF) o corrupta
    if (flash_data->total_days == 0xFF || flash_data->stages[0].temp_target > 100.0f) {
        // --- VALORES DE F�?BRICA ---
        sysData.total_days = 21;

        // Etapa 0: Desarrollo
        snprintf(sysData.stages[0].name, 12, "DESARROLLO");
        sysData.stages[0].end_day = 18;
        sysData.stages[0].temp_target = 37.7f;
        sysData.stages[0].hum_target = 55.0f;
        sysData.stages[0].motor_on = 1;

        // Etapa 1: Eclosión
        snprintf(sysData.stages[1].name, 12, "ECLOSION");
        sysData.stages[1].end_day = 21;
        sysData.stages[1].temp_target = 37.2f;
        sysData.stages[1].hum_target = 70.0f;
        sysData.stages[1].motor_on = 0;

        sysData.is_running = 0;
        sysData.saved_timestamp = 0;
    } else {
        // Copiar datos guardados a RAM
        memcpy(&sysData, flash_data, sizeof(IncubatorData_t));
    }
    sysData.last_boot_tick = HAL_GetTick();
}

void Save_Config_To_Flash(void) {
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_7;
    EraseInitStruct.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    uint32_t address = FLASH_USER_START_ADDR;
    uint32_t *data_ptr = (uint32_t *)&sysData;

    // Actualizar tiempo acumulado antes de guardar
    if (sysData.is_running) {
        uint32_t current_session_min = (HAL_GetTick() - sysData.last_boot_tick) / 60000;
        sysData.saved_timestamp += current_session_min;
        sysData.last_boot_tick = HAL_GetTick();
    }

    for (int i = 0; i < sizeof(IncubatorData_t); i += 4) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *data_ptr);
        address += 4;
        data_ptr++;
    }
    HAL_FLASH_Lock();
}

// --- LÓGICA DE TIEMPO ---

uint8_t Get_Current_Day(void) {
    if (!sysData.is_running) return 0;
    uint32_t session_mins = (HAL_GetTick() - sysData.last_boot_tick) / 60000;
    uint32_t total_mins = sysData.saved_timestamp + session_mins;
    uint8_t day = (total_mins / 1440) + 1; // 1440 min = 1 dia
    return day;
}

void Get_Active_Targets(float *temp, float *hum, uint8_t *motor_on) {
    uint8_t day = Get_Current_Day();
    if (day == 0) {
        *temp = 0; *hum = 0; *motor_on = 0;
        return;
    }
    // Si estamos en etapa de desarrollo
    if (day <= sysData.stages[0].end_day) {
        *temp = sysData.stages[0].temp_target;
        *hum = sysData.stages[0].hum_target;
        *motor_on = sysData.stages[0].motor_on;
    }
    else {
        // Eclosión o fin
        *temp = sysData.stages[1].temp_target;
        *hum = sysData.stages[1].hum_target;
        *motor_on = sysData.stages[1].motor_on;
    }
}

// --- HARDWARE HELPERS ---
void toggle_output(int index) {
    if(index < 0 || index >= test_menu_size) return;
    test_outputs[index].state = !test_outputs[index].state;
    HAL_GPIO_WritePin(test_outputs[index].port, test_outputs[index].pin,
                      test_outputs[index].state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// --- RENDERIZADO DASHBOARD (RPM) ---
void render_dashboard(void) {
    char buffer[20];
    float target_t = 0; float target_h = 0; uint8_t mon = 0;

    if (sysData.is_running) {
        Get_Active_Targets(&target_t, &target_h, &mon);
        // T:37.7/37.7
        snprintf(buffer, sizeof(buffer), "T:%04.1f/%04.1f    ", current_temp, target_t);
    } else {
        snprintf(buffer, sizeof(buffer), "T:%04.1f (PAUSA) ", current_temp);
    }

    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(buffer);

    // Línea 2
    char hum_char = ' ';
    if (hum_state == HUM_STATE_DOSING) hum_char = '*';
    if (hum_state == HUM_STATE_COOLDOWN) hum_char = 'w';

    if (sysData.is_running) {
        snprintf(buffer, sizeof(buffer), "H:%02.0f%% %c D:%d  ", current_hum, hum_char, Get_Current_Day());
    } else {
        snprintf(buffer, sizeof(buffer), "H:%02.0f%% STANDBY ", current_hum);
    }

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(buffer);
}

// --- RENDERIZADO MENÚ (Tu lógica original segura) ---
void render_menu(void) {
    char line_buffer[32];
    for (int i = 0; i < LCD_ROWS; i++) {
        int item_index = menu_top_item + i;
        if (item_index < menu_size) {
            char cursor = (item_index == selected_item) ? '>' : ' ';
            // Se usa tu array de char* menu_items
            snprintf(line_buffer, sizeof(line_buffer), "%c%-15.15s", cursor, menu_items[item_index]);
        } else {
            snprintf(line_buffer, sizeof(line_buffer), "%-16s", " ");
        }
        HD44780_SetCursor(0, i);
        HD44780_PrintStr(line_buffer);
        // ELIMINADO: osDelay(LCD_COMMAND_DELAY); (Causa problemas en critical section)
    }
}

void render_test_menu(void) {
    char line_buffer[20];
    for (int i = 0; i < LCD_ROWS; i++) {
        int item_index = test_top_item + i;
        if (item_index < test_menu_size) {
            char cursor = (item_index == test_selected_item) ? '>' : ' ';
            char state_char = (test_outputs[item_index].state) ? 'X' : ' ';
            snprintf(line_buffer, sizeof(line_buffer), "%c[%c] %-9s",
                     cursor, state_char, test_outputs[item_index].name);
        } else {
            snprintf(line_buffer, sizeof(line_buffer), "%-16s", " ");
        }
        HD44780_SetCursor(0, i);
        HD44780_PrintStr(line_buffer);
    }
}

void render_config_edit(void) {
    char line1[20];
    char line2[20];
    StageConfig_t *stage = &sysData.stages[sysData.current_stage_idx];

    // L1: Título
    snprintf(line1, sizeof(line1), "CFG: %s", stage->name);

    // L2: Parámetro
    char value_str[12];
    switch(config_item) {
        case 0: snprintf(value_str, sizeof(value_str), "FinDia:%d", stage->end_day); break;
        case 1: snprintf(value_str, sizeof(value_str), "T:%.1f", stage->temp_target); break;
        case 2: snprintf(value_str, sizeof(value_str), "H:%.0f%%", stage->hum_target); break;
        case 3: snprintf(value_str, sizeof(value_str), "Mot:%s", stage->motor_on ? "ON" : "OFF"); break;
        case 4: snprintf(value_str, sizeof(value_str), "[SALIR]"); break;
    }

    if (is_editing_val && config_item != 4) {
        snprintf(line2, sizeof(line2), "%s <ADJ>", value_str);
    } else {
        snprintf(line2, sizeof(line2), "%c %s", (config_item == 4 ? '>' : ' '), value_str);
    }

    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(line1);
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(line2);
}

void update_display()
{
    taskENTER_CRITICAL();
    if (current_ui_mode == UI_MODE_DASHBOARD) {
        render_dashboard();
    } else if (current_ui_mode == UI_MODE_MAIN_MENU) {
        render_menu();
    } else if (current_ui_mode == UI_MODE_TEST_MENU) {
        render_test_menu();
    } else if (current_ui_mode == UI_MODE_CONFIG_EDIT) {
        render_config_edit();
    }
    taskEXIT_CRITICAL();
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  Enable_Internal_Pullups(); // Anti-ruido I2C
  I2C_Scan();                // Escaneo

  // Cargar configuración guardada
  Load_Config_From_Flash();

  HD44780_Init(2);
  HD44780_Clear();

  HD44780_SetCursor(0, 0);
  HD44780_PrintStr("TDII: INCUBADORA");


  HAL_Delay(1000);
  HD44780_SetCursor(0, 0);
  HD44780_PrintStr("                ");

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of menuQueue */
  osMessageQDef(menuQueue, 32, uint32_t);
  menuQueueHandle = osMessageCreate(osMessageQ(menuQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of menuTask */
  osThreadDef(menuTask, StartMenuTask, osPriorityNormal, 0, 512);
  menuTaskHandle = osThreadCreate(osThread(menuTask), NULL);

  /* definition and creation of debounceTask */
  osThreadDef(debounceTask, StartDebounceTask, osPriorityIdle, 0, 128);
  debounceTaskHandle = osThreadCreate(osThread(debounceTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, StartMotorTask, osPriorityIdle, 0, 128);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|COOLER_Pin|DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HUMIDIFICADOR_Pin|buzzer_Pin|LUZ_Pin|MOTOR_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : LD2_Pin DHT11_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_SW_Pin */
  GPIO_InitStruct.Pin = ENCODER_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENCODER_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HUMIDIFICADOR_Pin buzzer_Pin LUZ_Pin MOTOR_Pin */
  GPIO_InitStruct.Pin = HUMIDIFICADOR_Pin|buzzer_Pin|LUZ_Pin|MOTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : COOLER_Pin */
  GPIO_InitStruct.Pin = COOLER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(COOLER_GPIO_Port, &GPIO_InitStruct);

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MOTOR_SENSOR_PIN) {
        motor_pulse_count++;
        return;
    }

    if (GPIO_Pin == ENCODER_A_Pin || GPIO_Pin == ENCODER_B_Pin) {
        static uint8_t last_state = 0;
        static int8_t counter = 0;

        uint8_t state_A = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin);
        uint8_t state_B = HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);
        uint8_t current_state = (state_A << 1) | state_B;

        if (current_state == last_state) return;

        const int8_t transition_table[16] = {
             0, -1,  1,  0,
             1,  0,  0, -1,
            -1,  0,  0,  1,
             0,  1, -1,  0,
        };

        int8_t transition = transition_table[(last_state << 2) | current_state];

        if (transition != 0) {
            counter += transition;
            if (counter >= 4) {
                MenuEvent_t event = ENCODER_RIGHT;
                osMessagePut(menuQueueHandle, (uint32_t)event, 0);
                counter -= 4;
            } else if (counter <= -4) {
                MenuEvent_t event = ENCODER_LEFT;
                osMessagePut(menuQueueHandle, (uint32_t)event, 0);
                counter += 4;
            }
        }
        last_state = current_state;
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMenuTask */
/**
  * @brief  Function implementing the menuTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMenuTask */
void StartMenuTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
    update_display();

    osEvent event;
    uint32_t last_draw_time = 0;
    int needs_update = 0;

    // Variables para "Update on Change" del Dashboard
    float last_disp_temp = -999.0f;
    float last_disp_hum = -999.0f;
    uint16_t last_disp_rpm = 9999;
    HumidifierState_t last_disp_hum_state = HUM_STATE_IDLE;
    uint8_t last_disp_day = 255;
    uint8_t last_disp_is_running = 255;

    for(;;)
    {
        // Timeout de 200ms para chequear cambios frecuentemente sin saturar I2C
        event = osMessageGet(menuQueueHandle, 200);

        if (event.status == osEventMessage) {
            MenuEvent_t initial_event = (MenuEvent_t)event.value.v;

            if (initial_event == BUTTON_LONG_PRESS) {
                // Volver atrás / Reset
                if (current_ui_mode == UI_MODE_TEST_MENU || current_ui_mode == UI_MODE_CONFIG_EDIT) {
                    current_ui_mode = UI_MODE_MAIN_MENU;
                } else if (current_ui_mode == UI_MODE_MAIN_MENU) {
                    current_ui_mode = UI_MODE_DASHBOARD;
                }
                needs_update = 1;
                continue;
            }

            int8_t net_movement = 0;
            int button_pressed = 0;

            if (initial_event == ENCODER_RIGHT) net_movement++;
            else if (initial_event == ENCODER_LEFT) net_movement--;
            else if (initial_event == BUTTON_PRESS) button_pressed = 1;

            int batch_limit = 20;
            while (batch_limit-- > 0 && (event = osMessageGet(menuQueueHandle, 0)).status == osEventMessage) {
                MenuEvent_t batch_evt = (MenuEvent_t)event.value.v;
                if (batch_evt == ENCODER_RIGHT) net_movement++;
                else if (batch_evt == ENCODER_LEFT) net_movement--;
            }

            // --- LÓGICA DE INTERFAZ ---

            if (current_ui_mode == UI_MODE_DASHBOARD) {
                if (button_pressed) {
                    current_ui_mode = UI_MODE_MAIN_MENU;
                    selected_item = 0;
                    needs_update = 1;
                }
            }
            else if (current_ui_mode == UI_MODE_MAIN_MENU) {
                // Navegación Menú Principal
                if (net_movement != 0) {
                    int16_t new_pos = selected_item + net_movement;
                    selected_item = ((new_pos % menu_size) + menu_size) % menu_size;

                    if (selected_item >= menu_top_item + LCD_ROWS) menu_top_item = selected_item - (LCD_ROWS - 1);
                    else if (selected_item < menu_top_item) menu_top_item = selected_item;
                    needs_update = 1;
                }
                if (button_pressed) {
                    if (selected_item == 1) { // Configuracion
                        current_ui_mode = UI_MODE_CONFIG_EDIT;
                        sysData.current_stage_idx = 0; // Default Etapa 1
                        config_item = 0;
                        is_editing_val = 0;
                    } else if (selected_item == 2) { // Test
                        current_ui_mode = UI_MODE_TEST_MENU;
                        test_selected_item = 0;
                    } else { // Ver Sensores (Volver)
                        current_ui_mode = UI_MODE_DASHBOARD;
                    }
                    needs_update = 1;
                }
            }
            else if (current_ui_mode == UI_MODE_TEST_MENU) {
                // Menú Test
                if (net_movement != 0) {
                    int16_t new_pos = test_selected_item + net_movement;
                    test_selected_item = ((new_pos % test_menu_size) + test_menu_size) % test_menu_size;
                    if (test_selected_item >= test_top_item + LCD_ROWS) test_top_item = test_selected_item - (LCD_ROWS - 1);
                    else if (test_selected_item < test_top_item) test_top_item = test_selected_item;
                    needs_update = 1;
                }
                if (button_pressed) {
                    toggle_output(test_selected_item);
                    needs_update = 1;
                }
            }
            else if (current_ui_mode == UI_MODE_CONFIG_EDIT) {
                // Editor de Configuración
                if (button_pressed) {
                    if (config_item == 4) { // SALIR
                        Save_Config_To_Flash(); // Guardar cambios
                        current_ui_mode = UI_MODE_MAIN_MENU;
                    } else {
                        is_editing_val = !is_editing_val; // Toggle edición
                    }
                    needs_update = 1;
                }

                if (net_movement != 0) {
                    if (!is_editing_val) {
                        config_item += net_movement;
                        if (config_item < 0) config_item = 4;
                        if (config_item > 4) config_item = 0;
                    } else {
                        // Modificar valores
                        StageConfig_t *st = &sysData.stages[sysData.current_stage_idx];
                        switch(config_item) {
                            case 0: st->end_day += net_movement;
                                    if(st->end_day < 1) st->end_day = 1;
                                    break;
                            case 1: st->temp_target += (net_movement * 0.1f); break;
                            case 2: st->hum_target += (net_movement * 1.0f); break;
                            case 3: if(net_movement!=0) st->motor_on = !st->motor_on; break;
                        }
                    }
                    needs_update = 1;
                }
            }
        }
        else if (event.status == osEventTimeout) {
            // Chequeo inteligente de cambios
            if (current_ui_mode == UI_MODE_DASHBOARD) {
                // Obtenemos valores actuales para comparar
                uint8_t day = Get_Current_Day();

                // Comparamos con la última vez que dibujamos
                if (current_temp != last_disp_temp ||
                    current_hum != last_disp_hum ||
                    global_rpm != last_disp_rpm ||
                    hum_state != last_disp_hum_state ||
                    day != last_disp_day ||
                    sysData.is_running != last_disp_is_running)
                {
                    needs_update = 1;
                }
            }
        }

        uint32_t current_time = HAL_GetTick();
        if (needs_update && (current_time - last_draw_time >= MIN_DRAW_INTERVAL_MS)) {

            // Actualizamos referencias ANTES de dibujar
            if (current_ui_mode == UI_MODE_DASHBOARD) {
                last_disp_temp = current_temp;
                last_disp_hum = current_hum;
                last_disp_rpm = global_rpm;
                last_disp_hum_state = hum_state;
                last_disp_day = Get_Current_Day();
                last_disp_is_running = sysData.is_running;
            }

            update_display();
            last_draw_time = current_time;
            needs_update = 0;
        }
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDebounceTask */
/**
* @brief Function implementing the debounceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDebounceTask */
void StartDebounceTask(void const * argument)
{
  /* USER CODE BEGIN StartDebounceTask */
    uint8_t integrator = 0;
    uint8_t prev_state = 1;
    uint32_t hold_counter = 0;
    uint8_t long_press_sent = 0;

    for(;;)
    {
        osDelay(DEBOUNCE_POLL_RATE_MS);
        uint8_t pin_state = HAL_GPIO_ReadPin(ENCODER_SW_GPIO_Port, ENCODER_SW_Pin);

        if (pin_state == GPIO_PIN_RESET) {
            if (integrator < DEBOUNCE_THRESHOLD) integrator++;
        } else {
            if (integrator > 0) integrator--;
        }

        if (integrator >= DEBOUNCE_THRESHOLD && prev_state == 1) {
            prev_state = 0;
            hold_counter = 0;
            long_press_sent = 0;
            MenuEvent_t event = BUTTON_PRESS;
            osMessagePut(menuQueueHandle, (uint32_t)event, 0);
        }
        else if (prev_state == 0 && integrator >= DEBOUNCE_THRESHOLD) {
            hold_counter++;
            if (hold_counter >= LONG_PRESS_TICKS && long_press_sent == 0) {
                long_press_sent = 1;
                MenuEvent_t event = BUTTON_LONG_PRESS;
                osMessagePut(menuQueueHandle, (uint32_t)event, 0);
            }
        }
        else if (integrator == 0 && prev_state == 0) {
            prev_state = 1;
        }
    }
  /* USER CODE END StartDebounceTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorTask */
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
