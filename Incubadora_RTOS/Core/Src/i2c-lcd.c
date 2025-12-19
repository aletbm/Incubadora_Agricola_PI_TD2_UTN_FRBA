#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>    // Agregado para seguridad con tipos
#include <string.h>
#include <stdio.h>

/* --- AGREGAR ESTO EN ESTE ORDEN EXACTO --- */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* ----------------------------------------- */

#include "i2c-lcd.h"
#include "utils.h"

extern I2C_HandleTypeDef hi2c1;
extern SemaphoreHandle_t xLcdMutex;


uint8_t dpFunction;
uint8_t dpControl;
uint8_t dpMode;
uint8_t dpRows;
uint8_t dpBacklight;

uint8_t special1[8] = {
        0b00000,
        0b11001,
        0b11011,
        0b00110,
        0b01100,
        0b11011,
        0b10011,
        0b00000
};

uint8_t special2[8] = {
		0b11000,
        0b11000,
        0b00110,
        0b01001,
        0b01000,
        0b01001,
        0b00110,
        0b00000
};

// --- VARIABLES DE UI (Menú Principal Actualizado) ---
// Nota: El item index 1 es dinámico (INICIAR/PAUSAR)
char* menu_items[] = {"Ver Sensores", "Ciclo: ???   ", "Configuracion", "TEST", "Volver <-"};
int8_t selected_item = 0;
const int8_t menu_size = 5;
int8_t menu_top_item = 0;

const int8_t test_menu_size = 4;
int8_t test_selected_item = 0;
int8_t test_top_item = 0;

// Variables Config Select
// ACTUALIZADO: Agregado "Ajustar Tiempo"
char* config_menu_items[] = {"CFG: Desarrollo", "CFG:Eclosion", "Ciclo Total", "Ajustar Tiempo", "Volver <-"};
int8_t config_sel_index = 0;
int8_t config_top_index = 0;
const int8_t config_menu_sz = 5; // Aumentado a 5

// Variables Config Edit
int8_t config_item = 0;
int8_t is_editing_val = 0;

// --- VARIABLES DE SISTEMA ---
UIMode_t old_ui_mode = UI_MODE_DASHBOARD;
UIMode_t current_ui_mode = UI_MODE_DASHBOARD;


void SendCommand(uint8_t cmd)
{
  Send(cmd, 0);
}

void SendChar(uint8_t ch)
{
  Send(ch, RS);
}

void Send(uint8_t value, uint8_t mode)
{
  uint8_t highnib = value & 0xF0;
  uint8_t lownib = (value<<4) & 0xF0;
  Write4Bits((highnib)|mode);
  Write4Bits((lownib)|mode);
}

void Write4Bits(uint8_t value)
{
  ExpanderWrite(value);
  PulseEnable(value);
}

void ExpanderWrite(uint8_t _data)
{
  uint8_t data = _data | dpBacklight;
  HAL_I2C_Master_Transmit(&hi2c1, DEVICE_ADDR, &data, 1, 100);
}

void PulseEnable(uint8_t _data)
{
  ExpanderWrite(_data | ENABLE);
  DelayUS(1);

  ExpanderWrite(_data & ~ENABLE);
  DelayUS(50);
}

void DelayInit(void)
{
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;

  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

  DWT->CYCCNT = 0;

  /* 3 NO OPERATION instructions */
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");
}

void DelayUS(uint32_t us) {
  uint32_t cycles = (SystemCoreClock/1000000L)*us;
  uint32_t start = DWT->CYCCNT;
  volatile uint32_t cnt;

  do
  {
    cnt = DWT->CYCCNT - start;
  } while(cnt < cycles);
}

void HD44780_Init(uint8_t rows)
{
  dpRows = rows;
  dpBacklight = LCD_BACKLIGHT;

  // Configuración inicial: 4 bits, 1 línea, fuentes 5x8
  dpFunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;

  if (dpRows > 1) {
    dpFunction |= LCD_2LINE;
  }

  /* 1. Espera inicial larga */
  // Es vital dar tiempo al LCD para que su voltaje interno se estabilice
  DelayInit();
  HAL_Delay(100); // Aumentado a 100ms
  LCD_ForceReset();
  /* 2. Sincronización de interfaz (Software Reset) */
  // Este paso es el que evita los "caracteres extraños".
  // Forzamos al LCD a modo 8 bits 3 veces para que se resetee internamente.

  SendCommand(LCD_FUNCTIONSET | dpFunction);

  dpControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  HD44780_Display();

  HD44780_Clear();

  dpMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  SendCommand(LCD_ENTRYMODESET | dpMode);

  HAL_Delay(5);

  HD44780_CreateSpecialChar(0, special1);
  HD44780_CreateSpecialChar(1, special2);

  HD44780_Home();
}

void HD44780_Clear()
{
  SendCommand(LCD_CLEARDISPLAY);
  DelayUS(2000);
}

void HD44780_Home()
{
  SendCommand(LCD_RETURNHOME);
  DelayUS(2000);
}

void HD44780_SetCursor(uint8_t col, uint8_t row)
{
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if (row >= dpRows)
  {
    row = dpRows-1;
  }
  SendCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void HD44780_NoDisplay()
{
  dpControl &= ~LCD_DISPLAYON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_Display()
{
  dpControl |= LCD_DISPLAYON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_NoCursor()
{
  dpControl &= ~LCD_CURSORON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_Cursor()
{
  dpControl |= LCD_CURSORON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_NoBlink()
{
  dpControl &= ~LCD_BLINKON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_Blink()
{
  dpControl |= LCD_BLINKON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_ScrollDisplayLeft(void)
{
  SendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void HD44780_ScrollDisplayRight(void)
{
  SendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void HD44780_LeftToRight(void)
{
  dpMode |= LCD_ENTRYLEFT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

void HD44780_RightToLeft(void)
{
  dpMode &= ~LCD_ENTRYLEFT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

void HD44780_AutoScroll(void)
{
  dpMode |= LCD_ENTRYSHIFTINCREMENT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

void HD44780_NoAutoScroll(void)
{
  dpMode &= ~LCD_ENTRYSHIFTINCREMENT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

void HD44780_CreateSpecialChar(uint8_t location, uint8_t charmap[])
{
  location &= 0x7;
  SendCommand(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++)
  {
    SendChar(charmap[i]);
  }
}

void HD44780_PrintSpecialChar(uint8_t index)
{
  SendChar(index);
}

void HD44780_LoadCustomCharacter(uint8_t char_num, uint8_t *rows)
{
  HD44780_CreateSpecialChar(char_num, rows);
}

void HD44780_PrintStr(const char c[])
{
  while(*c){
	  SendChar(*c++);
  }
}

void HD44780_SetBacklight(uint8_t new_val)
{
  if(new_val) HD44780_Backlight();
  else HD44780_NoBacklight();
}

void HD44780_NoBacklight(void)
{
  dpBacklight=LCD_NOBACKLIGHT;
  ExpanderWrite(dpBacklight);
}

void HD44780_Backlight(void)
{
  dpBacklight=LCD_BACKLIGHT;
  ExpanderWrite(dpBacklight);
}

/**
 * @brief Habilita las resistencias pull-up internas para el bus I2C1
 *
 * Esta función configura los pines GPIO asociados al bus I2C1
 * (PB8 = SCL, PB9 = SDA) en modo Alternate Function Open-Drain
 * con resistencias pull-up internas habilitadas.
 *
 * Las resistencias pull-up son necesarias para el correcto
 * funcionamiento del bus I2C, ya que las líneas SCL y SDA
 * operan en configuración open-drain.
 *
 * Nota:
 * En aplicaciones finales se recomienda el uso de resistencias
 * pull-up externas para cumplir con las especificaciones del bus
 * I2C y garantizar mejores tiempos de subida.
 */
void Enable_Internal_Pullups(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Habilitar clock del puerto GPIOB */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configuración de pines PB8 (SCL) y PB9 (SDA) */
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

    /* Inicialización de los pines GPIO */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void LCD_ForceReset(void)
{
    /* Apagar backlight (opcional pero recomendable) */
    ExpanderWrite(0x00);
    HAL_Delay(20);

    /* Según datasheet HD44780:
       Forzar modo 8 bits 3 veces */
    for (int i = 0; i < 3; i++) {
        ExpanderWrite(0x30);      // 0b0011xxxx → 8-bit mode
        PulseEnable(0x30);
        HAL_Delay(5);
    }

    /* Pasar a modo 4 bits */
    ExpanderWrite(0x20);          // 0b0010xxxx → 4-bit mode
    PulseEnable(0x20);
    HAL_Delay(5);
}

/**
 * @brief Inicializa el LCD y muestra un mensaje de bienvenida
 *
 * Inicializa el controlador HD44780, limpia la pantalla y muestra
 * un mensaje en la primera línea durante un tiempo determinado.
 *
 * @param rows Cantidad de filas del LCD (ej: 2 o 4)
 * @param msg  Mensaje a mostrar en la primera línea
 * @param delay_ms Tiempo en milisegundos que el mensaje permanece visible
 */
void LCD_ShowWelcome(const char *msg)
{
    char buffer[17];
    /* Limpiar SOLO una vez */
    HD44780_Clear();

    /* Preparar mensaje padded */
    snprintf(buffer, sizeof(buffer), "%-16.16s", msg);

    /* Mostrar mensaje */
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(buffer);
}

/**
 * @brief Renderiza la pantalla principal (Dashboard) del sistema.
 *
 * Muestra en el LCD:
 *  - Temperatura actual y objetivo
 *  - Humedad actual
 *  - Estado del control de humedad
 *  - Día y hora del ciclo de incubación
 *
 * El contenido varía dependiendo de si el sistema está en ejecución
 * o en pausa.
 */
void render_dashboard(void)
{
    char buffer[17];                     // Buffer para línea LCDA
    char tmp[17];
    float target_t = 0;                  // Temperatura objetivo
    float target_h = 0;                  // Humedad objetivo (no mostrada aquí)
    uint8_t mon = 0;                     // Estado del motor (no usado en dashboard)

    // Si el ciclo está activo, mostrar temperatura actual / objetivo
    if (sysData.is_running) {
        Get_Active_Targets(&target_t, &target_h, &mon);

        // Formato: T:37.7/37.7
        snprintf(tmp, sizeof(tmp),"T:%04.1f/%04.1f", liveStatus.temp_current, target_t);
    }
    // Si está en pausa, mostrar solo temperatura actual
    else {
    	snprintf(tmp, sizeof(tmp),"T:%04.1f (PAUSA)", liveStatus.temp_current);
    }

    // Escribir línea superior del LCD
	snprintf(buffer, sizeof(buffer), "%-16.16s", tmp);
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(buffer);

    // Determinar símbolo de estado de humedad
    char hum_char = ' ';
    if (hum_state == HUM_STATE_DOSING)   hum_char = '*'; // Humidificando
    if (hum_state == HUM_STATE_COOLDOWN) hum_char = 'w'; // En enfriamiento

    // Línea inferior del LCD
    if (sysData.is_running) {
        snprintf(tmp, sizeof(tmp),
                 "H:%02.0f%%%c D:%02d H:%02d",
                 liveStatus.hum_current,
                 hum_char,
                 liveStatus.day_current,
                 liveStatus.hour_current);
    }
    else {
        snprintf(tmp, sizeof(tmp),
                 "H:%02.0f%% STANDBY ",
                 liveStatus.hum_current);
    }


	snprintf(buffer, sizeof(buffer), "%-16.16s", tmp);
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(buffer);
}

/**
 * @brief Renderiza el menú principal del sistema.
 *
 * Muestra una lista desplazable de opciones con cursor.
 * El texto de la opción "Iniciar/Pausar ciclo" se actualiza
 * dinámicamente según el estado del sistema.
 */
void render_menu(void)
{
    char buffer[17];   // Buffer de línea
    for (int i = 0; i < LCD_ROWS; i++) {
        int item_index = menu_top_item + i;

        if (item_index < menu_size) {
            // Mostrar cursor en el ítem seleccionado
            char cursor = (item_index == selected_item) ? '>' : ' ';

            // Texto dinámico para iniciar/pausar ciclo
            if (item_index == 1) {
                snprintf(buffer, sizeof(buffer),
                         "%c%-15.15s",
                         cursor,
                         sysData.is_running ? "PAUSAR CICLO" : "INICIAR CICLO");
            }
            // Texto normal del menú
            else {
                snprintf(buffer, sizeof(buffer),
                         "%c%-15.15s",
                         cursor,
                         menu_items[item_index]);
            }
        } else {
            // Línea vacía con 16 espacios para limpiar residuos
            snprintf(buffer, sizeof(buffer), "%-16.16s", " ");
        }
        HD44780_SetCursor(0, i);
        HD44780_PrintStr(buffer);
    }
}


/**
 * @brief Renderiza el menú de pruebas de salidas.
 *
 * Permite activar/desactivar manualmente salidas digitales
 * para verificación de hardware.
 */
void render_test_menu(void)
{
    char line_buffer[20];

    for (int i = 0; i < LCD_ROWS; i++) {
        int item_index = test_top_item + i;

        if (item_index < test_menu_size) {
            char cursor = (item_index == test_selected_item) ? '>' : ' ';
            char state_char = (test_outputs[item_index].state) ? 'X' : ' ';

            snprintf(line_buffer, sizeof(line_buffer),
                     "%c[%c] %-9s",
                     cursor,
                     state_char,
                     test_outputs[item_index].name);
        }
        else {
            snprintf(line_buffer, sizeof(line_buffer), "%-16s", " ");
        }

        HD44780_SetCursor(0, i);
        HD44780_PrintStr(line_buffer);
    }
}

/**
 * @brief Renderiza el menú de selección de configuración.
 *
 * Permite elegir entre configuración global, por etapas
 * u otras opciones de ajuste.
 */
void render_config_select(void)
{
    char line_buffer[20];

    for (int i = 0; i < LCD_ROWS; i++) {
        int item_index = config_top_index + i;

        if (item_index < config_menu_sz) {
            char cursor = (item_index == config_sel_index) ? '>' : ' ';
            snprintf(line_buffer, sizeof(line_buffer),
                     "%c%-15.15s",
                     cursor,
                     config_menu_items[item_index]);
        }
        else {
            snprintf(line_buffer, sizeof(line_buffer), "%-16s", " ");
        }

        HD44780_SetCursor(0, i);
        HD44780_PrintStr(line_buffer);
    }
}

/**
 * @brief  Renderiza la pantalla de configuración de parámetros generales del sistema.
 * * Esta interfaz permite gestionar valores globales que no dependen de una etapa específica,
 * como la duración total del proceso de incubación. Maneja visualmente la navegación
 * entre los ítems y el estado de edición de los mismos.
 * * @note   Utiliza 'sysData.total_days' para la persistencia del valor y 'config_item'
 * para determinar el foco del cursor.
 * * @retval None
 */
void render_config_global(void) {
    char line1[20];      // Buffer para la línea de encabezado
    char line2[20];      // Buffer para la línea de parámetros/acción
    char temp_buff[20];  // Buffer temporal para construcción de strings

    // --- LÍNEA 1: Encabezado Estático ---
    // Se utiliza relleno manual de espacios para asegurar que se limpie el residuo
    // de pantallas anteriores en el controlador del LCD.
    snprintf(line1, sizeof(line1), "CFG: GENERAL    ");

    // --- LÍNEA 2: Lógica de Visualización de Parámetros ---
    // Dependiendo de config_item, se muestra el parámetro ajustable o la acción de guardado
    if (config_item == 0) {
        /* Caso 0: Ajuste de la duración total del ciclo */
        if(is_editing_val) {
            // Estado: El usuario está modificando el número de días
            snprintf(temp_buff, sizeof(temp_buff), "TotDias:%d <ADJ>", sysData.total_days);
        } else {
            // Estado: Navegación (foco sobre el ítem de días totales)
            snprintf(temp_buff, sizeof(temp_buff), "> TotDias:%d", sysData.total_days);
        }
    } else {
        /* Caso 1: Opción para confirmar y persistir los cambios */
        snprintf(temp_buff, sizeof(temp_buff), "> [GUARDAR]");
    }

    // --- FORMATEO Y LIMPIEZA ---
    // El especificador %-16s garantiza que el string se alinee a la izquierda
    // y se rellene con espacios hasta los 16 caracteres, evitando "fantasmas"
    // visuales de textos más largos renderizados anteriormente.
    snprintf(line2, sizeof(line2), "%-16s", temp_buff);

    // --- SALIDA A HARDWARE (I2C) ---
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(line1);

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(line2);
}

/**
 * @brief  Renderiza la pantalla de edición para los parámetros de una etapa de incubación.
 * * Esta función permite al usuario visualizar y modificar los ajustes específicos de la etapa
 * actual (por ejemplo, Temperatura, Humedad, Día Final y Motor). Utiliza una técnica de
 * "sobrescritura con espacios" para evitar el parpadeo del LCD que causaría un comando Clear.
 * * @note   Depende de 'sysData' para los valores de las etapas y de las variables globales
 * 'config_item' e 'is_editing_val' para el estado de la UI.
 * * @retval None
 */
void render_config_edit(void) {
    char line1[20];      // Almacena la cadena final formateada para la fila 0
    char line2[20];      // Almacena la cadena final formateada para la fila 1
    char temp_buff[20];  // Buffer auxiliar para construir el texto antes del padding

    /* Obtener referencia a la etapa actual mediante puntero para optimizar acceso */
    StageConfig_t *stage = &sysData.stages[sysData.current_stage_idx];

    // --- FILA 1: Título de la Etapa ---
    // Se construye el nombre de la configuración y se aplica padding de 16 espacios
    // para asegurar que cualquier texto anterior quede borrado.
    snprintf(temp_buff, sizeof(temp_buff), "CFG: %s", stage->name);
    snprintf(line1, sizeof(line1), "%-16s", temp_buff);

    // --- FILA 2: Selector de Parámetros ---
    char value_str[12];

    /* Traducción del índice de ítem a una cadena representativa del valor */
    switch(config_item) {
        case 0: // Día de finalización de la etapa actual
            snprintf(value_str, sizeof(value_str), "FinDia:%d", stage->end_day);
            break;
        case 1: // Objetivo de temperatura
            snprintf(value_str, sizeof(value_str), "T:%.1f", stage->temp_target);
            break;
        case 2: // Objetivo de humedad relativa
            snprintf(value_str, sizeof(value_str), "H:%.0f%%", stage->hum_target);
            break;
        case 3: // Estado del motor de volteo
            snprintf(value_str, sizeof(value_str), "Mot:%s", stage->motor_on ? "ON" : "OFF");
            break;
        case 4: // Opción de salida del sub-menú
            snprintf(value_str, sizeof(value_str), "[SALIR]");
            break;
    }

    /* Gestión del cursor y modo de edición */
    if (is_editing_val && config_item != 4) {
        // Indica visualmente que el sistema está esperando una entrada del usuario (<ADJ>)
        snprintf(temp_buff, sizeof(temp_buff), "%s <ADJ>", value_str);
    } else {
        // Modo navegación: muestra '>' solo si el foco está en el botón [SALIR]
        snprintf(temp_buff, sizeof(temp_buff), "%c %s", (config_item == 4 ? '>' : ' '), value_str);
    }

    /* Aplicar padding final para limpiar la fila 2 del LCD */
    snprintf(line2, sizeof(line2), "%-16s", temp_buff);

    // --- TRANSMISIÓN I2C ---
    // Envío de las tramas de texto al controlador HD44780
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(line1);

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(line2);
}

/**
 * @brief  Renderiza la interfaz de ajuste manual de tiempo del ciclo.
 * * Permite al usuario modificar el progreso actual del ciclo de incubación (Día, Hora y Minuto).
 * Esta función visualiza variables temporales de edición, lo que evita modificar el conteo
 * en tiempo real del sistema hasta que el usuario selecciona la opción "[GUARDAR]".
 * * @note   Utiliza las variables globales 'edit_day', 'edit_hour' y 'edit_min' como buffers
 * de entrada antes de la persistencia de datos.
 * @retval None
 */
void render_config_time(void) {
    char line1[20];      // Buffer para el encabezado
    char line2[20];      // Buffer para el parámetro seleccionado
    char temp_buff[20];  // Buffer auxiliar para construcción de strings

    // --- LÍNEA 1: Título de la sección ---
    // Se utiliza relleno de espacios para limpiar residuos visuales.
    snprintf(line1, sizeof(line1), "AJUSTAR TIEMPO  ");

    // --- LÍNEA 2: Selección de Parámetro Temporal ---
    char val_str[10];

    /* Mapeo del ítem seleccionado a su respectiva variable de edición */
    switch(config_item) {
        case 0: // Ajuste del día actual del ciclo
            snprintf(val_str, 10, "Dia:%d", edit_day);
            break;
        case 1: // Ajuste de la hora
            snprintf(val_str, 10, "Hr:%d", edit_hour);
            break;
        case 2: // Ajuste de los minutos
            snprintf(val_str, 10, "Min:%d", edit_min);
            break;
        case 3: // Acción de confirmación y guardado
            snprintf(val_str, 10, "[GUARDAR]");
            break;
    }

    /* Gestión visual del modo Edición vs. Navegación */
    if (is_editing_val && config_item != 3) {
        // Muestra indicador <ADJ> cuando el valor está siendo incrementado/decrementado
        snprintf(temp_buff, sizeof(temp_buff), "%s <ADJ>", val_str);
    } else {
        // Muestra cursor '>' solo cuando el foco está en el botón de Guardar
        snprintf(temp_buff, sizeof(temp_buff), "%c %s", (config_item == 3 ? '>' : ' '), val_str);
    }

    // --- RENDERIZADO FINAL ---
    // Aplicación de padding de 16 caracteres para asegurar una fila limpia en el LCD
    snprintf(line2, sizeof(line2), "%-16s", temp_buff);

    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(line1);

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(line2);
}

void clear_LCD(void) {
	SendCommand(LCD_CLEARDISPLAY);       // Clear display
	DelayUS(2000);
    HD44780_SetCursor(0, 0); // Vuelve al inicio de la pantalla
}


/**
 * @brief Actualiza el contenido del LCD según el modo actual de la UI.
 *
 * Esta función centraliza el renderizado de pantallas y asegura
 * acceso exclusivo al bus I2C/LCD mediante una sección crítica.
 *
 * @note No debe incluir retardos internos.
 */
void update_display(void)
{
	// Proteger el acceso al LCD frente a ISR o tareas concurrentes
	taskENTER_CRITICAL();
	if      (current_ui_mode == UI_MODE_DASHBOARD)      render_dashboard();
	else if (current_ui_mode == UI_MODE_MAIN_MENU)      render_menu();
	else if (current_ui_mode == UI_MODE_TEST_MENU)      render_test_menu();
	else if (current_ui_mode == UI_MODE_CONFIG_SELECT)  render_config_select();
	else if (current_ui_mode == UI_MODE_CONFIG_EDIT)    render_config_edit();
	else if (current_ui_mode == UI_MODE_CONFIG_GLOBAL)  render_config_global();
	else if (current_ui_mode == UI_MODE_CONFIG_TIME)    render_config_time();
	taskEXIT_CRITICAL();
}


/**
 * @brief Recupera el funcionamiento del LCD ante fallas de comunicación
 *
 * Esta función reinicializa el display LCD basado en HD44780,
 * limpia la pantalla y muestra un mensaje de alerta indicando
 * que se realizó un reinicio de emergencia del display.
 *
 * Luego del mensaje, la pantalla se limpia nuevamente y se
 * redibuja la interfaz actual del sistema.
 *
 * Se utiliza un retardo para garantizar la correcta
 * inicialización del controlador LCD.
 */
void recover_lcd(void)
{
    /* Reinicializar LCD */
    HD44780_Init(2);
    HD44780_Clear();

    /* Pequeña espera para estabilizar el display */
    osDelay(50);

    /* Mensaje de advertencia */
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("! PANIC RESET ! ");

    /* Mostrar mensaje durante 1 segundo */
    osDelay(1000);

    /* Limpiar pantalla y restaurar la UI */
    HD44780_Clear();
    update_display();
}

/**
 * @brief Escanea el bus I2C en busca de dispositivos conectados
 *
 * Esta función recorre todas las direcciones posibles del bus I2C
 * y verifica si existe un dispositivo que responda en cada una.
 *
 * Las direcciones detectadas se informan por consola UART,
 * lo cual resulta útil para depuración y validación de hardware.
 *
 * El escaneo se realiza sobre el periférico I2C1 y reporta
 * únicamente los dispositivos que responden correctamente.
 */
void I2C_Scan(void)
{
    /* Mensaje de inicio del escaneo */
    char info[] = "Escaneando bus I2C...\r\n";
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)info,
                      sizeof(info) - 1,
                      100);

    HAL_StatusTypeDef res;

    /* Recorrer todas las direcciones I2C posibles */
    for (uint16_t i = 0; i < 128; i++) {

        /* Comprobar si un dispositivo responde en la dirección */
        res = HAL_I2C_IsDeviceReady(&hi2c1,
                                   (uint16_t)(i << 1),
                                   1,
                                   10);

        if (res == HAL_OK) {
            char msg[64];
            snprintf(msg, sizeof(msg),
                     "-> Disp: 0x%02X\r\n", i);

            HAL_UART_Transmit(&huart2,
                              (uint8_t *)msg,
                              strlen(msg),
                              100);
        }
    }

    /* Mensaje de fin de escaneo */
    char end_info[] = "Fin del escaneo.\r\n";
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)end_info,
                      sizeof(end_info) - 1,
                      100);
}
