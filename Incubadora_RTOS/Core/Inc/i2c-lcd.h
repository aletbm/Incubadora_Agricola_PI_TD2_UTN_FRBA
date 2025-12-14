#ifndef LIQUIDCRYSTAL_I2C_H_
#define LIQUIDCRYSTAL_I2C_H_

#include "stm32f4xx_hal.h"

/* Command */
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

/* Entry Mode */
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

/* Display On/Off */
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

/* Cursor Shift */
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

/* Function Set */
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

/* Backlight */
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

/* Enable Bit */
#define ENABLE 0x04

/* Read Write Bit */
#define RW 0x0

/* Register Select Bit */
#define RS 0x01

/* Device I2C Address */
#define DEVICE_ADDR     (0x27 << 1)

#define LCD_ROWS 2
#define LCD_COMMAND_DELAY       5
#define MIN_DRAW_INTERVAL_MS    50

static void SendCommand(uint8_t);
static void SendChar(uint8_t);
static void Send(uint8_t, uint8_t);
static void Write4Bits(uint8_t);
static void ExpanderWrite(uint8_t);
static void PulseEnable(uint8_t);
static void DelayInit(void);
static void DelayUS(uint32_t);

void HD44780_Init(uint8_t rows);
void HD44780_Clear();
void HD44780_Home();
void HD44780_NoDisplay();
void HD44780_Display();
void HD44780_NoBlink();
void HD44780_Blink();
void HD44780_NoCursor();
void HD44780_Cursor();
void HD44780_ScrollDisplayLeft();
void HD44780_ScrollDisplayRight();
void HD44780_PrintLeft();
void HD44780_PrintRight();
void HD44780_LeftToRight();
void HD44780_RightToLeft();
void HD44780_ShiftIncrement();
void HD44780_ShiftDecrement();
void HD44780_NoBacklight();
void HD44780_Backlight();
void HD44780_AutoScroll();
void HD44780_NoAutoScroll();
void HD44780_CreateSpecialChar(uint8_t, uint8_t[]);
void HD44780_PrintSpecialChar(uint8_t);
void HD44780_SetCursor(uint8_t, uint8_t);
void HD44780_SetBacklight(uint8_t new_val);
void HD44780_LoadCustomCharacter(uint8_t char_num, uint8_t *rows);
void HD44780_PrintStr(const char[]);

void I2C_Scan(void);
void Enable_Internal_Pullups(void);

void LCD_ShowWelcome(uint8_t rows, const char *msg, uint32_t delay_ms)
void I2C_Scan(void);
void Enable_Internal_Pullups(void);
void recover_lcd(void);
void update_display(void);
void render_dashboard(void);
void render_menu(void);
void render_test_menu(void);
void render_config_select(void);
void render_config_edit(void);
void render_config_global(void);
void render_config_time(void);

typedef enum {
    UI_MODE_DASHBOARD,
    UI_MODE_MAIN_MENU,
    UI_MODE_TEST_MENU,
    UI_MODE_CONFIG_SELECT, // Selección de etapa
    UI_MODE_CONFIG_EDIT,   // Edición de etapa
    UI_MODE_CONFIG_GLOBAL, // Edición global (Ciclo Total)
    UI_MODE_CONFIG_TIME    // Edición de Tiempo (Dia/Hora/Min)
} UIMode_t;

typedef enum {
    ENCODER_RIGHT,
    ENCODER_LEFT,
    BUTTON_PRESS,
    BUTTON_LONG_PRESS
} MenuEvent_t;

extern UIMode_t current_ui_mode;
extern int8_t config_sel_index;
extern int8_t config_top_index;
extern const int8_t config_menu_sz;
extern int8_t config_item;
extern int8_t is_editing_val;
extern const int8_t test_menu_size;
extern int8_t test_selected_item;
extern int8_t test_top_item;

#endif /* LIQUIDCRYSTAL_I2C_H_ */
