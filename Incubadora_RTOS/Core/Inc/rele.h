#ifndef RELE_H
#define RELE_H

#include "main.h"
#include "config.h"
// Estados
#define RELE_ACTIVO   GPIO_PIN_RESET
#define RELE_INACTIVO GPIO_PIN_SET

// Prototipos de funciones
void RELES_Init(void);
void RELES_EncenderTodos(void);
void RELES_ApagarTodos(void);
void RELES_ToggleTodos(void);

// Funciones individuales
void encender_buzzer(void);
void apagar_buzzer(void);
uint8_t estado_buzzer(void);

void encender_humidificador(void);
void apagar_humidificador(void);
uint8_t estado_humidificador(void);

void encender_lampara(void);
void apagar_lampara(void);
uint8_t estado_lampara(void);

void encender_cooler(void);
void apagar_cooler(void);
uint8_t estado_cooler(void);

#endif
