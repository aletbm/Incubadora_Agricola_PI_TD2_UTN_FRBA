#include "rele.h"
#include "main.h"
#include "config.h"
#include <stdio.h>

// Variables privadas para estado de los relés
static uint8_t buzzer_estado = 0;
static uint8_t humidificador_estado = 0;
static uint8_t lampara_estado = 0;
static uint8_t cooler_estado = 0;
static uint8_t motor_estado = 0;

void RELES_Init(void)
{
    RELES_ApagarTodos();
}

/*
 * FUNCIONES PARA DEBUGGEO
 */
void RELES_EncenderTodos(void)
{
	HAL_GPIO_WritePin(HUMIFICADOR_GPIO_Port, HUMIDIFICADOR_Pin, RELE_ACTIVO);
	HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, RELE_ACTIVO);
	HAL_GPIO_WritePin(LAMPARA_GPIO_Port, LAMPARA_Pin, RELE_ACTIVO);
	HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, RELE_ACTIVO);
    //HAL_GPIO_WritePin(GPIOB, buzzer_Pin | humidificador_Pin | lampara_Pin | cooler_Pin, RELE_ACTIVO);
    motor_estado = 1;
    humidificador_estado = 1;
    lampara_estado = 1;
    cooler_estado = 1;
}
void RELES_ApagarTodos(void)
{
	//HAL_GPIO_WritePin(GPIOB, buzzer_Pin | humidificador_Pin | lampara_Pin | cooler_Pin, RELE_INACTIVO);
	HAL_GPIO_WritePin(HUMIDIFICADOR_GPIO_Port, HUMIDIFICADOR_Pin, RELE_INACTIVO);
	HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, RELE_INACTIVO);
	HAL_GPIO_WritePin(LAMPARA_GPIO_Port, LAMPARA_Pin, RELE_INACTIVO);
	HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, RELE_INACTIVO);

	motor_estado = 0;
    humidificador_estado = 0;
    lampara_estado = 0;
    cooler_estado = 0;
}
void RELES_ToggleTodos(void)
{
	HAL_GPIO_TogglePin(HUMIDIFICADOR_GPIO_Port, HUMIDIFICADOR_Pin);
	HAL_GPIO_TogglePin(MOTOR_GPIO_Port, MOTOR_Pin);
	HAL_GPIO_TogglePin(LAMPARA_GPIO_Port, LAMPARA_Pin);
	HAL_GPIO_TogglePin(COOLER_GPIO_Port, COOLER_Pin);

	motor_estado = !motor_estado;
    humidificador_estado = !humidificador_estado;
    lampara_estado = !lampara_estado;
    cooler_estado = !cooler_estado;
}
// BUZZER
void encender_buzzer(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RELE_ACTIVO);
    buzzer_estado = 1;
}
void apagar_buzzer(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RELE_INACTIVO);
    buzzer_estado = 0;
}

uint8_t estado_buzzer(void){
	return buzzer_estado;
}

// HUMIDIFICADOR
void encender_humidificador(void)
{
    HAL_GPIO_WritePin(HUMIDIFICADOR_GPIO_Port, HUMIDIFICADOR_Pin, RELE_ACTIVO);
    humidificador_estado = 1;
}
void apagar_humidificador(void)
{
    HAL_GPIO_WritePin(HUMIDIFICADOR_GPIO_Port, HUMIDIFICADOR_Pin, RELE_INACTIVO);
    humidificador_estado = 0;
}
uint8_t estado_humidificador(void){
	return humidificador_estado;
}

// LAMPARA
void encender_lampara(void)
{
    HAL_GPIO_WritePin(LAMPARA_GPIO_Port, LAMPARA_Pin, RELE_ACTIVO);
    lampara_estado = 1;
}
void apagar_lampara(void)
{
    HAL_GPIO_WritePin(LAMPARA_GPIO_Port, LAMPARA_Pin, RELE_INACTIVO);
    lampara_estado = 0;
}
uint8_t estado_lampara(void){
	return lampara_estado;
}

// COOLER
void encender_cooler(void)
{
    HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, RELE_ACTIVO);
    cooler_estado = 1;
}

void apagar_cooler(void)
{
    HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, RELE_INACTIVO);
    cooler_estado = 0;
}
uint8_t estado_cooler(void){
	return cooler_estado;
}

// MOTOR
void encender_motor(void)
{
    HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, RELE_ACTIVO);
    motor_estado = 1;
}

void apagar_motor(void)
{
    HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, RELE_INACTIVO);
    motor_estado = 0;
}
uint8_t estado_motor(void){
	return motor_estado;
}
