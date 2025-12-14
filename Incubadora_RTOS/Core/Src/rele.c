#include "rele.h"
#include "main.h"
#include "config.h"
#include <stdio.h>

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
}
void RELES_ApagarTodos(void)
{
	//HAL_GPIO_WritePin(GPIOB, buzzer_Pin | humidificador_Pin | lampara_Pin | cooler_Pin, RELE_INACTIVO);
	HAL_GPIO_WritePin(HUMIDIFICADOR_GPIO_Port, HUMIDIFICADOR_Pin, RELE_INACTIVO);
	HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, RELE_INACTIVO);
	HAL_GPIO_WritePin(LAMPARA_GPIO_Port, LAMPARA_Pin, RELE_INACTIVO);
	HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, RELE_INACTIVO);
}
void RELES_ToggleTodos(void)
{
	HAL_GPIO_TogglePin(HUMIDIFICADOR_GPIO_Port, HUMIDIFICADOR_Pin);
	HAL_GPIO_TogglePin(MOTOR_GPIO_Port, MOTOR_Pin);
	HAL_GPIO_TogglePin(LAMPARA_GPIO_Port, LAMPARA_Pin);
	HAL_GPIO_TogglePin(COOLER_GPIO_Port, COOLER_Pin);
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
	return HAL_GPIO_ReadPin(BUZZER_GPIO_Port, BUZZER_Pin);
}

// HUMIDIFICADOR
void encender_humidificador(void)
{
    HAL_GPIO_WritePin(HUMIDIFICADOR_GPIO_Port, HUMIDIFICADOR_Pin, RELE_ACTIVO);
}
void apagar_humidificador(void)
{
    HAL_GPIO_WritePin(HUMIDIFICADOR_GPIO_Port, HUMIDIFICADOR_Pin, RELE_INACTIVO);
}
uint8_t estado_humidificador(void){
	return HAL_GPIO_ReadPin(HUMIDIFICADOR_GPIO_Port, HUMIDIFICADOR_Pin);
}

// LAMPARA
void encender_lampara(void)
{
    HAL_GPIO_WritePin(LAMPARA_GPIO_Port, LAMPARA_Pin, RELE_ACTIVO);
}
void apagar_lampara(void)
{
    HAL_GPIO_WritePin(LAMPARA_GPIO_Port, LAMPARA_Pin, RELE_INACTIVO);
}
uint8_t estado_lampara(void){
	HAL_GPIO_ReadPin(LAMPARA_GPIO_Port, LAMPARA_Pin);
}

// COOLER
void encender_cooler(void)
{
    HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, RELE_ACTIVO);
}

void apagar_cooler(void)
{
    HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, RELE_INACTIVO);
}
uint8_t estado_cooler(void){
	return HAL_GPIO_ReadPin(COOLER_GPIO_Port, COOLER_Pin);
}

// MOTOR
void encender_motor(void)
{
    HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, RELE_ACTIVO);
}

void apagar_motor(void)
{
    HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, RELE_INACTIVO);
}
uint8_t estado_motor(void){
	return	HAL_GPIO_ReadPin(MOTOR_GPIO_Port, MOTOR_Pin);
}
