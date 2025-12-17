/*
 * mde.c
 *
 *  Created on: 12 nov. 2025
 *      Author: Magdalena
 *      MAQUINAS DE ESTADO Y LOGICA GENERAL DEL PROYECTO
 */
#include "main.h"
#include "dht11.h"
#include "rele.h"
#include "config.h"
#include <string.h>
#include <stdio.h>

//extern uint32_t humedad; //dht11.h
//extern uint32_t temperatura; //dht11.h
extern UART_HandleTypeDef huart1; //main
static uint32_t alarma_temp_countdown = -1;
static uint32_t alarma_humedad_countdown = -1;
// 0 = no hay alarma, 1 = alarma, 2 = alarma critica
static uint8_t estado_alarma_temp = 0;
static uint8_t estado_alarma_humedad = 0;

static char msg_temp[128];
static char msg_hum[128];
/*
 * Resetea los timers desde que la humedad o temperatura estan fuera de rango
 * En caso de haber entrado en alarma se envia un mensaje avisando que retorno a normalidad
 * Usadas por control_parametros()
 */
void reset_alarma_temp(uint32_t temperatura){
	int len = 0;
	apagar_buzzer();
	estado_alarma_temp = 0;
	// Si se habia entrado en alarma se envia un mensaje avisando que se soluciono
	if(estado_alarma_temp == 1)
		len = snprintf(msg_temp, sizeof(msg_temp), "Temperatura se autoregulo: %02lu °C\r\n", temperatura);
	else if(estado_alarma_temp == 2)
		len = snprintf(msg_temp, sizeof(msg_temp), "ATENCION: Temperatura se autoregulo: %02lu °C, se recomienda investigar posible falla\r\n", temperatura);
	if(len > 0)
		HAL_UART_Transmit(&huart1, (uint8_t*)msg_temp, len, 1000);
	alarma_temp_countdown = HAL_GetTick();
}

void reset_alarma_humedad(uint32_t humedad){
	int len = 0;
	apagar_buzzer();
	estado_alarma_humedad = 0;
	// Si se habia entrado en alarma se envia un mensaje avisando que se soluciono
	if(estado_alarma_humedad == 1)
		len = snprintf(msg_hum, sizeof(msg_hum), "Humedad se autoregulo: %02lu \r\n", humedad);
	else if(estado_alarma_humedad == 2)
		len = snprintf(msg_hum, sizeof(msg_hum), "ATENCION: Humedad se autoregulo: %02lu , se recomienda investigar posible falla\r\n", humedad);
	if(len > 0)
		HAL_UART_Transmit(&huart1, (uint8_t*)msg_hum, len, 1000);
	alarma_humedad_countdown = HAL_GetTick();
}

/*
 * Controla que la humedad y temperatura no esten fuera de rango por mas tiempo que TIEMPO_ALARMA_MS
 * En caso de superar TIEMPO_ALARMA_MS enciende el buzzer y envia un mensaje de telegram
 * Si pasado el tiempo TIEMPO_ALARMA_CRITICA_MS el sistema sigue sin poder regularse envia otro mensaje de telegram
 * TIEMPO_ALARMA_MS y TIEMPO_ALARMA_CRITICA_MS estan definidos en config.h
 */
void control_alarma(uint32_t temperatura, uint32_t humedad){
	if(HAL_GetTick() - alarma_temp_countdown > TIEMPO_ALARMA_MS){
		if(estado_alarma_temp == 0){
			estado_alarma_temp = 1; //alarma
			encender_buzzer();
			int len = snprintf(msg_temp, sizeof(msg_temp), "ATENCION: la temperatura en la incubadora es %02lu °C, y no se puede autoregular. Asistirla en persona\r\n", temperatura);
		  	HAL_UART_Transmit(&huart1, (uint8_t*)msg_temp, len, 1000);
		}
		else if(estado_alarma_temp == 1 && HAL_GetTick() - alarma_temp_countdown > TIEMPO_ALARMA_CRITICA_MS){
			estado_alarma_temp = 2; //emergencia (reenvio mensaje)
			int len = snprintf(msg_temp, sizeof(msg_temp), "ADVERTENCIA: la temperatura en la incubadora es %02lu °C, y no se puede autoregular. Asistirla en persona\r\n", temperatura);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg_temp, len, 1000);
	  	}
	}

	if(HAL_GetTick() - alarma_humedad_countdown > TIEMPO_ALARMA_MS){
		if(estado_alarma_humedad == 0){
			estado_alarma_humedad = 1; //alarma
			encender_buzzer();
		  	int len = snprintf(msg_hum, sizeof(msg_hum), "ATENCION: la humedad en la incubadora es %02lu , y no se puede autoregular. Asistirla en persona\r\n", humedad);
		  	HAL_UART_Transmit(&huart1, (uint8_t*)msg_hum, len, 1000);
		}
		else if(estado_alarma_humedad == 1 && HAL_GetTick() - alarma_humedad_countdown > TIEMPO_ALARMA_CRITICA_MS){
			estado_alarma_humedad = 2; //emergencia (reenvio mensaje)
			int len = snprintf(msg_hum, sizeof(msg_hum), "ADVERTENCIA: la humedad en la incubadora es %02lu , y no se puede autoregular. Asistirla en persona\r\n", humedad);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg_hum, len, 1000);
	  	}
	}
}

/*
 * Mantiene parametros dentro de histeresis
 */
void control_parametros(uint32_t temperatura, uint32_t humedad)
{
	if(estado_alarma_temp == -1)
		reset_alarma_temp(temperatura);

	if(estado_alarma_humedad == -1)
		reset_alarma_humedad(humedad);

    // Control de temperatura
    if (temperatura > TEMP_ETAPA1_MAX_TOLERABLE){ //mucho calor
        apagar_lampara();
        encender_cooler();
        if(temperatura > TEMP_ETAPA1_MAX_CRITICA)
        	alerta_telegram();
    }
    else if (temperatura < TEMP_ETAPA1_MIN_TOLERABLE){ // mucho frio
        encender_lampara();
        apagar_cooler();
        if(temperatura < TEMP_ETAPA1_MIN_CRITICA)
        	alerta_telegram();
    }
    else{reset_alarma_temp(temperatura);} //La temperatura esta en rango

    // Control de humedad
    if (humedad > HUMEDAD_ETAPA1_MAX_TOLERABLE){ // mucha humedad
    	apagar_humidificador();
    	if(temperatura > TEMP_ETAPA1_MIN_TOLERABLE) //Si no hace frio (control temp tiene mayor prioridad)
    		encender_cooler();
    	if(humedad > HUMEDAD_ETAPA1_MAX_CRITICA)
    	    alerta_telegram();
    }
    else if (humedad < HUMEDAD_ETAPA1_MIN_TOLERABLE) { // poca humedad
        encender_humidificador();
    	if(temperatura < TEMP_ETAPA1_MAX_TOLERABLE) //Si no hace calor (control temp tiene mayor prioridad)
    		apagar_cooler();
    	if(humedad < HUMEDAD_ETAPA1_MIN_CRITICA)
    	    alerta_telegram();
    }
    else{reset_alarma_humedad(humedad);} //La humedad esta en rango

    control_alarma(temperatura, humedad);
}
