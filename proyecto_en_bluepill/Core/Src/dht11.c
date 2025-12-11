/*
 * dht11.c
 *
 *  Created on: Aug 28, 2025
 *      Author: Magdalena
 */
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "dht11.h"

static uint32_t humedad = 0;
static uint32_t temperatura = 0;
static uint32_t x = 0;
static uint32_t y = 0;

extern TIM_HandleTypeDef htim1;
/*Funcion para iniciar el timer*/
void start_timer(void){
	HAL_TIM_Base_Start(&htim1);
}
/*Funcion para resetar el timer*/
void reset_timer(void){
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
}
/*Retorna el valor actual del timer 1*/
uint32_t read_timer(void){
	return __HAL_TIM_GET_COUNTER(&htim1);
}
/*Funcion para enviar pulsos de un dado periodo al pin us_timer (A1)*/
void prueba(uint16_t tiempo){
  if (__HAL_TIM_GET_COUNTER(&htim1) > tiempo){
	  //HAL_GPIO_TogglePin(us_timer_GPIO_Port, us_timer_Pin);
	  HAL_GPIO_TogglePin(GPIOA, DHT11_Pin);
	  reset_timer();
  }
}
/*cambia el modo del pin de input a output y viceversa*/
void toggle_pin_mode(uint8_t mode) { //1 output 0 input
    static uint8_t current_mode = 1; // Empieza en Output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_Pin;

    if(current_mode != mode){ //chequeo que no este en ese modo
    	current_mode = mode;
        if (mode) {
            // Modo Output
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        } else {
            // Modo Input
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        }
        GPIO_InitStruct.Pull = GPIO_NOPULL;

        HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
    }
}
/*Funcion inicializadora del sensor de humedad DHT11*/
void DHT11_Init(void) {
    // Configurar pin como salida y poner en alto
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, DHT11_Pin, GPIO_PIN_SET); // Idle data-pin alto

    HAL_Delay(1000); // Un segundo de espera desde alimentacion recomendado por el datasheet
    start_timer(); // Inicio timer de microsegundos
}

void DHT11_Solicitar_datos(void) {
	toggle_pin_mode(1); //pongo pin en output
	reset_timer();
	HAL_GPIO_WritePin(GPIOA, DHT11_Pin, GPIO_PIN_RESET);
    while (read_timer() < 22000); // pulldown por 20ms
    HAL_GPIO_WritePin(GPIOA, DHT11_Pin, GPIO_PIN_SET);
    while (read_timer() < 30); // up por 30us
}

uint8_t DHT11_Espero_datos(void) {
	toggle_pin_mode(0); //pongo pin en input  /0=input, 1=output
    reset_timer();
    while (HAL_GPIO_ReadPin(GPIOA, DHT11_Pin) == GPIO_PIN_RESET) {
      if (read_timer() >= 100) { // Deberia responder en 80 us
        return 0;
      }
    }

    reset_timer();
    while (HAL_GPIO_ReadPin(GPIOA, DHT11_Pin) == GPIO_PIN_SET) {
      if (read_timer() >= 100) { // Deberia responder en 80 us
        return 0;
      }
    }
    return 1;
}

uint8_t DHT11_Leo_datos(void) {
	uint32_t dht11_data[5] = {0};      // Buffer para almacenar los 40 bits de datos
	uint32_t data_ciclos_bajo[40] = {0};// Buffer para almacenar los tiempos de pulso bajo para cada bit
	uint32_t data_ciclos_alto[40] = {0};// Buffer para almacenar los tiempos de pulso alto para cada bit
	for(int i=0; i < 41; i++){
		if(i != 0){
			reset_timer();
			while(HAL_GPIO_ReadPin(GPIOA, DHT11_Pin) == GPIO_PIN_RESET){

			}
			data_ciclos_bajo[i-1] = read_timer();
			reset_timer();
			while(HAL_GPIO_ReadPin(GPIOA, DHT11_Pin) == GPIO_PIN_SET);
			data_ciclos_alto[i-1] = read_timer();
		}
		else{
			reset_timer();
			while(HAL_GPIO_ReadPin(GPIOA, DHT11_Pin) == GPIO_PIN_RESET);
			data_ciclos_bajo[i] = read_timer();
			reset_timer();
			while(HAL_GPIO_ReadPin(GPIOA, DHT11_Pin) == GPIO_PIN_SET);
			data_ciclos_alto[i] = read_timer();
		}
	}

	// De acuerdo a si la mayoria del pulso fue alto o bajo puedo determinar si es un 1 o 0
	for(int i=0; i < 40; ++i){
		dht11_data[i / 8] <<= 1;
	    if (data_ciclos_alto[i] > data_ciclos_bajo[i]) {
	      // Si el ciclo en alto duro mas entonces es un 1
	    	dht11_data[i / 8] |= 1;
	    }
	}
	if (dht11_data[4] == (dht11_data[0] + dht11_data[1] + dht11_data[2] + dht11_data[3])) {
		humedad = dht11_data[0] + dht11_data[1] * 0.1;
		temperatura = dht11_data[2] + (dht11_data[3] & 0x0f) * 0.1;
		return 1;
	}
	else
		return 0;

}

uint32_t* DHT11_GetTempHumedad(void){
	static uint32_t datos[3]; ;
	DHT11_Solicitar_datos();
	if(DHT11_Espero_datos())
		if(DHT11_Leo_datos()){
			datos[0] = humedad;
			datos[1] = temperatura;
			return datos;}
		else
			return NULL;
	else
		return NULL;
}

// Me retorna solo la temperatura del array dht11_data
uint32_t DHT11_GetTemperatura(void) {
	DHT11_Solicitar_datos();
	if(DHT11_Espero_datos())
		if(DHT11_Leo_datos())
			return temperatura;
		else
			return -1;
	else
		return -1;
}

// Retorna solo la humedad del array dht11_data
uint32_t DHT11_GetHumedad(void){
	DHT11_Solicitar_datos();
	if(DHT11_Espero_datos())
		if(DHT11_Leo_datos())
			return humedad;
		else
			return -1;
	else
		return -1;
}
/*
uint8_t DHT11_Lectura(void) {
    static State_DHT11_t estado = ST_SEND_START;
    static uint8_t arr_data[40] = {0};
    static uint8_t i = 0;
    //static uint32_t start_time = 0;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    switch(estado) {
    	case ST_SEND_START:
            // Configuro pin DATA como salida
            GPIO_InitStruct.Pin = DHT11_Pin;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

            // Inicializo lectura
            HAL_GPIO_WritePin(GPIOA, DHT11_Pin, GPIO_PIN_RESET);
            reset_timer();
            if (__HAL_TIM_GET_COUNTER(&htim1) > 18000){ // Si pasaron al menos 18 ms
                // Poner bus en HIGH y configurar como entrada
                HAL_GPIO_WritePin(GPIOA, DHT11_Pin, GPIO_PIN_SET);

                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

                reset_timer();
                estado = ST_WAIT_RESPONSE;
            }
            break;

        case ST_WAIT_RESPONSE:
            // Espero flanco de bajada con timeout
            if (HAL_GPIO_ReadPin(GPIOA, DHT11_Pin) == GPIO_PIN_RESET) {
                estado = ST_LOW;
                reset_timer();
            }
            else if (__HAL_TIM_GET_COUNTER(&htim1) > 10000){ { // Timeout 10ms
                estado = ST_ERROR;
            }
            break;

        case ST_LOW:
            // Esperar flanco de subida (80us)
            if (HAL_GPIO_ReadPin(GPIOA, DHT11_Pin) == GPIO_PIN_SET) {
                estado = ST_HIGH;
                reset_timer();
            }
            else if (__HAL_TIM_GET_COUNTER(&htim1) > 10000){ // Timeout 10ms
                estado = ST_ERROR;
            }
            break;

        case ST_HIGH:
            // Esperar flanco de bajada para el inicio del primer bit
            if (HAL_GPIO_ReadPin(GPIOA, DHT11_Pin) == GPIO_PIN_RESET) {
                estado = ST_WAIT_BIT;
                i = 0;
                memset(arr_data, 0, sizeof(arr_data));
                reset_timer();
            }
            else if (__HAL_TIM_GET_COUNTER(&htim1) > 10000) { // Timeout 10ms
                estado = ST_ERROR;
            }
            break;

        case ST_WAIT_BIT:
        	if(__HAL_TIM_GET_COUNTER(&htim1) > 63){ //Pasaron 63 micro segundos, paso a leer el medio del bit
        		reset_timer();
        		estado = ST_READ_BIT;
        	}
    		break;

        case ST_READ_BIT:
    		arr_data[i] = HAL_GPIO_ReadPin(GPIOA, DHT11_Pin);
    		i++;
        	if(__HAL_TIM_GET_COUNTER(&htim1) > 14){ // Pasaron 14 micro segundos
        		if(i < 40){
        			estado = ST_WAIT_BIT;
        			reset_timer();
        		}
        		else
        			estado = ST_PROCESS_DATA;
        	}
        	break;

        case ST_PROCESS_DATA:
            // Convertir 40 bits a 5 bytes
            for (uint8_t j = 0; j < 5; j++) {
                dht11_data[j] = 0;
                for (uint8_t k = 0; k < 8; k++) {
                    if (arr_data[j * 8 + k]) {
                        dht11_data[j] |= (1 << (7 - k));
                    }
                }
            }

            // Verifico checksum
            if (dht11_data[4] == (dht11_data[0] + dht11_data[1] + dht11_data[2] + dht11_data[3])) {
                estado = ST_IDLE;
            } else {
                estado = ST_ERROR;
            }
            break;

        case ST_ERROR:
            estado = ST_SEND_START;
            //return 0;
            break;

        case ST_IDLE:
        default:
            // Configurar pin como salida en estado alto
            GPIO_InitStruct.Pin = DHT11_Pin;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            HAL_GPIO_WritePin(GPIOA, DHT11_Pin, GPIO_PIN_SET);

            estado = ST_SEND_START;
            return 0;
    }
}
}

*/
