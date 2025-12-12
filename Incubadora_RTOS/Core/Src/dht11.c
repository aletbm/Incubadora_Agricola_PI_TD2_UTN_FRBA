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

static float humedad = 0;
static float temperatura = 0;
//int8_t flag = 0;
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
	return __HAL_TIM_GET_COUNTER(&htim1);	//el tim no para en el debug. Acá lo leo. En 2^16 se recarga solo
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
    HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET); // Idle data-pin alto

    HAL_Delay(1500); // Un segundo de espera desde alimentacion recomendado por el datasheet.
    //El delay deberia ejecutarse ante cada muestreo
    //HAL_Delay(100); //pequeño delay de precaucion
    start_timer(); // Inicio timer de microsegundos
}


// Retorna solo la humedad del array dht11_data
float* DHT11_GetDatos(void){

	uint32_t dht11_data[5] = {0};      // Buffer para almacenar los 40 bits de datos
	uint32_t data_ciclos_bajo[40] = {0};// Buffer para almacenar los tiempos de pulso bajo para cada bit
	uint32_t data_ciclos_alto[40] = {0};// Buffer para almacenar los tiempos de pulso alto para cada bit
	//DHT11_Solicitar_datos() ---------------------------------------------------------------------------
	toggle_pin_mode(1); //pongo pin en output
	reset_timer();
	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
	while (read_timer() < 22000); // pulldown por 22ms, de minima es 18ms, esto da margen de deteccion por el dht
	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
	while (read_timer() < 30); // up por 30us. From here, solo escucho al dht.

	//DHT11_Espero_datos() ---------------------------------------------------------------------------
	toggle_pin_mode(0); //pongo pin en input
    reset_timer();
    while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == GPIO_PIN_RESET) {
      if (read_timer() >= 100) { // Deberia responder en 80 us. Estos son Timeouts para evitar cuelgue.
//    	  flag = 1;
    	  return NULL;
      }
    }

    reset_timer();
    while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == GPIO_PIN_SET) {
      if (read_timer() >= 100) { // Deberia responder en 80 us
//    	  flag = 2;
        return NULL;
      }
    }
    //reset_timer();
    //From here, Comienza la transmision

	for(int i=0; i < 40; i++)
	{
		reset_timer();
		while(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == GPIO_PIN_RESET){	//el comienzo es un bajo
		if (read_timer() >= 90){
//			flag = read_timer();
			return NULL;}}
		data_ciclos_bajo[i] = read_timer();
		reset_timer();
		while(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == GPIO_PIN_SET){	//mido el tiempo en alto
		if (read_timer() >= 100){
//			flag = 4;
		return NULL;}}
		data_ciclos_alto[i] = read_timer();
		//int x=0;
	}

	// De acuerdo a si la mayoria del pulso fue alto o bajo puedo determinar si es un 1 o 0
	for(int i=0; i < 40; ++i){
		dht11_data[i / 8] <<= 1;	//desplazo un lugar y agrego cero en LSB
		if (data_ciclos_alto[i] > data_ciclos_bajo[i]) {
		  // Si el ciclo en alto duro mas entonces es un 1
			dht11_data[i / 8] |= 1;
		}
	}
	if (dht11_data[4] == (dht11_data[0] + dht11_data[1] + dht11_data[2] + dht11_data[3])) {		//verifico chksum
		humedad = dht11_data[0] + (dht11_data[1]) * 0.1;
		temperatura = dht11_data[2] + (dht11_data[3] & 0x0f) * 0.1;
//		flag--;

		static float datos[2];
		datos[0] = humedad;
		datos[1] = temperatura;

		return datos;
	}
	else
		return NULL;
}

/* Resumen:
 * Se unificaron las 3 funciones en una sola.
 * Cada timeout asigna un valor distinto a FLAG, variable global que permite ver donde estoy
 * parado en tiempo real.
 * Un FLAG decreciente significa "conversion correcta"
 * Si FLAG adquiere un valor, cayó en un timeout.
 * Se cambiaron los return -1 por NULL para adaptarlo al funcionamiento en "proyecto bluepill"
 *
 */


