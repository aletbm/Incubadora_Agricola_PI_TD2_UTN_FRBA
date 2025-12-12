/*
 * dht11.h
 *
 *  Created on: Aug 28, 2025
 *      Author: Magdalena
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

void prueba(uint16_t);
void start_timer(void);
void reset_timer(void);
uint32_t read_timer(void);

void toggle_pin_mode(uint8_t);
void DHT11_Init(void);
int8_t DHT11_Leo_datos(void);
uint8_t DHT11_Espero_datos(void);
void DHT11_Solicitar_datos(void);
uint8_t DHT11_Lectura(void);

uint32_t DHT11_GetTemperatura(void);
uint32_t DHT11_GetHumedad(void);
float* DHT11_GetDatos(void);

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define us_timer_Pin GPIO_PIN_1
#define us_timer_GPIO_Port GPIOA
#define DHT11_Pin GPIO_PIN_11
#define DHT11_GPIO_Port GPIOA

typedef enum {
    ST_IDLE,
    ST_SEND_START,
    ST_WAIT_RESPONSE,
    ST_LOW,
    ST_HIGH,
    ST_WAIT_BIT,
    ST_READ_BIT,
    ST_PROCESS_DATA,
    ST_ERROR
} State_DHT11_t;
#endif /* INC_DHT11_H_ */
