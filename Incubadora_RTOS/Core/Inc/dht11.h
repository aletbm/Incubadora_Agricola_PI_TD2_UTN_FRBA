/*
 * dht11.h
 *
 *  Created on: Aug 28, 2025
 *      Author: Magdalena
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

void start_timer(void);
void reset_timer(void);
uint32_t read_timer(void);

void toggle_pin_mode(uint8_t);

void DHT11_Init(void);
void DHT11_GetDatos(void);

#endif /* INC_DHT11_H_ */
