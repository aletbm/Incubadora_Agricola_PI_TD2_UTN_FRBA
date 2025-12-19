/*
 * config.h
 *
 *  Created on: 12 dic. 2025
 *      Author: Alexander
 */
#include "main.h"

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define MOTOR_SENSOR_PIN	GPIO_PIN_0
#define MOTOR_SENSOR_PORT	GPIOA

#define DEBOUNCE_POLL_RATE_MS   5
#define DEBOUNCE_THRESHOLD      4
#define LONG_PRESS_MS           2000
#define LONG_PRESS_TICKS        (LONG_PRESS_MS / DEBOUNCE_POLL_RATE_MS)

#define ENCODER_SLOTS           20
#define MOTOR_CALC_INTERVAL_MS  1000

#define DHT_READ_INTERVAL_MS    30000   // Leer sensor cada 30 seg
#define CONTROL_LOOP_MS         1000    // Recalcular lógica cada 1 seg
#define HUM_DOSE_TIME_MS        10000   // Humidificador ON 10 seg
#define HUM_COOLDOWN_TIME_MS    300000  // Espera 5 min

//ETAPA 1 -> Hasta los primeros 18 dias de incubacion.
//ETAPA 2 -> Desde el dia 18 hasta el dia 21.

#define	TEMP_ETAPA1_MAX_CRITICA	40
#define	TEMP_ETAPA1_MAX_TOLERABLE	38
#define	TEMP_ETAPA1_MIN_TOLERABLE	37
#define	TEMP_ETAPA1_MIN_CRITICA	35

#define	TEMP_ETAPA2_MAX_CRITICA	38
#define	TEMP_ETAPA2_MAX_TOLERABLE	36
#define	TEMP_ETAPA2_MIN_TOLERABLE	35
#define	TEMP_ETAPA2_MIN_CRITICA	33

#define	HUMEDAD_ETAPA1_MAX_CRITICA	75
#define	HUMEDAD_ETAPA1_MAX_TOLERABLE	60
#define	HUMEDAD_ETAPA1_MIN_TOLERABLE	55
#define	HUMEDAD_ETAPA1_MIN_CRITICA	40

#define	HUMEDAD_ETAPA2_MAX_CRITICA	80
#define	HUMEDAD_ETAPA2_MAX_TOLERABLE	75
#define	HUMEDAD_ETAPA2_MIN_TOLERABLE	70
#define	HUMEDAD_ETAPA2_MIN_CRITICA	60

#define TIEMPO_ALARMA_MS	1800000	//30 min
#define TIEMPO_ALARMA_CRITICA_MS	3600000	//1 hora

#endif /* INC_CONFIG_H_ */
