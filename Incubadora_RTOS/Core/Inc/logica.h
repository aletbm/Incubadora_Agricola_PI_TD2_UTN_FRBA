/*
 * logica.h
 *
 *  Created on: 22 nov. 2025
 *      Author: Magdalena
 */

#ifndef INC_LOGICA_H_
#define INC_LOGICA_H_

void reset_alarma_temp(uint32_t temperatura);
void reset_alarma_humedad(uint32_t humedad);
void control_alarma(uint32_t temperatura, uint32_t humedad);
void control_parametros(uint32_t temperatura, uint32_t humedad);



#endif /* INC_LOGICA_H_ */
