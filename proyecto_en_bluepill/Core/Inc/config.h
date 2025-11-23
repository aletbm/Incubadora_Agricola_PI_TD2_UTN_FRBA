/*
 *      Archivo para la configuracion de datos escenciales como:
 *      -histereiss de temperatura y humedad
 *      -red wifi para comunicacion inalambrica
 *      -id de usuario telegram
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

// WIFI Y TELEGRAM
#define ssid NoConectarQueTeHackea //nombre de red wifi
#define password pangea01 //contraseña de red wifi
#define chat_id 6326727457 //Id chat telegram

// TEMPERATURA (°C)
#define temp_max 39
#define temp_min 36
// HUMEDAD (%)
#define hum_max 60
#define hum_min 50

// Tiempo que se tolera para que la temperatura o humedad
// vuelva a rango antes de disparar alarma
#define tiempo_alarma 120000 // 2 minutos
// Tiempo donde se reenvia un mensaje para resaltar la emergencia de la situacion
#define tiempo_alarma_critica 600000 // 10 minutos


#endif /* INC_CONFIG_H_ */
