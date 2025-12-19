/*
 * eeprom.h
 *
 *  Created on: Dec 14, 2025
 *      Author: Alexander
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define FLASH_USER_START_ADDR 0x08060000 // Sector 7 (F446RE)

void Load_Config_From_Flash(void);
void Save_Config_To_Flash(void);

#endif /* INC_EEPROM_H_ */
