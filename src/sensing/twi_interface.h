/*
 * twi_interface.h
 *
 *  Created on: 12 de ago de 2017
 *      Author: Gabriel
 */

#ifndef SRC_SENSING_TWI_INTERFACE_H_
#define SRC_SENSING_TWI_INTERFACE_H_

#include <stdbool.h>
#include <stdint.h>

void twi_init();
void twi_tx(uint8_t address, uint8_t *p_data, uint8_t length, bool no_stop);
void twi_rx(uint8_t address, uint8_t *p_data, uint8_t length);



#endif /* SRC_SENSING_TWI_INTERFACE_H_ */
