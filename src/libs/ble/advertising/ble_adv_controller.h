/*
 * ble_adv_controller.h
 *
 *  Created on: 12 de ago de 2017
 *      Author: Gabriel
 */

#ifndef SRC_BLE_BLE_ADVERTISING_H_
#define SRC_BLE_BLE_ADVERTISING_H_

#include "sensor_public_interface.h"

void advertising_init();
void advertising_start();
void advertising_stop();
ret_code_t advertising_update_adv_int(uint16_t ms);
ret_code_t advertising_update_tx_pwr(int8_t tx_pwr);
void adv_on_sensor_event(sensor_evt_t *p_sensor_evt);


#endif /* SRC_BLE_BLE_ADVERTISING_H_ */
