/*
 * ble_adv_controller.h
 *
 *  Created on: 12 de ago de 2017
 *      Author: Gabriel
 */

#ifndef SRC_BLE_BLE_ADVERTISING_H_
#define SRC_BLE_BLE_ADVERTISING_H_


void advertising_init();
void advertising_start();
void advertising_stop();
void advertising_on_sensor_event(sensor_event_t *p_sensor_evt);

#endif /* SRC_BLE_BLE_ADVERTISING_H_ */
