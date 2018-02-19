/*
 * ble_adv_frame.h
 *
 *  Created on: 12 de ago de 2017
 *      Author: Gabriel
 */

#ifndef SRC_BLE_BLE_ADV_FRAME_H_
#define SRC_BLE_BLE_ADV_FRAME_H_

typedef struct __attribute__((packed)) ble_adv_data
{
	uint32_t pressure;
	int16_t temperature;
	uint16_t humidity;
	uint32_t visible_lux;
	uint32_t infrared_lux;
}ble_adv_data_t;

#endif /* SRC_BLE_BLE_ADV_FRAME_H_ */
