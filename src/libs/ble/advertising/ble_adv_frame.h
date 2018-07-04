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
	uint32_t pres;   /*<< Pressure */
	int16_t temp; 	 /*<< Temperature*/
	uint16_t hum;     /*<< Humidity */
	uint32_t vis_lux; /*<< Visible Lux */
	uint32_t ir_lux;  /*<< Infrared Lux */
}ble_adv_data_t;

#endif /* SRC_BLE_BLE_ADV_FRAME_H_ */
