/*
 * htu21d_drv.h
 *
 *  Created on: 16 de nov de 2017
 *      Author: Gabriel
 */

#ifndef SRC_LIBS_SENSING_HTU21D_DRV_HTU21D_DRV_H_
#define SRC_LIBS_SENSING_HTU21D_DRV_HTU21D_DRV_H_

/*******************************************
 * HTU21D Public Enums
 ******************************************/
typedef enum htu21d_resolution
{
    HTU21D_RES_RH_12_TEMP_14 = 0x02,
    HTU21D_RES_RH_8_TEMP_12 = 0x03,
    HTU21D_RES_RH_10_TEMP_13 = 0x82,
    HTU21D_RES_RH_11_TEMP_11 = 0x83,
}htu_resolution_t;

/*******************************************
 * HTU21D Public structs
 ******************************************/



/*******************************************
 * HTU21D public functions
 ******************************************/
sensor_err_code_t htu21d_drv_begin(nrf_drv_twi_t *p_twi,
        htu_resolution_t resolution,
        htu_resolution_t **p_p_res,
        sensor_evt_callback_t (* htu21d_event_cb)(sensor_evt_t * event_data));

sensor_err_code_t htu21d_drv_convert_data();

htu21d_data_t htu21d_get_last_conversion_data();

sensor_err_code_t htu21d_drv_set_resolution(htu_resolution_t resolution);

#endif /* SRC_LIBS_SENSING_HTU21D_DRV_HTU21D_DRV_H_ */
