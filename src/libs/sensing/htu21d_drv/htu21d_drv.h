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
}htu21d_resolution_t;

/*******************************************
 * HTU21D Public structs
 ******************************************/
typedef struct htu21d_data
{
        uint16_t humidity;
        int16_t temperature;
}htu21d_data_t;

/*******************************************
 * HTU21D callbacks
 ******************************************/
typedef void (*htu21d_event_cb_t)(sensor_event_t *event_data);

/*******************************************
 * HTU21D public functions
 ******************************************/
sensor_error_code_t htu21d_drv_begin(nrf_drv_twi_t *p_twi,
        htu21d_resolution_t resolution,
        htu21d_resolution_t **p_p_res,
        htu21d_event_cb_t (* htu21d_event_cb)(sensor_event_t * event_data));

sensor_error_code_t htu21d_drv_convert_data();

htu21d_data_t htu21d_get_last_conversion_data();

sensor_error_code_t htu21d_drv_set_resolution(htu21d_resolution_t resolution);

#endif /* SRC_LIBS_SENSING_HTU21D_DRV_HTU21D_DRV_H_ */
