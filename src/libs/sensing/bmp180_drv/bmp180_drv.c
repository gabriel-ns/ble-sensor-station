/*
 * bmp180_drv.c
 *
 *  Created on: 17 de nov de 2017
 *      Author: Gabriel
 */

#include <stddef.h>
#include <stdint.h>

#include "nrf_drv_twi.h"

#include "sensor_public_interface.h"
#include "bmp180_drv.h"

/***********************************************
 * Public functions implementation
 ***********************************************/
sensor_error_code_t bmp180_drv_begin(nrf_drv_twi_t *p_twi,
        bmp180_pwr_mode_t pwr_mode,
        bmp180_pwr_mode_t ** p_p_pwr_mode,
        bmp180_event_cb_t (* bmp180_event_cb)(sensor_event_t * event_data))
{
    return SENSOR_SUCCESS;
}

sensor_error_code_t bmp180_drv_convert_data()
{
    return SENSOR_SUCCESS;
}

bmp180_data_t bmp180_drv_get_last_converson()
{
    bmp180_data_t data;
    return data;
}

sensor_error_code_t bmp180_drv_set_pwr_mode(bmp180_pwr_mode_t pwr_mode)
{
    return SENSOR_SUCCESS;
}
