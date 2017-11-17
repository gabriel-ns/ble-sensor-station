/*
 * tsl2561_drv.c
 *
 *  Created on: 17 de nov de 2017
 *      Author: Gabriel
 */
#include <stddef.h>
#include <stdint.h>

#include "nrf_drv_twi.h"

#include "sensor_public_interface.h"
#include "tsl2561_drv.h"

/***********************************************
 * Public functions implementation
 ***********************************************/
sensor_error_code_t tsl2561_drv_begin( nrf_drv_twi_t *p_twi,
        tsl2561_integration_time_t int_time,
        tsl2561_gain_t gain,
        tsl2561_config_t **config,
        tsl2561_event_cb_t (* tsl2561_event_cb)(sensor_event_t *event_data))
{
    return SENSOR_SUCCESS;
}

sensor_error_code_t tsl2561_drv_convert_data()
{
    return SENSOR_SUCCESS;
}

tsl2561_data_t tsl2561_drv_get_last_conversion_data()
{
    tsl2561_data_t data;
    return data;
}

sensor_error_code_t tsl2561_drv_set_integration_time(tsl2561_integration_time_t int_time)
{
    return SENSOR_SUCCESS;
}

sensor_error_code_t tsl2561_drv_set_gain(tsl2561_gain_t gain)
{
    return SENSOR_SUCCESS;
}

