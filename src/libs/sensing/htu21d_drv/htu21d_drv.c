/*
 * htu21d_drv.c
 *
 *  Created on: 17 de nov de 2017
 *      Author: Gabriel
 */
#include <stddef.h>
#include <stdint.h>

#include "nrf_drv_twi.h"

#include "sensor_public_interface.h"
#include "htu21d_drv.h"

/***********************************************
 * Private defines
 ***********************************************/

/***********************************************
 * Private macros
 ***********************************************/

/***********************************************
 * Private enums
 ***********************************************/

/***********************************************
 * Private structs
 ***********************************************/

/***********************************************
 * Private global variables
 ***********************************************/

/***********************************************
 * Private functions prototypes
 ***********************************************/

/***********************************************
 * Public functions implementation
 ***********************************************/
sensor_error_code_t htu21d_drv_begin(nrf_drv_twi_t *p_twi,
        htu21d_resolution_t resolution,
        htu21d_resolution_t **p_p_res,
        htu21d_event_cb_t (* htu21d_event_cb)(sensor_event_t * event_data))
{
    return SENSOR_SUCCESS;
}

sensor_error_code_t htu21d_drv_convert_data()
{
    return SENSOR_SUCCESS;
}

htu21d_data_t htu21d_get_last_conversion_data()
{
    htu21d_data_t data;
    return data;
}

sensor_error_code_t htu21d_drv_set_resolution(htu21d_resolution_t resolution)
{
    return SENSOR_SUCCESS;
}
