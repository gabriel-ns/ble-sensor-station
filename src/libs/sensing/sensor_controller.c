/*
 * sensor_controller.c
 *
 *  Created on: 17 de nov de 2017
 *      Author: Gabriel
 */

#include <stddef.h>
#include <stdint.h>

#include "nrf_drv_twi.h"
#include "sensor_public_interface.h"

#include "bmp180_drv.h"
#include "htu21d_drv.h"
#include "tsl2561_drv.h"

#include "sensor_controller.h"

/***********************************************
 * Public functions implementation
 ***********************************************/
void sensor_controller_init()
{

}

sensor_data_t * sensor_controller_get_sensor_data_pointer()
{
    return NULL;
}

sensor_controller_cfg_data_t * sensor_controller_get_config_pointer()
{
    return NULL;
}

sensor_error_code_t sensor_controller_set_tsl_gain(tsl2561_gain_t gain)
{
    return SENSOR_SUCCESS;
}

sensor_error_code_t sensor_controller_set_tsl_int_time(tsl2561_integration_time_t int_time)
{
    return SENSOR_SUCCESS;
}

sensor_error_code_t sensor_controller_set_htu_res(htu21d_resolution_t res)
{
    return SENSOR_SUCCESS;
}

sensor_error_code_t sensor_controller_set_bmp_pwr_mode(bmp180_pwr_mode_t pwr_mode)
{
    return SENSOR_SUCCESS;
}

sensor_error_code_t sensor_controller_set_sensor_sampling_interval(sensor_type_t sensor, uint32_t interval)
{
    return SENSOR_SUCCESS;
}

sensor_error_code_t sensor_controller_set_sensor_state(sensor_type_t sensor, sensor_state_t state)
{
    return SENSOR_SUCCESS;
}
