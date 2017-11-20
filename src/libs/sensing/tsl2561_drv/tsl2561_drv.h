/*
 * tsl2561_drv.h
 *
 *  Created on: 16 de nov de 2017
 *      Author: Gabriel
 */

#ifndef SRC_LIBS_SENSING_TSL2561_DRV_TSL2561_DRV_H_
#define SRC_LIBS_SENSING_TSL2561_DRV_TSL2561_DRV_H_

/*******************************************
 * TSL2561 public enums
 ******************************************/
typedef enum tsl2561_integration_time
{
    TSL2561_INTEGRATION_TIME_13_7MS = 0x00,
    TSL2561_INTEGRATION_TIME_101MS = 0x01,
    TSL2561_INTEGRATION_TIME_402MS = 0x02,
}tsl2561_integration_time_t;

typedef enum tsl2561_gain
{
    TSL2561_GAIN_0 = 0,
    TSL2561_GAIN_16X = 1
}tsl2561_gain_t;

/*******************************************
 * TSL2561 public structs
 ******************************************/
typedef struct tsl2561_data
{
        uint32_t visible_lux;
        uint32_t infrared_lux;
}tsl2561_data_t;

typedef struct tsl2561_config
{
        tsl2561_integration_time_t int_time;
        tsl2561_gain_t gain;
}tsl2561_config_t;

/*******************************************
 * TSL2561 public functions
 ******************************************/
sensor_error_code_t tsl2561_drv_begin( nrf_drv_twi_t *p_twi,
        tsl2561_integration_time_t int_time,
        tsl2561_gain_t gain,
        tsl2561_config_t **config,
        sensor_event_callback_t (* tsl2561_event_cb)(sensor_event_t *event_data));

sensor_error_code_t tsl2561_drv_convert_data();

tsl2561_data_t tsl2561_drv_get_last_conversion_data();

sensor_error_code_t tsl2561_drv_set_integration_time(tsl2561_integration_time_t int_time);

sensor_error_code_t tsl2561_drv_set_gain(tsl2561_gain_t gain);

#endif /* SRC_LIBS_SENSING_TSL2561_DRV_TSL2561_DRV_H_ */
