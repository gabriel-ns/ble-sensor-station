/**
 * \file sensor_manager.h
 * \brief   Describes the interface with the sensor manager module.
 *
 *          The sensor manager module is responsible for handling all the
 *          application sensors, setting it's configurations, holding
 *          the data and for periodically requiring new sensor readings.
 *
 */
/*
 *  UNIVERSIDADE FEDERAL DO ABC
 *  Aluno: Gabriel Nascimento dos Santos
 *  Curso: Engenharia de Instrumentação, Automação e Robótica
 *
 *  Projeto: DESENVOLVIMENTO DE REDE DE SENSORES BLUETOOTH LOW ENERGY CONECTADOS À NUVEM
 */

#ifndef SENSOR_MANAGER_H__
#define SENSOR_MANAGER_H__
#if 0
#include "app_config.h"
#include "nrf_drv_twi.h"
#include "nrf5-bmp180-drv.h"
#include "nrf5-htu21d-drv.h"
#include "nrf5-tsl2561-drv.h"

typedef enum sensor_state
{
    SENSOR_STATE_UNINIT = 0x00,
    SENSOR_STATE_ACTIVE,
    SENSOR_STATE_INACTIVE,
    SENSOR_STATE_ERROR,
    SENSOR_INVALID = 0xFF
}sensor_state_t;

typedef enum sensor
{
    SENSOR_BMP180 = 0x00,
    SENSOR_HTU21D,
    SENSOR_TSL2561
}sensor_t;

typedef struct sensor_manager_data_buff
{
        int32_t     pressure;
        int16_t     temperature;
        uint16_t    humidity;
        int32_t     luminosity;
}sensor_manager_data_buff_t;

typedef struct sensor_manager_cfg_data
{
        bmp180_pwr_mode_t           bmp180_pwr;
        htu21d_resolution_t         htu21d_res;
        tsl2561_integration_time_t  tsl2561_int_time;
        tsl2561_gain_t              tsl2561_gain;
        tsl2561_power_t             tsl2561_pwr;
        uint32_t                    lum_sampling_interval;
        uint32_t                    th_sampling_interval;
        uint32_t                    pres_sampling_interval;
        sensor_state_t              pres_state;
        sensor_state_t              th_state;
        sensor_state_t              lum_state;
}sensor_manager_cfg_data_t;

void sensor_manager_init();

/* Sensor configuration functions */
ret_code_t sensor_manager_set_pres_cfg(bmp180_pwr_mode_t pwr_mode);

ret_code_t sensor_manager_set_th_cfg(htu21d_resolution_t res);

ret_code_t sensor_manager_set_lum_int_time(tsl2561_integration_time_t int_time);

ret_code_t sensor_manager_set_lum_set_gain(tsl2561_gain_t gain);

ret_code_t sensor_manager_set_lum_set_pwr(tsl2561_power_t pwr);

ret_code_t sensor_manager_set_sensor_sampling_interval(sensor_t sensor, uint32_t time_ms);

ret_code_t sensor_manager_set_sensor_state(sensor_t sensor, sensor_state_t state);

sensor_state_t sensor_manager_get_sensor_state(sensor_t sensor);

sensor_manager_cfg_data_t sensor_manager_get_curr_cfg();

/* Sensor data handling functions */
sensor_manager_data_buff_t sensor_manager_get_data_buff();

int16_t sensor_manager_last_get_temperature();

uint16_t sensor_manager_last_get_humidity();

int32_t sensor_manager_last_get_pressure();

int32_t sensor_manager_last_get_luminosity();

#endif
#endif /** SENSOR_MANAGER_H__ */
