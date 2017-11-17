/*
 * sensor_controller.h
 *
 *  Created on: 16 de nov de 2017
 *      Author: Gabriel
 */

#ifndef SRC_LIBS_SENSING_SENSOR_CONTROLLER_H_
#define SRC_LIBS_SENSING_SENSOR_CONTROLLER_H_

typedef struct sensor_data
{
        bmp180_data_t   bmp180_data;
        htu21d_data_t   htu21d_data;
        tsl2561_data_t  tsl2561_data;
}sensor_controller_data_t;

typedef struct bmp180_sc_config
{
        bmp180_pwr_mode_t   *p_pwr_mode;
        uint32_t            sampling_interval;
        sensor_state_t      state;
}bmp180_sc_config_t;

typedef struct htu21d_sc_config
{
        htu21d_resolution_t *p_res;
        uint32_t            sampling_interval;
        sensor_state_t      state;
}htu21d_sc_config_t;

typedef struct tsl2561_sc_config
{
        tsl2561_config_t    *p_config;
        uint32_t            sampling_interval;
        sensor_state_t      state;
}tsl2561_sc_config_t;

typedef struct sensor_controller_cfg_data
{
        bmp180_sc_config_t  bmp_cfg;
        htu21d_sc_config_t  htu_cfg;
        tsl2561_sc_config_t tsl_cfg;
}sensor_controller_cfg_data_t;

void sensor_controller_init();

sensor_controller_data_t * sensor_controller_get_sensor_data_pointer();

sensor_controller_cfg_data_t * sensor_controller_get_config_pointer();

sensor_error_code_t sensor_controller_set_tsl_gain(tsl2561_gain_t gain);

sensor_error_code_t sensor_controller_set_tsl_int_time(tsl2561_integration_time_t int_time);

sensor_error_code_t sensor_controller_set_htu_res(htu21d_resolution_t res);

sensor_error_code_t sensor_controller_set_bmp_pwr_mode(bmp180_pwr_mode_t pwr_mode);

sensor_error_code_t sensor_controller_set_sensor_sampling_interval(sensor_type_t sensor, uint32_t interval);

sensor_error_code_t sensor_controller_set_sensor_state(sensor_type_t sensor, sensor_state_t state);





#endif /* SRC_LIBS_SENSING_SENSOR_CONTROLLER_H_ */
