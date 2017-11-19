/**
 * @file sensor_controller.h
 * @brief The sensor controller module is responsible for requesting data and setting configurations to sensors
 *
 * @author Gabriel Nascimento dos Santos
 *
 * The sensor controller module controls three sensors: BMP180 pressure sensor,
 * TSL2561 luminosity sensor and HTU21D temperature and humidity sensor, holding sensor reading data and
 * the sensor drivers internal configuration.
 * \n In addition, the configuration is extended by adding the a sensor current state and a timer to schedule
 * the sensor readings.
 *
 * \n UNIVERSIDADE FEDERAL DO ABC
 * \n Curso: Engenharia de Instrumentação, Automação e Robótica
 * \n Projeto: Desenvolvimento de rede de sensores bluetooth conectados à nuvem
 * \n\n Desenvolvido durante as disciplinas de Trabalho de Graduação 1, 2 e 3 do curso.
 *
 * @addtogroup SENSING
 * @{
 */

#ifndef SRC_LIBS_SENSING_SENSOR_CONTROLLER_H_
#define SRC_LIBS_SENSING_SENSOR_CONTROLLER_H_

/*****************************************************
 * Public typedefs
 ****************************************************/

/**
 * @brief Type that stores the reading data from all sensors.
 */
typedef struct sensor_data sensor_controller_data_t;

/**
 * @brief Type that stores the configuration data for BMP180 sensor.
 */
typedef struct bmp180_sc_config bmp180_sc_config_t;

/**
 * @brief Type that stores the configuration data for HTU21D sensor.
 */
typedef struct htu21d_sc_config htu21d_sc_config_t;

/**
 * @brief Type that stores the configuration data for TSL2561 sensor.
 */
typedef struct tsl2561_sc_config tsl2561_sc_config_t;

/**
 * @brief Type that stores the configuration data for all used sensor.
 */
typedef struct sensor_controller_cfg_data sensor_controller_cfg_data_t;

/*****************************************************
 * Public structs definitions
 ****************************************************/
/**
 * @brief Structure that stores the reading data from all sensors.
 */
struct sensor_data
{
        bmp180_data_t   bmp180_data;    /**< Holds the read data from bmp180 sensor. See @ref bmp180_data_t. */
        htu21d_data_t   htu21d_data;    /**< Holds the read data from HTU21D sensor. See @ref htu21d_data_t. */
        tsl2561_data_t  tsl2561_data;   /**< Holds the read data from TSL2561 sensor. See @ref tsl2561_data_t. */
};

/**
 * @brief Structure that stores the configuration data from BMP180 sensor.
 */
struct bmp180_sc_config
{
        bmp180_pwr_mode_t   *p_pwr_mode;        /**< Pointer to sensor power mode. */
        uint32_t            sampling_interval;  /**< Sampling interval for the BMP180 sensor. */
        sensor_state_t      state;              /**< The current state for BMP180 sensor. See @ref sensor_state_t. */
};

/**
 * @brief Structure that stores the configuration data from HTU21D sensor.
 */
struct htu21d_sc_config
{
        htu21d_resolution_t *p_res;
        uint32_t            sampling_interval;
        sensor_state_t      state;
};

/**
 * @brief Structure that stores the configuration data from TSL2561 sensor.
 */
struct tsl2561_sc_config
{
        tsl2561_config_t    *p_config;
        uint32_t            sampling_interval;
        sensor_state_t      state;
};

/**
 * @brief Structure that stores the configuration data from all sensors.
 */
struct sensor_controller_cfg_data
{
        bmp180_sc_config_t  bmp_cfg;
        htu21d_sc_config_t  htu_cfg;
        tsl2561_sc_config_t tsl_cfg;
};

/*****************************************************
 * Public functions
 ****************************************************/
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

/**
 * @}
 */
