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
typedef struct sensor_data sensor_ctrl_data_t;

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
typedef struct sensor_controller_cfg_data sensor_ctrl_cfg_t;

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
        bmp_pwr_mode_t   *p_pwr_mode;        /**< Pointer to sensor power mode. */
        uint32_t            interval;  /**< Sampling interval for the BMP180 sensor. */
        sensor_state_t      state;              /**< The current state for BMP180 sensor. See @ref sensor_state_t. */
};

/**
 * @brief Structure that stores the configuration data from HTU21D sensor.
 */
struct htu21d_sc_config
{
        htu_resolution_t *p_res;
        uint32_t            interval;
        sensor_state_t      state;
};

/**
 * @brief Structure that stores the configuration data from TSL2561 sensor.
 */
struct tsl2561_sc_config
{
        tsl2561_config_t    *p_config;
        uint32_t            interval;
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
/** @brief Initializes the sensor controller */
void sensor_ctrl_init();

/** @brief Subscribe to sensor events */
void sensor_ctrl_evt_sub(sensor_evt_callback_t p_evt_cb);

/** @brief Get the pointer to the structure that stores
 *         the current sensor data */
sensor_ctrl_data_t * sensor_ctrl_get_data_ptr();

/** @brief Get the pointer to the current sensors configuration */
sensor_ctrl_cfg_t * sensor_ctrl_get_cfg_ptr();

/** @brief Interface to set the TSL2561 sensor gain */
sensor_err_code_t sensor_ctrl_set_tsl_gain(tsl_gain_t gain);

/** @brief Interface to set the TSL2561 integration time */
sensor_err_code_t sensor_ctrl_set_tsl_int_time(tsl_int_time_t int_time);

/** @brief Interface to set the HTU21D sensor resolution */
sensor_err_code_t sensor_ctrl_set_htu_res(htu_resolution_t res);

/** @brief Interface to set the BMP180 sensor power mode */
sensor_err_code_t sensor_ctrl_set_bmp_pwr_mode(bmp_pwr_mode_t pwr_mode);

/** @brief Interface to set the sampling interval for a sensor */
sensor_err_code_t sensor_ctrl_set_interval(sensor_type_t sensor,
		                                   uint32_t interval);

/** @brief Interface to set the sensor state  */
sensor_err_code_t sensor_ctrl_set_sensor_state(sensor_type_t sensor,
											   sensor_state_t state);

#endif /* SRC_LIBS_SENSING_SENSOR_CONTROLLER_H_ */

/**
 * @}
 */
