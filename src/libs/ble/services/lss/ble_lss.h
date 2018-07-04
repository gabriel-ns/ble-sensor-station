/**
 * @file ble_lss.h
 *
 * @brief Public data about the Luminosity Sensor Service.
 * @author Gabriel Nascimento dos Santos
 *
 * \n UNIVERSIDADE FEDERAL DO ABC
 * \n Curso: Engenharia de Instrumentação, Automação e Robótica
 * \n Projeto: Desenvolvimento de rede de sensores bluetooth conectados à nuvem
 * \n\n Desenvolvido durante as disciplinas de Trabalho de Graduação 1, 2 e 3 do curso.
 *
 * @addtogroup LSS
 * @{
 */
 
#ifndef _BLE_LSS_H_
#define _BLE_LSS_H_

/**
 * @brief UUID for Temperature and Humidity Sensor Service.
 */
#define BLE_LSS_SERVICE_UUID                    0xABC3

#define BLE_LSS_SENSING_INTERVAL_UUID           0x0C00
#define BLE_LSS_SENSOR_STATUS_UUID              0x0C01
#define BLE_LSS_SENSOR_INTEGRATION_TIME_UUID    0x0C02
#define BLE_LSS_SENSOR_GAIN_UUID                0x0C03
#define BLE_LSS_VISIBLE_LUX_DATA_UUID           0x0C04
#define BLE_LSS_INFRARED_LUX_DATA_UUID          0x0C05

/**
 * @brief Type for Temperature and Humidity Sensor Service.
 */
typedef struct ble_lss ble_lss_t;

/**
 * @brief Structure definition for Temperature and Humidity Sensor Service.
 */
struct ble_lss
{
        uint16_t                    conn_handle;              /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
        uint16_t                    service_handle;           /**< Handle of Air Pressure Sensor Service (as provided by the BLE stack). */
        ble_gatts_char_handles_t    sensing_interval_handle;  /**< Handle of Sensing Interval Characteristic. */
        ble_gatts_char_handles_t    sensor_status_handle;     /**< Handle of Sensor Status Characteristic. */
        ble_gatts_char_handles_t    sensor_int_time_handle;   /**< Handle of Sensor Integration Time Characteristic. */
        ble_gatts_char_handles_t    sensor_gain_handle;       /**< Handle of Sensor Gain Characteristic. */
        ble_gatts_char_handles_t    visible_data_handle;      /**< Handle of Visible Lux Data Characteristic. */
        ble_gatts_char_handles_t    infrared_data_handle;     /**< Handle of Infrared Lux Data Characteristic. */
        tsl2561_sc_config_t         *p_sensor_cfg;            /**< Pointer to TSL2561 sensor configuration. */
        tsl2561_data_t              *p_sensor_data;           /**< Pointer to TSL2561 sensor data */
};

/**
 * @brief Callback called when receiving a BLE event.
 * @param p_lss    Pointer to Service Structure
 * @param p_ble_evt Pointer to BLE event that caused the call
 */
uint32_t ble_lss_on_ble_evt(ble_lss_t * p_lss, ble_evt_t * p_ble_evt);

/**
 * @brief Function called when receiving a Sensor Event
 * @param p_lss         Pointer to Service Structure
 * @param p_sensor_evt  Pointer to the sensor event
 */
void ble_lss_on_sensor_evt(ble_lss_t * p_lss, sensor_evt_t *p_sensor_evt);

/**
 * @brief Function for initializing Luminosity Sensor Service.
 *
 * @param[in]   p_lss       Pointer to Service structure.
 */
void ble_lss_init(ble_lss_t * p_lss);

#endif /* _BLE_LSS_H_ */
/**
 * @}
 */
