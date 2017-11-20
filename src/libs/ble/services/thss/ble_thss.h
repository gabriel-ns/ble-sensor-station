/**
 * @file ble_thss.h
 *
 * @brief  Public data about the Temperature and Humidity Sensor Service.
 * @author Gabriel Nascimento dos Santos
 *
 * \n UNIVERSIDADE FEDERAL DO ABC
 * \n Curso: Engenharia de Instrumentação, Automação e Robótica
 * \n Projeto: Desenvolvimento de rede de sensores bluetooth conectados à nuvem
 * \n\n Desenvolvido durante as disciplinas de Trabalho de Graduação 1, 2 e 3 do curso.
 *
 * @addtogroup THSS
 * @{
 */
 
#ifndef _BLE_THSS_H_
#define _BLE_THSS_H_

/**
 * @brief UUID for Temperature and Humidity Sensor Service.
 */
#define BLE_THSS_SERVICE_UUID               0xABC2

#define BLE_THSS_SENSING_INTERVAL_UUID      0x0B00
#define BLE_THSS_SENSOR_STATUS_UUID         0x0B01
#define BLE_THSS_SENSOR_RESOLUTION_UUID     0x0B02
#define BLE_THSS_TEMPERATURE_DATA_UUID      0x0B03
#define BLE_THSS_HUMIDITY_DATA_UUID         0x0B04

/**
 * @brief Type for Temperature and Humidity Sensor Service.
 */
typedef struct ble_thss ble_thss_t;

/**
 * @brief Structure definition for Temperature and Humidity Sensor Service.
 */
struct ble_thss
{
        uint16_t                    conn_handle;              /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
        uint16_t                    service_handle;           /**< Handle of Air Pressure Sensor Service (as provided by the BLE stack). */
        ble_gatts_char_handles_t    sensing_interval_handle;  /**< Handle of Sensing Interval Characteristic. */
        ble_gatts_char_handles_t    sensor_status_handle;     /**< Handle of Sensor Status Characteristic. */
        ble_gatts_char_handles_t    sensor_resolution_handle; /**< Handle of Sensor Resolution Characteristic. */
        ble_gatts_char_handles_t    temperature_data_handle;  /**< Handle of Temperature Data Characteristic. */
        ble_gatts_char_handles_t    humidity_data_handle;     /**< Handle of Humidity Data Characteristic. */
        htu21d_sc_config_t          *p_sensor_config;         /**< Pointer to HTU21D sensor configuration. */
        htu21d_data_t               *p_sensor_data;           /**< Pointer to HTU21D sensor data */
};

/**
 * @brief Callback called when receiving a BLE event.
 * @param p_thss    Pointer to Service Structure
 * @param p_ble_evt Pointer to BLE event that caused the call * @return
 */
uint32_t ble_thss_on_ble_evt(ble_thss_t * p_thss, ble_evt_t * p_ble_evt);

/**
 * @brief Function called when receiving a Sensor Event
 * @param p_thss        Pointer to Service Structure
 * @param p_sensor_evt  Pointer to the sensor event
 */
void ble_thss_on_sensor_evt(ble_thss_t * p_thss, sensor_event_t *p_sensor_evt);

/**
 * @brief Function for initializing Temperature and Humidity Sensor Service.
 *
 * @param[in]   p_thss       Pointer to Service structure.
 */
void ble_thss_init(ble_thss_t * p_thss);

#endif /* _BLE_THSS_H_ */
/**
 * @}
 */
