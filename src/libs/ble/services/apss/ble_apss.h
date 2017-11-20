/**
 * @file ble_apss.h
 *
 * @brief Public data about the Air Pressure Sensor Service.
 * @author Gabriel Nascimento dos Santos
 *
 * \n UNIVERSIDADE FEDERAL DO ABC
 * \n Curso: Engenharia de Instrumentação, Automação e Robótica
 * \n Projeto: Desenvolvimento de rede de sensores bluetooth conectados à nuvem
 * \n\n Desenvolvido durante as disciplinas de Trabalho de Graduação 1, 2 e 3 do curso.
 *
 * @addtogroup APSS
 * @{
 */
 
#ifndef _BLE_APSS_H_
#define _BLE_APSS_H_

/**
 * @brief UUID for Air Pressure Sensor Service.
 */
#define BLE_APSS_SERVICE_UUID               0xABC1

#define BLE_APSS_SENSING_INTERVAL_UUID      0x0A00
#define BLE_APSS_SENSOR_STATUS_UUID         0x0A01
#define BLE_APSS_SENSOR_RESOLUTION_UUID     0x0A02
#define BLE_APSS_PRESSURE_DATA_UUID         0x0A03
#define BLE_APSS_TEMPERATURE_DATA_UUID      0x0A04


/**
 * @brief Type for Air Pressure Sensor Service information.
 */
typedef struct ble_apss ble_apss_t;


//todo Explicar o que são handles no texto escrito.

/**
 * @brief Structure definition for Air Pressure Sensor Service.
 */
struct ble_apss
{
        uint16_t                    conn_handle;              /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
        uint16_t                    service_handle;           /**< Handle of Air Pressure Sensor Service (as provided by the BLE stack). */
        ble_gatts_char_handles_t    sensing_interval_handle;  /**< Handle of Sensing Interval Characteristic. */
        ble_gatts_char_handles_t    sensor_status_handle;     /**< Handle of Sensor Status Characteristic. */
        ble_gatts_char_handles_t    sensor_resolution_handle; /**< Handle of Sensor Resolution Characteristic. */
        ble_gatts_char_handles_t    pressure_data_handle;     /**< Handle of Pressure Data Characteristic. */
        ble_gatts_char_handles_t    temperature_data_handle;  /**< Handle of Temperature Data Characteristic. */
        bmp180_sc_config_t          *p_sensor_config;         /**< Pointer to BMP180 sensor configuration. */
        bmp180_data_t               *p_sensor_data;           /**< Pointer to BMP180 sensor data */
};

/**
 * @brief Function called when receiving a BLE event.
 * @param p_apss    Pointer to Service Structure
 * @param p_ble_evt Pointer to BLE event that caused the call * @return
 */
uint32_t ble_apss_on_ble_evt(ble_apss_t * p_apss, ble_evt_t * p_ble_evt);

/**
 * @brief Function called when receiving a Sensor Event
 * @param p_apss        Pointer to Service Structure
 * @param p_sensor_evt  Pointer to the sensor event
 */
void ble_apss_on_sensor_evt(ble_apss_t * p_apss, sensor_event_t *p_sensor_evt);

/**
 * @brief Function for initializing Air Pressure Sensor Service.
 *
 * @param[in]   p_apss       Pointer to Service structure.
 */
void ble_apss_init(ble_apss_t * p_apss);


#endif /* _BLE_APSS_H_ */
/**
 * @}
 */
