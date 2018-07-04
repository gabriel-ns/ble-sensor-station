/**
 * @file ble_thss.c
 *
 * @brief Implementation of the Temperature and Humidity Sensor Service functionalities.
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

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "app_config.h"
#include "app_error.h"

#include "nrf_drv_twi.h"
#include "sensor_public_interface.h"
#include "bmp180_drv.h"
#include "htu21d_drv.h"
#include "tsl2561_drv.h"

#include "sensor_controller.h"

#include "ble_services_common.h"
#include "ble.h"
#include "ble_gatts.h"
#include "ble_thss.h"

/***********************************************
 * Private functions prototypes
 ***********************************************/

/**
 * @brief Function that handles the writing attempts to this service's characteristics.
 *
 * @param p_thss        Pointer to the application THSS Service structure.
 * @param p_ble_evt     Pointer to the event received from BLE stack.
 */
static void on_write(ble_thss_t *p_thss, ble_evt_t *p_ble_evt);

/**
 * @brief Handle read/write authorize request events from the SoftDevice.
 *
 * @param p_thss        Pointer to the application THSS Service structure.
 * @param p_ble_evt     Pointer to the event received from BLE stack.
 */
static uint32_t on_rw_authorize_req(ble_thss_t *p_thss, ble_evt_t *p_ble_evt);

/**
 * @brief Pushes the new sensor value to the BLE client
 *
 * @param p_thss        Pointer to the application THSS Service structure.
 * @param p_sensor_evt  Pointer to the event received from sensor controller.
 */
static void ble_thss_notify(ble_thss_t *p_thss, sensor_evt_t *p_sensor_evt);

/**
 * @brief Function that initializes the Sensing Interval Characteristic.
 *
 * @param p_thss    Pointer to the application THSS Service structure.
 */
static void ble_thss_sensing_interval_init(ble_thss_t *p_thss);

/**
 * @brief Function that initializes the Sensor Status Characteristic.
 *
 * @param p_thss    Pointer to the application THSS Service structure.
 */
static void ble_thss_sensor_status_init(ble_thss_t *p_thss);

/**
 * @brief Function that initializes the Sensor Resolution Characteristic.
 *
 * @param p_thss    Pointer to the application THSS Service structure.
 */
static void ble_thss_sensor_resolution_init(ble_thss_t *p_thss);

/**
 * @brief Function that initializes the Pressure Data Characteristic.
 *
 * @param p_thss    Pointer to the application THSS Service structure.
 */
static void ble_thss_humidity_data_init(ble_thss_t *p_thss);

/**
 * @brief Function that initializes the Temperature Data Characteristic.
 *
 * @param p_thss    Pointer to the application THSS Service structure.
 */
static void ble_thss_temperature_data_init(ble_thss_t *p_thss);

/***********************************************
 * Public functions implementation
 ***********************************************/
uint32_t ble_thss_on_ble_evt(ble_thss_t * p_thss, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
        {
            case BLE_GAP_EVT_CONNECTED:
                p_thss->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                break;
            case BLE_GAP_EVT_DISCONNECTED:
                p_thss->conn_handle = BLE_CONN_HANDLE_INVALID;
                break;
            case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
                on_rw_authorize_req(p_thss,p_ble_evt);
                break;
            default:
                // No implementation needed.
                break;
        }
        return NRF_SUCCESS;
}

void ble_thss_on_sensor_evt(ble_thss_t *p_thss, sensor_evt_t *p_sensor_evt)
{
    if(p_sensor_evt->sensor == SENSOR_HTU21D &&
            p_thss->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ble_thss_notify(p_thss, p_sensor_evt);
    }
}

void ble_thss_init(ble_thss_t * p_thss)
{
    uint32_t        err_code;
    ble_uuid_t      service_uuid;
    ble_uuid128_t   base_uuid = { BLE_SERVICES_BASE_UUID };


    service_uuid.uuid = BLE_THSS_SERVICE_UUID;

    sensor_ctrl_cfg_t *p_cfg_data = sensor_ctrl_get_cfg_ptr();
    sensor_ctrl_data_t *p_data = sensor_ctrl_get_data_ptr();

    p_thss->p_sensor_config = &p_cfg_data->htu_cfg;
    p_thss->p_sensor_data = &p_data->htu21d_data;

    /* Start the service with invalid handle */
    p_thss->conn_handle = BLE_CONN_HANDLE_INVALID;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
            &service_uuid,
            &p_thss->service_handle);
    APP_ERROR_CHECK(err_code);

    /* Add all the characteristics */
    ble_thss_sensing_interval_init(p_thss);
    ble_thss_sensor_status_init(p_thss);
    ble_thss_sensor_resolution_init(p_thss);
    ble_thss_temperature_data_init(p_thss);
    ble_thss_humidity_data_init(p_thss);
}


/***********************************************
 * Private functions implementation
 ***********************************************/
static void on_write(ble_thss_t *p_thss, ble_evt_t *p_ble_evt)
{
    ret_code_t err_code;
        ble_gatts_rw_authorize_reply_params_t reply;

        // Initialize reply.
        reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
        reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
        reply.params.write.update = 1;
        reply.params.write.offset = 0;
        reply.params.write.len = 0;
        reply.params.write.p_data = NULL;

        // Write to appropriate characteristic.
        ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.authorize_request.request.write;


        if(p_evt_write->handle == p_thss->sensing_interval_handle.value_handle)
        {
            uint32_t new_interval = *((uint32_t *) p_evt_write->data);

            uint32_t err_code;
            err_code = sensor_ctrl_set_interval(SENSOR_HTU21D, new_interval);

            if(err_code == SENSOR_SUCCESS)
            {
                reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
                reply.params.write.len = sizeof(p_thss->p_sensor_config->interval);
                reply.params.write.p_data = (uint8_t *) (&p_thss->p_sensor_config->interval);
            }
            else
            {
                reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
                reply.params.write.len = 0;
                reply.params.write.p_data = NULL;
            }
        }

        else if(p_evt_write->handle == p_thss->sensor_status_handle.value_handle)
        {
            sensor_state_t new_state = *((sensor_state_t *) p_evt_write->data);

            uint32_t err_code;
            err_code = sensor_ctrl_set_sensor_state(SENSOR_HTU21D, new_state);

            if(err_code == SENSOR_SUCCESS)
            {
                reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
                reply.params.write.len = sizeof(p_thss->p_sensor_config->state);
                reply.params.write.p_data = (uint8_t *) (&p_thss->p_sensor_config->state);
            }
            else
            {
                reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
                reply.params.write.len = 0;
                reply.params.write.p_data = NULL;
            }
        }
        else if(p_evt_write->handle == p_thss->sensor_resolution_handle.value_handle)
        {
            htu_resolution_t res = *((htu_resolution_t *) p_evt_write->data);

            uint32_t err_code;
            err_code = sensor_ctrl_set_htu_res(res);

            if(err_code == SENSOR_SUCCESS)
            {
                reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
                reply.params.write.len = sizeof(*p_thss->p_sensor_config->p_res);
                reply.params.write.p_data = (uint8_t *) (p_thss->p_sensor_config->p_res);
            }
            else
            {
                reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
                reply.params.write.len = 0;
                reply.params.write.p_data = NULL;
            }
        }
        else
        {
            // Return without replying. This event is not relevant for this service.
            return;
        }

        // Send write reply.
        if (p_thss->conn_handle != BLE_CONN_HANDLE_INVALID )
        {
            err_code = sd_ble_gatts_rw_authorize_reply(p_thss->conn_handle, &reply);
            if (err_code != NRF_SUCCESS)
            {
                APP_ERROR_CHECK(err_code);
            }
        }
}

/**
 * @brief Handle read/write authorize request events from the SoftDevice.
 *
 * @param[in] p_stcs    DLU Service structure.
 * @param[in] p_ble_evt  Pointer to the event received from BLE stack.
 */
static uint32_t on_rw_authorize_req(ble_thss_t *p_thss, ble_evt_t *p_ble_evt)
{
    if (p_ble_evt->evt.gatts_evt.params.authorize_request.type ==
             BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        on_write(p_thss, p_ble_evt);
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}

static void ble_thss_notify(ble_thss_t *p_thss, sensor_evt_t *p_sensor_evt)
{
    uint16_t               len = sizeof(p_thss->p_sensor_data->temp);
    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));

    /* Notify the temperature data */
    hvx_params.handle = p_thss->temperature_data_handle.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &len;
    hvx_params.p_data = (uint8_t*)&p_thss->p_sensor_data->hum;

    sd_ble_gatts_hvx(p_thss->conn_handle, &hvx_params);

    /* Notify the pressure data */
    len = sizeof(p_thss->p_sensor_data->hum);

    hvx_params.handle = p_thss->humidity_data_handle.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &len;
    hvx_params.p_data = (uint8_t*)&p_thss->p_sensor_data->hum;
    sd_ble_gatts_hvx(p_thss->conn_handle, &hvx_params);
}

static void ble_thss_sensing_interval_init(ble_thss_t *p_thss)
{
    uint32_t            err_code;
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = { BLE_SERVICES_BASE_UUID };

    char_uuid.uuid = BLE_THSS_SENSING_INTERVAL_UUID;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);

    //Add read/write properties
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.char_props.write = 1;

    //Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc = BLE_GATTS_VLOC_USER;

    //Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.wr_auth = 1;

    //Configure the characteristic value attribute
    ble_gatts_attr_t attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;

    //Set characteristic length in number of bytes and value
    attr_char_value.max_len     = sizeof(p_thss->p_sensor_config->interval);
    attr_char_value.init_len    = sizeof(p_thss->p_sensor_config->interval);
    attr_char_value.p_value     = (uint8_t *) &(p_thss->p_sensor_config->interval);

    //Add characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_thss->service_handle,
            &char_md,
            &attr_char_value,
            &p_thss->sensing_interval_handle);

    APP_ERROR_CHECK(err_code);
}

static void ble_thss_sensor_status_init(ble_thss_t *p_thss)
{
    uint32_t            err_code;
        ble_uuid_t          char_uuid;
        ble_uuid128_t       base_uuid = { BLE_SERVICES_BASE_UUID };

        char_uuid.uuid = BLE_THSS_SENSOR_STATUS_UUID;

        err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
        APP_ERROR_CHECK(err_code);

        //Add read/write properties
        ble_gatts_char_md_t char_md;
        memset(&char_md, 0, sizeof(char_md));

        char_md.char_props.read  = 1;
        char_md.char_props.write = 1;

        //Configure the attribute metadata
        ble_gatts_attr_md_t attr_md;
        memset(&attr_md, 0, sizeof(attr_md));

        attr_md.vloc = BLE_GATTS_VLOC_USER;

        //Set read/write security levels to our characteristic
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
        attr_md.wr_auth = 1;

        //Configure the characteristic value attribute
        ble_gatts_attr_t attr_char_value;
        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid      = &char_uuid;
        attr_char_value.p_attr_md   = &attr_md;

        //Set characteristic length in number of bytes and value
        attr_char_value.max_len     = sizeof(p_thss->p_sensor_config->state);
        attr_char_value.init_len    = sizeof(p_thss->p_sensor_config->state);
        attr_char_value.p_value     = (uint8_t *) &(p_thss->p_sensor_config->state);

        //Add characteristic to the service
        err_code = sd_ble_gatts_characteristic_add(p_thss->service_handle,
                &char_md,
                &attr_char_value,
                &p_thss->sensor_status_handle);

        APP_ERROR_CHECK(err_code);
}

static void ble_thss_sensor_resolution_init(ble_thss_t *p_thss)
{
    uint32_t            err_code;
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = { BLE_SERVICES_BASE_UUID };

    char_uuid.uuid = BLE_THSS_SENSOR_RESOLUTION_UUID;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);

    //Add read/write properties
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.char_props.write = 1;

    //Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc = BLE_GATTS_VLOC_USER;

    //Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.wr_auth = 1;

    //Configure the characteristic value attribute
    ble_gatts_attr_t attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;

    //Set characteristic length in number of bytes and value
    attr_char_value.max_len     = sizeof(*p_thss->p_sensor_config->p_res);
    attr_char_value.init_len    = sizeof(*p_thss->p_sensor_config->p_res);
    attr_char_value.p_value     = (uint8_t *) (p_thss->p_sensor_config->p_res);

    //Add characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_thss->service_handle,
            &char_md,
            &attr_char_value,
            &p_thss->sensor_resolution_handle);

    APP_ERROR_CHECK(err_code);
}

static void ble_thss_temperature_data_init(ble_thss_t *p_thss)
{
    uint32_t            err_code;
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = { BLE_SERVICES_BASE_UUID };

    char_uuid.uuid = BLE_THSS_TEMPERATURE_DATA_UUID;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);

    //Add read/write properties
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;

    //Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc = BLE_GATTS_VLOC_USER;

    //Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;

    //Configure the characteristic value attribute
    ble_gatts_attr_t attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;

    //Set characteristic length in number of bytes and value
    attr_char_value.max_len     = sizeof(p_thss->p_sensor_data->temp);
    attr_char_value.init_len    = sizeof(p_thss->p_sensor_data->temp);
    attr_char_value.p_value     = (uint8_t *) &(p_thss->p_sensor_data->temp);

    //Add characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_thss->service_handle,
            &char_md,
            &attr_char_value,
            &p_thss->temperature_data_handle);

    APP_ERROR_CHECK(err_code);
}

static void ble_thss_humidity_data_init(ble_thss_t *p_thss)
{
    uint32_t            err_code;
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = { BLE_SERVICES_BASE_UUID };

    char_uuid.uuid = BLE_THSS_HUMIDITY_DATA_UUID;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);

    //Add read/write properties
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;

    //Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc = BLE_GATTS_VLOC_USER;

    //Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;

    //Configure the characteristic value attribute
    ble_gatts_attr_t attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;

    //Set characteristic length in number of bytes and value
    attr_char_value.max_len     = sizeof(p_thss->p_sensor_data->hum);
    attr_char_value.init_len    = sizeof(p_thss->p_sensor_data->hum);
    attr_char_value.p_value     = (uint8_t *) &(p_thss->p_sensor_data->hum);

    //Add characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_thss->service_handle,
            &char_md,
            &attr_char_value,
            &p_thss->humidity_data_handle);

    APP_ERROR_CHECK(err_code);
}
/**
 * @}
 */
 
