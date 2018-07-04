/**
 * @file ble_manager.c
 * @brief The Bluetooth Manager module starts the BLE Stack, the Bluetooth Services and the Bluetooth Advertising.
 * @author Gabriel Nascimento dos Santos
 *
 * \n UNIVERSIDADE FEDERAL DO ABC
 * \n Curso: Engenharia de Instrumentação, Automação e Robótica
 * \n Projeto: Desenvolvimento de rede de sensores bluetooth conectados à nuvem
 * \n\n Desenvolvido durante as disciplinas de Trabalho de Graduação 1, 2 e 3 do curso.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ble_manager.h"
#include "softdevice_handler.h"
#include "app_config.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble_conn_params.h"
#include "nrf51.h"
#include "crc32.h"

#include "nrf_drv_twi.h"
#include "sensor_public_interface.h"
#include "bmp180_drv.h"
#include "htu21d_drv.h"
#include "tsl2561_drv.h"
#include "sensor_controller.h"

#include "ble_thss.h"
#include "ble_apss.h"
#include "ble_lss.h"
#include "ble_rpcs.h"

#include "ble_adv_controller.h"


static void gap_params_init(void);
static void ble_stack_init(void);
static void conn_params_init(void);
static void ble_mac_addr_get(uint8_t *device_addr);

static void sensor_evt_dispatch(sensor_evt_t *p_sensor_evt);

static ble_thss_t m_ble_thss; // THSS Service Instance
static ble_apss_t m_ble_apss; // APSS Service Instance
static ble_lss_t m_ble_lss;   // LSS Service Instance
static ble_rpcs_t m_ble_rpcs; // RPCS Service Instance

void ble_manager_init()
{
	// Initialize the BLE
    ble_stack_init();
    gap_params_init();
    conn_params_init();

    // Initialize the Advertising
    advertising_init();

    // Initializes the GATT Services
    ble_apss_init(&m_ble_apss);
    ble_thss_init(&m_ble_thss);
    ble_lss_init(&m_ble_lss);
    ble_rpcs_init(&m_ble_rpcs);

    // Subscribe to sensor events
    sensor_ctrl_evt_sub(sensor_evt_dispatch);
}

static void sensor_evt_dispatch(sensor_evt_t *p_sensor_evt)
{
	adv_on_sensor_event(p_sensor_evt);
    ble_apss_on_sensor_evt(&m_ble_apss, p_sensor_evt);
    ble_thss_on_sensor_evt(&m_ble_thss, p_sensor_evt);
    ble_lss_on_sensor_evt(&m_ble_lss, p_sensor_evt);

}

static void gap_params_init()
{
    ret_code_t err_code;
    ble_gap_addr_t device_addr;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    uint8_t device_name[] = "UFABC001";

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
            device_name,
            strlen((const char *)device_name));
    APP_ERROR_CHECK(err_code);

    ble_mac_addr_get(device_addr.addr);
    device_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;

    err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &device_addr);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(DEFAULT_ADVERTISING_TX_POWER);
    APP_ERROR_CHECK(err_code);
}

static void conn_params_init(void)
{
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);

    ble_apss_on_ble_evt(&m_ble_apss, p_ble_evt);
    ble_thss_on_ble_evt(&m_ble_thss, p_ble_evt);
    ble_lss_on_ble_evt(&m_ble_lss, p_ble_evt);
    ble_rpcs_on_ble_evt(&m_ble_rpcs, p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            advertising_stop();
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            advertising_start();
            break;

        default:
            break;
    }
}

/**
 * @brief Function for dispatching system events from the SoftDevice.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] evt_id  System event id.
 */
static void sys_evt_dispatch(uint32_t evt_id)
{

}

/**
 * @brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    //Subscribe for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

static void ble_mac_addr_get(uint8_t *device_addr)
{
    uint32_t device_id[2];

    // Get unique device ID
    device_id[0] = NRF_FICR->DEVICEID[0];
    device_id[1] = NRF_FICR->DEVICEID[1];

    // First 4 bytes of chip ID
    device_addr[0] = (device_id[0] >> 0 * 8) & 0xFF;
    device_addr[1] = (device_id[0] >> 1 * 8) & 0xFF;
    device_addr[2] = (device_id[0] >> 2 * 8) & 0xFF;
    device_addr[3] = (device_id[0] >> 3 * 8) & 0xFF;
    // 5th and 6th bytes of chip ID
    device_addr[4] = (device_id[1] >> 0 * 8) & 0xFF;
    device_addr[5] = (device_id[1] >> 1 * 8) & 0xFF;
    // two most significant bits of last byte must be 1's
    // for Random Static Address
    device_addr[5] |= 0xC0;
}


