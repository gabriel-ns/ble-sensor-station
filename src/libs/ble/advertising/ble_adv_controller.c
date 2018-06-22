/*
 * ble_adv_controller.c
 *
 *  Created on: Jan 20, 2018
 *      Author: gabriel
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app_config.h"
#include "app_error.h"

#include "ble_gap.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "sensor_public_interface.h"
#include "bmp180_drv.h"
#include "htu21d_drv.h"
#include "tsl2561_drv.h"
#include "sensor_controller.h"

#include "ble_adv_controller.h"

#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_adv_frame.h"

static ble_gap_adv_params_t m_adv_params;

static void update_adv_data();

ble_adv_data_t m_ble_adv_data =
{
		.pressure = 0,
		.temperature = 0,
		.humidity = 0,
		.vis_lux = 0,
		.ir_lux = 0
};

void adv_on_sensor_event(sensor_evt_t *p_sensor_evt)
{
	if(p_sensor_evt->evt_type !=
			SENSOR_EVT_DATA_READY) return;

	switch(p_sensor_evt->sensor)
	{
	case SENSOR_BMP180:
		m_ble_adv_data.pressure =
				p_sensor_evt->sensor_data.bmp_data.pressure;
		break;
	case SENSOR_HTU21D:
		m_ble_adv_data.temperature =
				p_sensor_evt->sensor_data.htu_data.temperature;
		m_ble_adv_data.humidity =
				p_sensor_evt->sensor_data.htu_data.humidity;
		break;
	case SENSOR_TSL2561:
		m_ble_adv_data.ir_lux =
				p_sensor_evt->sensor_data.tsl_data.ir_lux;
		m_ble_adv_data.vis_lux =
				p_sensor_evt->sensor_data.tsl_data.vis_lux;
		break;
	}

	update_adv_data();
}

void advertising_init()
{
    uint32_t err_code;

    /** Initialize advertising parameters (used when starting advertising). */
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    m_adv_params.type = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval = MSEC_TO_UNITS(200, UNIT_0_625_MS);
    m_adv_params.timeout = APP_CFG_CONNECTABLE_ADV_TIMEOUT;

	update_adv_data();
    advertising_start();
}

void advertising_start()
{
    uint32_t err_code = sd_ble_gap_adv_start(&m_adv_params);
    if (err_code != NRF_ERROR_BUSY && err_code != NRF_ERROR_CONN_COUNT)
    {
        APP_ERROR_CHECK(err_code);
    }
}
void advertising_stop()
{
    uint32_t err_code = sd_ble_gap_adv_stop();

    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
}

static void update_adv_data()
{
    uint32_t err_code;
	ble_advdata_t adv_data;
    ble_advdata_t scrsp_data;

    ble_advdata_manuf_data_t        	manuf_data; // Variable to hold manufacturer specific data
    manuf_data.company_identifier       = 0x0059; // Nordics company ID
    manuf_data.data.p_data              = (uint8_t *) &m_ble_adv_data;
    manuf_data.data.size                = sizeof(ble_adv_data_t);

    /** Build and set advertising data. */
    memset(&adv_data, 0, sizeof(ble_advdata_t));
    adv_data.name_type = BLE_ADVDATA_NO_NAME;
    adv_data.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    adv_data.p_manuf_specific_data = &manuf_data;

    /** Build and set scan response data. */
    memset(&scrsp_data, 0, sizeof(scrsp_data));
    scrsp_data.name_type = BLE_ADVDATA_FULL_NAME;
    scrsp_data.include_appearance = false;

    ble_advdata_set(&adv_data, &scrsp_data);
}

ret_code_t advertising_update_adv_int(uint16_t ms)
{
	if(ms > ADVERTISING_MAXIMUM_PERIOD_MS ||
			ms < ADVERTISING_MINIMUM_PERIOD_MS)
	{
		return NRF_ERROR_INVALID_PARAM;
	}
	m_adv_params.interval = MSEC_TO_UNITS(ms, UNIT_0_625_MS);
	return NRF_SUCCESS;
}

ret_code_t advertising_update_tx_pwr(int8_t tx_pwr)
{
	return sd_ble_gap_tx_power_set(tx_pwr);
}

