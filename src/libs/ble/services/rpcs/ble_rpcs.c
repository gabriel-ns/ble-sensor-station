/*
 * ble_rpcs.c
 *
 *  Created on: Jun 15, 2018
 *      Author: gabriel
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "app_config.h"
#include "app_error.h"

#include "ble_adv_controller.h"
#include "ble_services_common.h"
#include "ble.h"
#include "ble_gatts.h"
#include "ble_rpcs.h"

/**
 * @brief Function that handles the writing attempts to this service's characteristics.
 *
 * @param p_rpcs        Pointer to the application rpcs Service structure.
 * @param p_ble_evt     Pointer to the event received from BLE stack.
 */
static void on_write(ble_rpcs_t *p_rpcs, ble_evt_t *p_ble_evt);

/**
 * @brief Handle read/write authorize request events from the SoftDevice.
 *
 * @param p_rpcs        Pointer to the application rpcs Service structure.
 * @param p_ble_evt     Pointer to the event received from BLE stack.
 */
static uint32_t on_rw_authorize_req(ble_rpcs_t *p_rpcs, ble_evt_t *p_ble_evt);

static void ble_rpcs_tx_pwr_char_init(ble_rpcs_t *p_rpcs);
static void ble_rpcs_adv_int_char_init(ble_rpcs_t *p_rpcs);

static int8_t current_tx_pwr;
static uint16_t current_adv_interval;

void ble_rpcs_init(ble_rpcs_t * p_rpcs)
{
    uint32_t        err_code;
    ble_uuid_t      service_uuid;
    ble_uuid128_t   base_uuid = { BLE_SERVICES_BASE_UUID };


    service_uuid.uuid = BLE_RPCS_SERVICE_UUID;

    /* Start the service with invalid handle */
    p_rpcs->conn_handle = BLE_CONN_HANDLE_INVALID;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
            &service_uuid,
            &p_rpcs->service_handle);
    APP_ERROR_CHECK(err_code);

	ble_rpcs_adv_int_char_init(p_rpcs);
	ble_rpcs_tx_pwr_char_init(p_rpcs);
}

uint32_t ble_rpcs_on_ble_evt(ble_rpcs_t * p_rpcs, ble_evt_t * p_ble_evt)
{
	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
		p_rpcs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		break;
	case BLE_GAP_EVT_DISCONNECTED:
		p_rpcs->conn_handle = BLE_CONN_HANDLE_INVALID;
		break;
	case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
		on_rw_authorize_req(p_rpcs,p_ble_evt);
		break;
	default:
		// No implementation needed.
		break;
	}
	return NRF_SUCCESS;
}

static uint32_t on_rw_authorize_req(ble_rpcs_t *p_rpcs, ble_evt_t *p_ble_evt)
{
    if (p_ble_evt->evt.gatts_evt.params.authorize_request.type ==
             BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        on_write(p_rpcs, p_ble_evt);
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}

static void on_write(ble_rpcs_t *p_rpcs, ble_evt_t *p_ble_evt)
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

	if(p_evt_write->handle == p_rpcs->adv_int_handle.value_handle)
	{
		uint16_t new_interval = *((uint32_t *) p_evt_write->data);

		uint32_t err_code;
		err_code = advertising_update_adv_int(new_interval);

		if(err_code == NRF_SUCCESS)
		{
			current_adv_interval = new_interval;
			reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
			reply.params.write.len = sizeof(current_adv_interval);
			reply.params.write.p_data = (uint8_t *) (&current_adv_interval);
		}
		else
		{
			reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
			reply.params.write.len = 0;
			reply.params.write.p_data = NULL;
		}
	}
	if(p_evt_write->handle == p_rpcs->tx_pwr_handle.value_handle)
	{
		int8_t new_tx_pwr = *((uint8_t *) p_evt_write->data);

		uint32_t err_code;
		err_code = advertising_update_tx_pwr(new_tx_pwr);

		if(err_code == NRF_SUCCESS)
		{
			current_tx_pwr = new_tx_pwr;
			reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
			reply.params.write.len = sizeof(new_tx_pwr);
			reply.params.write.p_data = (uint8_t *) (&new_tx_pwr);
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
	if (p_rpcs->conn_handle != BLE_CONN_HANDLE_INVALID )
	{
		err_code = sd_ble_gatts_rw_authorize_reply(p_rpcs->conn_handle, &reply);
		if (err_code != NRF_SUCCESS)
		{
			APP_ERROR_CHECK(err_code);
		}
	}
}

static void ble_rpcs_tx_pwr_char_init(ble_rpcs_t *p_rpcs)
{
	uint32_t            err_code;
	ble_uuid_t          char_uuid;
	ble_uuid128_t       base_uuid = { BLE_SERVICES_BASE_UUID };

	char_uuid.uuid = BLE_RPCS_ADV_TX_PWR_UUID;

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

	current_tx_pwr = DEFAULT_ADVERTISING_TX_POWER;
	//Set characteristic length in number of bytes and value
	attr_char_value.max_len     = sizeof(current_tx_pwr);
	attr_char_value.init_len    = sizeof(current_tx_pwr);
	attr_char_value.p_value     = (uint8_t *) &current_tx_pwr;

	//Add characteristic to the service
	err_code = sd_ble_gatts_characteristic_add(p_rpcs->service_handle,
			&char_md,
			&attr_char_value,
			&p_rpcs->tx_pwr_handle);

	APP_ERROR_CHECK(err_code);
}

static void ble_rpcs_adv_int_char_init(ble_rpcs_t *p_rpcs)
{
	uint32_t            err_code;
	ble_uuid_t          char_uuid;
	ble_uuid128_t       base_uuid = { BLE_SERVICES_BASE_UUID };

	char_uuid.uuid = BLE_RPCS_ADV_INTERVAL_UUID;

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

	current_adv_interval = ADVERTISING_DEFAULT_PERIOD_MS;
	//Set characteristic length in number of bytes and value
	attr_char_value.max_len     = sizeof(current_adv_interval);
	attr_char_value.init_len    = sizeof(current_adv_interval);
	attr_char_value.p_value     = (uint8_t *) &current_adv_interval;

	//Add characteristic to the service
	err_code = sd_ble_gatts_characteristic_add(p_rpcs->service_handle,
			&char_md,
			&attr_char_value,
			&p_rpcs->adv_int_handle);

	APP_ERROR_CHECK(err_code);
}
