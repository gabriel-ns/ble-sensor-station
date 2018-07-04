/*
 * ble_rpcs.h
 *
 *  Created on: Jun 15, 2018
 *      Author: gabriel
 */

#ifndef _RPCS_H_
#define _RPCS_H_

#define BLE_RPCS_SERVICE_UUID                    0xABC4

#define BLE_RPCS_ADV_INTERVAL_UUID               0x0D00
#define BLE_RPCS_ADV_TX_PWR_UUID                 0x0D01

typedef struct ble_rpcs ble_rpcs_t;

struct ble_rpcs
{
        uint16_t                    conn_handle;
        uint16_t                    service_handle;
        ble_gatts_char_handles_t    adv_int_handle;
        ble_gatts_char_handles_t    tx_pwr_handle;
};

void ble_rpcs_init(ble_rpcs_t * p_rpcs);

uint32_t ble_rpcs_on_ble_evt(ble_rpcs_t * p_rpcs, ble_evt_t * p_ble_evt);
#endif /* _RPCS_H_ */
