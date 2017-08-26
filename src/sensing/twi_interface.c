/*
 * twi_interface.c
 *
 *  Created on: 13 de ago de 2017
 *      Author: Gabriel
 */


#include "app_config.h"
#include "twi_interface.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"

#define TWI_INSTANCE_ID     0

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief TWI events handler.
 */
//void twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
//{
//    switch (p_event->type)
//    {
//        case NRF_DRV_TWI_EVT_DONE:
//            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
//            {
//
//            }
//            m_xfer_done = true;
//            break;
//        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
//        	break;
//
//        case NRF_DRV_TWI_EVT_DATA_NACK:
//        	break;
//
//        default:
//            break;
//    }
//}

void twi_init()
{
    uint32_t err_code;

	    const nrf_drv_twi_config_t m_twi_config = {
	       .scl                = TWI_SCL_PIN,
	       .sda                = TWI_SDA_PIN,
	       .frequency          = NRF_TWI_FREQ_400K,
	       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
	       .clear_bus_init     = false
	    };

	    err_code = nrf_drv_twi_init(&m_twi, &m_twi_config, NULL, NULL);
	    APP_ERROR_CHECK(err_code);

	    nrf_drv_twi_enable(&m_twi);
}

void twi_tx(uint8_t address, uint8_t *p_data, uint8_t length, bool no_stop)
{
    uint32_t err_code;
    err_code = nrf_drv_twi_tx(&m_twi, address,  p_data, length, no_stop);
    APP_ERROR_CHECK(err_code);
}

void twi_rx(uint8_t address, uint8_t *p_data, uint8_t length)
{
    uint32_t err_code;
    err_code = nrf_drv_twi_rx(&m_twi, address,  p_data, length);
    APP_ERROR_CHECK(err_code);
}

