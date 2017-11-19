/*
 * tsl2561_drv.c
 *
 *  Created on: 17 de nov de 2017
 *      Author: Gabriel
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "app_config.h"
#include "app_timer.h"
#include "nrf_drv_twi.h"

#include "sensor_public_interface.h"
#include "tsl2561_drv.h"

/***********************************************
 * Private defines
 ***********************************************/
#define TSL2561_DEVICE_ADDR         0x39

#define TSL2561_CMD_TEMPLATE        0xC0

#define TSL2561_VISIBLE 2                   // channel 0 - channel 1
#define TSL2561_INFRARED 1                  // channel 1
#define TSL2561_FULLSPECTRUM 0              // channel 0


#define TSL2561_LUX_SCALE         (14)      // Scale by 2^14
#define TSL2561_RATIO_SCALE       (9)       // Scale ratio by 2^9

//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// Integration time scaling factors
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
#define TSL2561_CH_SCALE       (10)      // Scale channel values by 2^10
#define TSL2561_CH_SCALE_TINT0 (0x7517)  // 322/11 * 2^TSL2561_LUX_CHSCALE
#define TSL2561_CH_SCALE_TINT1 (0x0FE7)  // 322/81 * 2^TSL2561_LUX_CHSCALE

//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// T, FN, and CL Package coefficients
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// For Ch1/Ch0=0.00 to 0.50
//      Lux/Ch0=0.0304−0.062*((Ch1/Ch0)^1.4)
//      piecewise approximation
//          For Ch1/Ch0=0.00 to 0.125:
//              Lux/Ch0=0.0304−0.0272*(Ch1/Ch0)
//
//          For Ch1/Ch0=0.125 to 0.250:
//              Lux/Ch0=0.0325−0.0440*(Ch1/Ch0)
//
//          For Ch1/Ch0=0.250 to 0.375:
//              Lux/Ch0=0.0351−0.0544*(Ch1/Ch0)
//
//          For Ch1/Ch0=0.375 to 0.50:
//              Lux/Ch0=0.0381−0.0624*(Ch1/Ch0)
//
// For Ch1/Ch0=0.50 to 0.61:
//      Lux/Ch0=0.0224−0.031*(Ch1/Ch0)
//
// For Ch1/Ch0=0.61 to 0.80:
//      Lux/Ch0=0.0128−0.0153*(Ch1/Ch0)
//
// For Ch1/Ch0=0.80 to 1.30:
//      Lux/Ch0=0.00146−0.00112*(Ch1/Ch0)
//
// For Ch1/Ch0>1.3:
//      Lux/Ch0=0
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
#define TSL2561_K1T           (0x0040)  // 0.125 * 2^RATIO_SCALE
#define TSL2561_B1T           (0x01f2)  // 0.0304 * 2^LUX_SCALE
#define TSL2561_M1T           (0x01be)  // 0.0272 * 2^LUX_SCALE

#define TSL2561_K2T           (0x0080)  // 0.250 * 2^RATIO_SCALE
#define TSL2561_B2T           (0x0214)  // 0.0325 * 2^LUX_SCALE
#define TSL2561_M2T           (0x02d1)  // 0.0440 * 2^LUX_SCALE

#define TSL2561_K3T           (0x00c0)  // 0.375 * 2^RATIO_SCALE
#define TSL2561_B3T           (0x023f)  // 0.0351 * 2^LUX_SCALE
#define TSL2561_M3T           (0x037b)  // 0.0544 * 2^LUX_SCALE

#define TSL2561_K4T           (0x0100)  // 0.50 * 2^RATIO_SCALE
#define TSL2561_B4T           (0x0270)  // 0.0381 * 2^LUX_SCALE
#define TSL2561_M4T           (0x03fe)  // 0.0624 * 2^LUX_SCALE

#define TSL2561_K5T           (0x0138)  // 0.61 * 2^RATIO_SCALE
#define TSL2561_B5T           (0x016f)  // 0.0224 * 2^LUX_SCALE
#define TSL2561_M5T           (0x01fc)  // 0.0310 * 2^LUX_SCALE

#define TSL2561_K6T           (0x019a)  // 0.80 * 2^RATIO_SCALE
#define TSL2561_B6T           (0x00d2)  // 0.0128 * 2^LUX_SCALE
#define TSL2561_M6T           (0x00fb)  // 0.0153 * 2^LUX_SCALE

#define TSL2561_K7T           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_B7T           (0x0018)  // 0.00146 * 2^LUX_SCALE
#define TSL2561_M7T           (0x0012)  // 0.00112 * 2^LUX_SCALE

#define TSL2561_K8T           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_B8T           (0x0000)  // 0.000 * 2^LUX_SCALE
#define TSL2561_M8T           (0x0000)  // 0.000 * 2^LUX_SCALE

//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// CS package coefficients
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
// For 0 <= Ch1/Ch0 <= 0.52
//     Lux/Ch0 = 0.0315−0.0593*((Ch1/Ch0)^1.4)
//        piecewise approximation
//            For 0 <= Ch1/Ch0 <= 0.13
//                Lux/Ch0 = 0.0315−0.0262*(Ch1/Ch0)
//            For 0.13 <= Ch1/Ch0 <= 0.26
//                Lux/Ch0 = 0.0337−0.0430*(Ch1/Ch0)
//            For 0.26 <= Ch1/Ch0 <= 0.39
//                Lux/Ch0 = 0.0363−0.0529*(Ch1/Ch0)
//            For 0.39 <= Ch1/Ch0 <= 0.52
//                Lux/Ch0 = 0.0392−0.0605*(Ch1/Ch0)
//
//     For 0.52 < Ch1/Ch0 <= 0.65
//         Lux/Ch0 = 0.0229−0.0291*(Ch1/Ch0)
//     For 0.65 < Ch1/Ch0 <= 0.80
//         Lux/Ch0 = 0.00157−0.00180*(Ch1/Ch0)
//     For 0.80 < Ch1/Ch0 <= 1.30
//         Lux/Ch0 = 0.00338−0.00260*(Ch1/Ch0)
//     For Ch1/Ch0 > 1.30
//         Lux = 0
//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
#define TSL2561_K1C           (0x0043)  // 0.130 * 2^RATIO_SCALE
#define TSL2561_B1C           (0x0204)  // 0.0315 * 2^LUX_SCALE
#define TSL2561_M1C           (0x01ad)  // 0.0262 * 2^LUX_SCALE

#define TSL2561_K2C           (0x0085)  // 0.260 * 2^RATIO_SCALE
#define TSL2561_B2C           (0x0228)  // 0.0337 * 2^LUX_SCALE
#define TSL2561_M2C           (0x02c1)  // 0.0430 * 2^LUX_SCALE

#define TSL2561_K3C           (0x00c8)  // 0.390 * 2^RATIO_SCALE
#define TSL2561_B3C           (0x0253)  // 0.0363 * 2^LUX_SCALE
#define TSL2561_M3C           (0x0363)  // 0.0529 * 2^LUX_SCALE

#define TSL2561_K4C           (0x010a)  // 0.520 * 2^RATIO_SCALE
#define TSL2561_B4C           (0x0282)  // 0.0392 * 2^LUX_SCALE
#define TSL2561_M4C           (0x03df)  // 0.0605 * 2^LUX_SCALE

#define TSL2561_K5C           (0x014d)  // 0.65 * 2^RATIO_SCALE
#define TSL2561_B5C           (0x0177)  // 0.0229 * 2^LUX_SCALE
#define TSL2561_M5C           (0x01dd)  // 0.0291 * 2^LUX_SCALE

#define TSL2561_K6C           (0x019a)  // 0.80 * 2^RATIO_SCALE
#define TSL2561_B6C           (0x0101)  // 0.0157 * 2^LUX_SCALE
#define TSL2561_M6C           (0x0127)  // 0.0180 * 2^LUX_SCALE

#define TSL2561_K7C           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_B7C           (0x0037)  // 0.00338 * 2^LUX_SCALE
#define TSL2561_M7C           (0x002b)  // 0.00260 * 2^LUX_SCALE

#define TSL2561_K8C           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_B8C           (0x0000)  // 0.000 * 2^LUX_SCALE
#define TSL2561_M8C           (0x0000)  // 0.000 * 2^LUX_SCALE

/***********************************************
 * Private macros
 ***********************************************/

/***********************************************
 * Private enums
 ***********************************************/
typedef enum tsl2561_reg_addr
{
    TSL2561_REG_ADDR_CONTROL    = 0x00,
    TSL2561_REG_ADDR_TIMING     = 0x01,
    TSL2561_REG_ADDR_TSH_LL     = 0x02,
    TSL2561_REG_ADDR_TSH_LH     = 0x03,
    TSL2561_REG_ADDR_TSH_HL     = 0x04,
    TSL2561_REG_ADDR_TSH_HH     = 0x05,
    TSL2561_REG_ADDR_INT        = 0x06,
    TSL2561_REG_ADDR_CRC        = 0x08,
    TSL2561_REG_ADDR_ID         = 0x0A,
    TSL2561_REG_ADDR_DATA0L     = 0x0C,
    TSL2561_REG_ADDR_DATA0H     = 0x0D,
    TSL2561_REG_ADDR_DATA1L     = 0x0E,
    TSL2561_REG_ADDR_DATA1H     = 0x0F
} tsl2561_reg_addr_t;

typedef enum tsl2561_power
{
    TSL2561_POWER_UP = 0x03,
    TSL2561_POWER_DOWN = 0x00
}tsl2561_power_t;

/***********************************************
 * Private structs
 ***********************************************/
typedef struct tsl2561_adc_data
{
        uint16_t data0;
        uint16_t data1;
}tsl2561_adc_data_t;

/***********************************************
 * Private global variables
 ***********************************************/
APP_TIMER_DEF(m_tsl2561_internal_timer);

static tsl2561_config_t m_sensor_config;

static tsl2561_data_t m_sensor_data;

static nrf_drv_twi_t *mp_twi;

static tsl2561_event_cb_t p_event_callback = NULL;

static sensor_event_t m_last_evt;


/***********************************************
 * Private functions prototypes
 ***********************************************/
static void timeout_cb(void * p_ctx);

static uint16_t get_conversion_time(tsl2561_integration_time_t int_time);

//static sensor_error_code_t tsl2561_drv_check_res_integrity(tsl2561_integration_time_t * int_time);

static sensor_error_code_t calculate_lux(tsl2561_adc_data_t data);

static void error_call(sensor_evt_type_t evt_type, sensor_error_code_t err_code);

static sensor_error_code_t tsl2561_drv_set_power(tsl2561_power_t pwr);
/***********************************************
 * Public functions implementation
 ***********************************************/
sensor_error_code_t tsl2561_drv_begin( nrf_drv_twi_t *p_twi,
        tsl2561_integration_time_t int_time,
        tsl2561_gain_t gain,
        tsl2561_config_t **config,
        tsl2561_event_cb_t (* tsl2561_event_cb)(sensor_event_t *event_data))
{
    sensor_error_code_t err_code;
    m_last_evt.sensor = SENSOR_TSL2561;

    if(p_twi == NULL) return SENSOR_INVALID_PARAMETER;

    *config = &m_sensor_config;
    p_event_callback = (tsl2561_event_cb_t) tsl2561_event_cb;
    mp_twi = p_twi;

    memset(&m_sensor_data, 0x00, sizeof(m_sensor_data));

    err_code = app_timer_create(&m_tsl2561_internal_timer, APP_TIMER_MODE_SINGLE_SHOT, timeout_cb);
    SENSOR_TIMER_ERROR_CHECK(err_code);

    err_code = tsl2561_drv_set_gain(gain);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    err_code = tsl2561_drv_set_integration_time(int_time);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    return SENSOR_SUCCESS;
}

sensor_error_code_t tsl2561_drv_convert_data()
{
    sensor_error_code_t err_code;
    err_code = tsl2561_drv_set_power(TSL2561_POWER_UP);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    uint16_t conversion_time = get_conversion_time(m_sensor_config.int_time);

    err_code = app_timer_start(m_tsl2561_internal_timer,
            APP_TIMER_TICKS(conversion_time, APP_TIMER_PRESCALER),
            NULL );
    SENSOR_TIMER_ERROR_CHECK(err_code);

    return SENSOR_SUCCESS;
}

tsl2561_data_t tsl2561_drv_get_last_conversion_data()
{
    return m_sensor_data;
}

sensor_error_code_t tsl2561_drv_set_integration_time(tsl2561_integration_time_t int_time)
{
    static uint8_t cmd[2];
    cmd[0] = TSL2561_CMD_TEMPLATE | TSL2561_REG_ADDR_TIMING;
    cmd[1] = 0x00 | (m_sensor_config.gain << 4) | int_time;

    sensor_error_code_t err_code;
    err_code = nrf_drv_twi_tx(mp_twi, TSL2561_DEVICE_ADDR, cmd, 2, false);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    m_sensor_config.int_time = int_time;

    return SENSOR_SUCCESS;
}

sensor_error_code_t tsl2561_drv_set_gain(tsl2561_gain_t gain)
{
    uint8_t cmd[2];
    cmd[0] = TSL2561_CMD_TEMPLATE | TSL2561_REG_ADDR_TIMING;
    cmd[1] = 0x00 | (gain << 4) | m_sensor_config.int_time;

    sensor_error_code_t err_code;
    err_code = nrf_drv_twi_tx(mp_twi, TSL2561_DEVICE_ADDR, cmd, 2, false);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    m_sensor_config.gain = gain;
    return SENSOR_SUCCESS;
}

/***********************************************
 * Private functions implementation
 ***********************************************/
static void timeout_cb(void * p_ctx)
{
    uint8_t cmd;
    sensor_error_code_t err_code;

    tsl2561_adc_data_t data = {0,0};
    cmd = TSL2561_CMD_TEMPLATE | TSL2561_REG_ADDR_DATA0L;
    err_code = nrf_drv_twi_tx(mp_twi, TSL2561_DEVICE_ADDR, &cmd, 1, true);
    if(err_code != NRF_SUCCESS) error_call(SENSOR_EVT_ERROR, SENSOR_COMMUNICATION_ERROR);

    err_code = nrf_drv_twi_rx(mp_twi, TSL2561_DEVICE_ADDR, (uint8_t *) &data.data0, 2);
    if(err_code != NRF_SUCCESS) error_call(SENSOR_EVT_ERROR, SENSOR_COMMUNICATION_ERROR);

    cmd = TSL2561_CMD_TEMPLATE | TSL2561_REG_ADDR_DATA1L;
    err_code = nrf_drv_twi_tx(mp_twi, TSL2561_DEVICE_ADDR, &cmd, 1, true);
    if(err_code != NRF_SUCCESS) error_call(SENSOR_EVT_ERROR, SENSOR_COMMUNICATION_ERROR);

    err_code = nrf_drv_twi_rx(mp_twi, TSL2561_DEVICE_ADDR, (uint8_t *) &data.data1, 2);
    if(err_code != NRF_SUCCESS) error_call(SENSOR_EVT_ERROR, SENSOR_COMMUNICATION_ERROR);

    err_code = tsl2561_drv_set_power(TSL2561_POWER_DOWN);
    if(err_code != NRF_SUCCESS) error_call(SENSOR_EVT_ERROR, SENSOR_COMMUNICATION_ERROR);

    calculate_lux(data);

    m_last_evt.evt_type = SENSOR_EVT_DATA_READY;
    m_last_evt.p_sensor_data = &m_sensor_data;

    if(p_event_callback != NULL)
    {
        p_event_callback(&m_last_evt);
    }
}

static uint16_t get_conversion_time(tsl2561_integration_time_t int_time)
{
    switch(int_time)
    {
        case TSL2561_INTEGRATION_TIME_13_7MS:
            return 15;
            break;
        case TSL2561_INTEGRATION_TIME_101MS:
            return 105;
            break;
        case TSL2561_INTEGRATION_TIME_402MS:
            return 405;
            break;
    }
    return 405;
}

static sensor_error_code_t calculate_lux(tsl2561_adc_data_t data)
{
    uint32_t ch0buff = data.data0;
    uint32_t ch1buff = data.data1;

    uint32_t ch_scale;

    /* If the integration time is NOT 402ms it is scaled */
    switch(m_sensor_config.int_time)
    {
        case TSL2561_INTEGRATION_TIME_13_7MS:
            ch_scale = TSL2561_CH_SCALE_TINT0;
            break;
        case TSL2561_INTEGRATION_TIME_101MS:
            ch_scale = TSL2561_CH_SCALE_TINT0;
            break;
        default: // Assume no scaling
            ch_scale = (1 << TSL2561_CH_SCALE);
            break;
    }


    /* Scale if the gain is not 16x */
    if(!m_sensor_config.gain)
    {
        ch_scale = ch_scale << 4; // Scale 1x to 16x
    }

    ch0buff = (data.data0 * ch_scale) >> TSL2561_CH_SCALE;
    ch1buff = (data.data1 * ch_scale) >> TSL2561_CH_SCALE;

    /** find the ratio between channels (ch1 / ch0) */
    uint32_t ratio = 0;
    if(ch0buff != 0)
    {
        ratio = (ch1buff << (TSL2561_RATIO_SCALE + 1)) / ch0buff;
    }

    /** Rounding the division */
    ratio = (ratio + 1) >> 1;

    uint16_t b;
    uint16_t m;

    /** For now, it's only supported the T, FN and CL package */

    if ((ratio >= 0) && (ratio <= TSL2561_K1T))
    {
        b = TSL2561_B1T;
        m = TSL2561_M1T;
    }
    else if (ratio <= TSL2561_K2T)
    {
        b = TSL2561_B2T;
        m = TSL2561_M2T;
    }
    else if (ratio <= TSL2561_K3T)
    {
        b = TSL2561_B3T;
        m = TSL2561_M3T;
    }
    else if (ratio <= TSL2561_K4T)
    {
        b = TSL2561_B4T;
        m = TSL2561_M4T;
    }
    else if (ratio <= TSL2561_K5T)
    {
        b = TSL2561_B5T;
        m = TSL2561_M5T;
    }
    else if (ratio <= TSL2561_K6T)
    {
        b = TSL2561_B6T;
        m = TSL2561_M6T;
    }
    else if (ratio <= TSL2561_K7T)
    {
        b = TSL2561_B7T;
        m = TSL2561_M7T;
    }
    else if (ratio > TSL2561_K8T)
    {
        b = TSL2561_B8T;
        m = TSL2561_M8T;
    }

    uint32_t vis_lux;
    uint32_t ir_lux;
    ir_lux = (ch1buff * m);
    vis_lux = ((ch0buff * b) - ir_lux);

    vis_lux += (1 << (TSL2561_LUX_SCALE - 1));
    ir_lux += (1 << (TSL2561_LUX_SCALE - 1));

    vis_lux = vis_lux >> TSL2561_LUX_SCALE;
    ir_lux = ir_lux >> TSL2561_LUX_SCALE;

    m_sensor_data.infrared_lux = ir_lux;
    m_sensor_data.visible_lux = vis_lux;

    return NRF_SUCCESS;
}

static void error_call(sensor_evt_type_t evt_type, sensor_error_code_t err_code)
{
    static sensor_error_code_t m_err_code;
    m_err_code = err_code;

    m_last_evt.evt_type = SENSOR_EVT_ERROR;
    m_last_evt.p_sensor_data = &m_err_code;

    if(p_event_callback != NULL)
    {
        p_event_callback(&m_last_evt);
    }
}

static sensor_error_code_t tsl2561_drv_set_power(tsl2561_power_t pwr)
{
    uint8_t cmd[2];
    cmd[0] = TSL2561_CMD_TEMPLATE | TSL2561_REG_ADDR_CONTROL;
    cmd[1] = pwr;

    sensor_error_code_t err_code;
    err_code = nrf_drv_twi_tx(mp_twi, TSL2561_DEVICE_ADDR, cmd, 2, false);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    return SENSOR_SUCCESS;
}
