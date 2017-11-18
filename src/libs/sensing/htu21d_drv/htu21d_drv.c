/*
 * htu21d_drv.c
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
#include "nrf_error.h"

#include "sensor_public_interface.h"
#include "htu21d_drv.h"

/***********************************************
 * Private defines
 ***********************************************/
#define HTU21D_DEVICE_ADDR                  0x40

#define HTU21D_TEMP_14BIT_CONVERSION_TIME   55
#define HTU21D_TEMP_13BIT_CONVERSION_TIME   30
#define HTU21D_TEMP_12BIT_CONVERSION_TIME   15
#define HTU21D_TEMP_11BIT_CONVERSION_TIME   10

#define HTU21D_RH_12BIT_CONVERSION_TIME   20
#define HTU21D_RH_11BIT_CONVERSION_TIME   10
#define HTU21D_RH_10BIT_CONVERSION_TIME   6
#define HTU21D_RH_8BIT_CONVERSION_TIME    5

/***********************************************
 * Private macros
 ***********************************************/

/***********************************************
 * Private enums
 ***********************************************/
typedef enum htu21d_reg_addr
{
    HTU21D_REG_CONV_TEMP_HOLD = 0xE3,
    HTU21D_REG_CONV_HUM_HOLD = 0xE5,
    HTU21D_REG_CONV_TEMP_NO_HOLD = 0xF3,
    HTU21D_REG_CONV_HUM_NO_HOLD = 0xF5,
    HTU21D_REG_WRITE_USER = 0xE6,
    HTU21D_REG_READ_USER = 0xE7,
    HTU21D_REG_SOFT_RESET = 0xFE
} htu21d_reg_addr_t;

/***********************************************
 * Private structs
 ***********************************************/

/***********************************************
 * Private global variables
 ***********************************************/
APP_TIMER_DEF(m_htu21d_internal_timer);

static htu21d_resolution_t m_sensor_resolution;

static htu21d_data_t m_sensor_data;

static nrf_drv_twi_t *mp_twi;

static htu21d_event_cb_t p_event_callback = NULL;

static sensor_event_t m_last_evt;


/***********************************************
 * Private functions prototypes
 ***********************************************/
static void timeout_cb(void * p_ctx);

static uint16_t get_temp_conv_time(htu21d_resolution_t resolution);

static uint16_t get_hum_conv_time(htu21d_resolution_t resolution);

//static uint32_t htu21d_drv_check_res_integrity(htu21d_resolution_t * resolution);

static int16_t calculate_temperature(uint16_t buffer);

static uint16_t calculate_rh(uint16_t buffer);

static void error_call(sensor_evt_type_t evt_type, sensor_error_code_t err_code);

static htu21d_event_cb_t p_event_callback;

/***********************************************
 * Public functions implementation
 ***********************************************/
sensor_error_code_t htu21d_drv_begin(nrf_drv_twi_t *p_twi,
        htu21d_resolution_t resolution,
        htu21d_resolution_t **p_p_res,
        htu21d_event_cb_t (*htu21d_event_cb)(sensor_event_t * event_data))
{
    sensor_error_code_t err_code;
    m_last_evt.sensor = SENSOR_HTU21D;

    if(p_twi == NULL) return SENSOR_INVALID_PARAMETER;

    *p_p_res = &m_sensor_resolution;
    p_event_callback = (htu21d_event_cb_t) p_event_callback;
    mp_twi = p_twi;

    memset(&m_sensor_data, 0x00, sizeof(m_sensor_data));

    err_code = app_timer_create(&m_htu21d_internal_timer, APP_TIMER_MODE_SINGLE_SHOT, timeout_cb);
    SENSOR_TIMER_ERROR_CHECK(err_code);

    err_code = htu21d_drv_set_resolution(m_sensor_resolution);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    return SENSOR_SUCCESS;
}

sensor_error_code_t htu21d_drv_convert_data()
{
    uint8_t cmd = HTU21D_REG_CONV_TEMP_NO_HOLD;
    sensor_error_code_t err_code;

    err_code = nrf_drv_twi_tx(mp_twi, HTU21D_DEVICE_ADDR, &cmd, sizeof(cmd), false);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    uint16_t conversion_time = get_temp_conv_time(m_sensor_resolution);

    err_code = app_timer_start(m_htu21d_internal_timer,
            APP_TIMER_TICKS(conversion_time, APP_TIMER_PRESCALER) ,
            htu21d_drv_convert_data);
    SENSOR_TIMER_ERROR_CHECK(err_code);

    return SENSOR_SUCCESS;
}

htu21d_data_t htu21d_get_last_conversion_data()
{
    return m_sensor_data;
}

sensor_error_code_t htu21d_drv_set_resolution(htu21d_resolution_t resolution)
{
    sensor_error_code_t err_code;
//    err_code = htu21d_drv_check_res_integrity(&resolution);
//    HTU21D_RETURN_IF_ERROR(err_code);

    uint8_t cmd[2] = {HTU21D_REG_WRITE_USER, resolution};
    err_code = nrf_drv_twi_tx(mp_twi, HTU21D_DEVICE_ADDR, cmd, sizeof(cmd), false);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    m_sensor_resolution = resolution;

    return SENSOR_SUCCESS;
}

/***********************************************
 * Private functions implementation
 ***********************************************/
static void timeout_cb(void * p_ctx)
{
    static uint16_t buffer;
    sensor_error_code_t err_code;
    buffer = 0;

    if(p_ctx == NULL)
    {
        return;
    }
    else if(p_ctx == htu21d_drv_convert_data)
    {
        /* Read the converted temperature, then start the humidity conversion */
        static uint8_t convert_cmd = HTU21D_REG_CONV_HUM_NO_HOLD;
        err_code = nrf_drv_twi_rx(mp_twi, HTU21D_DEVICE_ADDR,  (uint8_t *) &buffer, 2);
        if(err_code != NRF_SUCCESS) error_call(SENSOR_EVT_ERROR, SENSOR_COMMUNICATION_ERROR);

        buffer = BYTES_REVERSE_16BIT(buffer);

        m_sensor_data.temperature = calculate_temperature(buffer);

        err_code = nrf_drv_twi_tx(mp_twi, HTU21D_DEVICE_ADDR, &convert_cmd, sizeof(convert_cmd), false);
        if(err_code != NRF_SUCCESS) error_call(SENSOR_EVT_ERROR, SENSOR_COMMUNICATION_ERROR);

        uint16_t conversion_time = get_hum_conv_time(m_sensor_resolution);

        err_code = app_timer_start(m_htu21d_internal_timer,
                APP_TIMER_TICKS(conversion_time, APP_TIMER_PRESCALER) ,
                timeout_cb);

        if(err_code != NRF_SUCCESS) error_call(SENSOR_EVT_ERROR, SENSOR_TIMER_ERROR);
    }
    else if(p_ctx == timeout_cb)
    {
        /* Read the converted humidity. */
        err_code = nrf_drv_twi_rx(mp_twi, HTU21D_DEVICE_ADDR,  (uint8_t *) &buffer, 2);
        if(err_code != NRF_SUCCESS) error_call(SENSOR_EVT_ERROR, SENSOR_COMMUNICATION_ERROR);

        buffer = BYTES_REVERSE_16BIT(buffer);
        m_sensor_data.humidity = calculate_rh(buffer);
        if(err_code != NRF_SUCCESS) error_call(SENSOR_EVT_ERROR, SENSOR_COMMUNICATION_ERROR);

        m_last_evt.evt_type = SENSOR_EVT_DATA_READY;
        m_last_evt.p_sensor_data = &m_sensor_data;

        if(p_event_callback != NULL)
        {
            p_event_callback(&m_last_evt);
        }
    }
}

static uint16_t get_temp_conv_time(htu21d_resolution_t resolution)
{
    switch(resolution)
    {
        case HTU21D_RES_RH_12_TEMP_14:
            return HTU21D_TEMP_14BIT_CONVERSION_TIME;
            break;
        case HTU21D_RES_RH_8_TEMP_12:
            return HTU21D_TEMP_12BIT_CONVERSION_TIME;
            break;
        case HTU21D_RES_RH_10_TEMP_13:
            return HTU21D_TEMP_13BIT_CONVERSION_TIME;
            break;
        case HTU21D_RES_RH_11_TEMP_11:
            return HTU21D_TEMP_11BIT_CONVERSION_TIME;
            break;
    }
    return HTU21D_TEMP_14BIT_CONVERSION_TIME;
}

static uint16_t get_hum_conv_time(htu21d_resolution_t resolution)
{
    switch(resolution)
    {
        case HTU21D_RES_RH_12_TEMP_14:
            return HTU21D_RH_12BIT_CONVERSION_TIME;
            break;
        case HTU21D_RES_RH_8_TEMP_12:
            return HTU21D_RH_8BIT_CONVERSION_TIME;
            break;
        case HTU21D_RES_RH_10_TEMP_13:
            return HTU21D_RH_10BIT_CONVERSION_TIME;
            break;
        case HTU21D_RES_RH_11_TEMP_11:
            return HTU21D_RH_11BIT_CONVERSION_TIME;
            break;
    }
    return HTU21D_RH_12BIT_CONVERSION_TIME;
}

static int16_t calculate_temperature(uint16_t buffer)
{
    int32_t temp;

    temp = (17572*buffer)/65535;
    temp = temp - 4685;

    return (int16_t) temp;
}

static uint16_t calculate_rh(uint16_t buffer)
{
    uint32_t rh;

    rh = (12500*buffer)/65535;
    rh = rh - 600;

    return (uint16_t) rh;
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
