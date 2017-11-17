/*
 * bmp180_drv.c
 *
 *  Created on: 17 de nov de 2017
 *      Author: Gabriel
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nrf_drv_twi.h"

#include "app_timer.h"

#include "sensor_public_interface.h"
#include "bmp180_drv.h"

/***********************************************
 * Private defines
 ***********************************************/
#define BMP180_DEVICE_ADDR    0x77

/***********************************************
 * Private macros
 ***********************************************/



/***********************************************
 * Private enums
 ***********************************************/
typedef enum bmp180_reg_addr
{
    BMP180_REG_OUT_XLSB = 0xF8,
    BMP180_REG_OUT_LSB = 0xF7,
    BMP180_REG_OUT_MSB = 0xF6,
    BMP180_REG_CTRL_MEAS = 0xF4,
    BMP180_REG_SOFT_RESET = 0xE0,
    BMP180_REG_ID = 0xD0,
    BMP180_REG_CALIB_START = 0xAA,
} bmp180_reg_addr_t;

typedef enum cmd
{
    CONVERT_TEMPERATURE         = 0x2E,
    CONVERT_PRES_OSS_SINGLE     = 0x34,
    CONVERT_PRES_OSS_2_TIMES    = 0x74,
    CONVERT_PRES_OSS_4_TIMES    = 0xB4,
    CONVERT_PRES_OSS_8_TIMES    = 0xF4,
    SOFTWARE_RESET              = 0xB6,
    GET_CHIP_ID                 = 0x55
}bmp180_cmd_t;

/***********************************************
 * Private structs
 ***********************************************/
typedef struct __attribute__((packed)) bmp180_callibration
{
    int16_t     AC1;
    int16_t     AC2;
    int16_t     AC3;
    uint16_t    AC4;
    uint16_t    AC5;
    uint16_t    AC6;
    int16_t     B1;
    int16_t     B2;
    int16_t     MB;
    int16_t     MC;
    int16_t     MD;
}bmp180_callibration_t;

/***********************************************
 * Private global variables
 ***********************************************/
APP_TIMER_DEF(m_bmp180_internal_timer);

static bmp180_callibration_t m_calib_data;

static bmp180_pwr_mode_t m_pwr_mode;

static bmp180_data_t m_sensor_data;

static uint16_t raw_buffer;

static nrf_drv_twi_t *mp_twi;

static bmp180_event_cb_t p_event_callback;

static sensor_event_t m_last_evt;
/***********************************************
 * Private functions prototypes
 ***********************************************/
static void timeout_cb(void * p_ctx);

static sensor_error_code_t read_callibration_data();

static uint16_t get_conversion_time(bmp180_pwr_mode_t pwr_mode);

static void calculate_result_values(int16_t raw_temp, uint32_t raw_pres);

static void error_call(sensor_evt_type_t evt_type, sensor_error_code_t err_code);

static sensor_error_code_t send_pressure_convert_cmd();

static sensor_error_code_t read_pressure_data(uint32_t *p_raw_press_data);

/***********************************************
 * Public functions implementation
 ***********************************************/
sensor_error_code_t bmp180_drv_begin(nrf_drv_twi_t *p_twi,
        bmp180_pwr_mode_t pwr_mode,
        bmp180_pwr_mode_t ** p_p_pwr_mode,
        bmp180_event_cb_t (* bmp180_event_cb)(sensor_event_t * event_data))
{
    sensor_error_code_t err_code;
    m_last_evt.sensor = SENSOR_BMP180;

    if(p_twi == NULL) return SENSOR_INVALID_PARAMETER;

    *p_p_pwr_mode = &m_pwr_mode;
    p_event_callback = (bmp180_event_cb_t) bmp180_event_cb;
    mp_twi = p_twi;

    memset(&m_sensor_data, 0x00, sizeof(m_sensor_data));

    err_code = app_timer_create(&m_bmp180_internal_timer, APP_TIMER_MODE_SINGLE_SHOT, timeout_cb);
    SENSOR_TIMER_ERROR_CHECK(err_code);

    err_code = read_callibration_data();
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    err_code = bmp180set_pwr_mode(pwr_mode);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    return SENSOR_SUCCESS;
}

sensor_error_code_t bmp180_drv_convert_data()
{
    static uint8_t tx_data[2] = {BMP180_REG_CTRL_MEAS, CONVERT_TEMPERATURE};

    uint32_t err_code;

    err_code = nrf_drv_twi_tx(mp_twi, BMP180_DEVICE_ADDR, tx_data, sizeof(tx_data), false);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    uint16_t conversion_time = get_conversion_time(m_pwr_mode);

    //TODO app timer prescaler
    err_code = app_timer_start(m_bmp180_internal_timer, APP_TIMER_TICKS(conversion_time, 0) , bmp180_drv_convert_data);
    SENSOR_TIMER_ERROR_CHECK(err_code);

    return SENSOR_SUCCESS;
}

bmp180_data_t bmp180_drv_get_last_converson()
{
    return m_sensor_data;
}

sensor_error_code_t bmp180set_pwr_mode(bmp180_pwr_mode_t pwr_mode)
{
    if(pwr_mode > BMP180_SHUTDOWN) return SENSOR_INVALID_PARAMETER;

    m_pwr_mode = pwr_mode;
    return SENSOR_SUCCESS;
}

/***********************************************
 * Private functions implementation
 ***********************************************/
static sensor_error_code_t send_pressure_convert_cmd()
{
    sensor_error_code_t err_code;
    static uint8_t reg_addr = BMP180_REG_OUT_MSB;

    err_code = nrf_drv_twi_tx(mp_twi, BMP180_DEVICE_ADDR, &reg_addr, sizeof(reg_addr), false);
    if(err_code != SENSOR_SUCCESS) error_call(SENSOR_EVT_ERROR, SENSOR_COMMUNICATION_ERROR);

    /** Store the raw data in the temperature buffer */
    err_code = nrf_drv_twi_rx(mp_twi, BMP180_DEVICE_ADDR,  (uint8_t *) &raw_buffer,2);
    if(err_code != SENSOR_SUCCESS) error_call(SENSOR_EVT_ERROR, SENSOR_COMMUNICATION_ERROR);

    raw_buffer = BYTES_REVERSE_16BIT(raw_buffer);

    static uint8_t tx_data[2] = {BMP180_REG_CTRL_MEAS, CONVERT_PRES_OSS_SINGLE};

    /** Set the conversion command according to the power mode */
    switch(m_pwr_mode)
    {
        case BMP180_ULTRA_LOW_PWR:
            tx_data[1] = CONVERT_PRES_OSS_SINGLE;
            break;

        case BMP180_STANDARD:
            tx_data[1] = CONVERT_PRES_OSS_2_TIMES;
            break;

        case BMP180_HIGH_RES:
            tx_data[1] = CONVERT_PRES_OSS_4_TIMES;
            break;

        case BMP180_ULTRA_HIGH_RES:
            tx_data[1] = CONVERT_PRES_OSS_8_TIMES;
            break;

        default:
            break;
    }

    err_code = nrf_drv_twi_tx(mp_twi, BMP180_DEVICE_ADDR, tx_data, sizeof(tx_data), false);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    uint16_t conversion_time = get_conversion_time(m_pwr_mode);

    //TODO app timer prescaler
    err_code = app_timer_start(m_bmp180_internal_timer, APP_TIMER_TICKS(conversion_time, 0) , send_pressure_convert_cmd );
    SENSOR_TIMER_ERROR_CHECK(err_code);

    return SENSOR_SUCCESS;
}

static sensor_error_code_t read_pressure_data(uint32_t *p_raw_press_data)
{
    sensor_error_code_t err_code;
    uint32_t pressure_data;
    static uint8_t reg_addr = BMP180_REG_OUT_MSB;

    if(p_raw_press_data == NULL) return SENSOR_INVALID_PARAMETER;

    err_code = nrf_drv_twi_tx(mp_twi, BMP180_DEVICE_ADDR, &reg_addr, sizeof(reg_addr), false);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);
    err_code = nrf_drv_twi_rx(mp_twi, BMP180_DEVICE_ADDR,  (uint8_t *) &pressure_data, 3);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    pressure_data = ((BYTES_REVERSE_32BIT(pressure_data)) >> 8) & 0x00FFFFFF;

    /* Resolution correction */
    pressure_data = (pressure_data) >> (8 - m_pwr_mode);
    *p_raw_press_data = pressure_data;

    return SENSOR_SUCCESS;
}
static void timeout_cb(void * p_ctx)
{
    sensor_error_code_t err_code;

    if(p_ctx == bmp180_drv_convert_data)
    {
        err_code = send_pressure_convert_cmd();
        if(err_code != SENSOR_SUCCESS) error_call(SENSOR_EVT_ERROR, err_code);
    }
    else if(p_ctx == send_pressure_convert_cmd)
    {
        uint32_t buff = 0;
        err_code = read_pressure_data(&buff);
        if(err_code != SENSOR_SUCCESS) error_call(SENSOR_EVT_ERROR, err_code);

        calculate_result_values(raw_buffer, buff);

        m_last_evt.evt_type = SENSOR_EVT_DATA_READY;
        m_last_evt.p_sensor_data = &m_sensor_data;

        if(p_event_callback != NULL)
        {
            p_event_callback(&m_last_evt);
        }
    }
}

static sensor_error_code_t read_callibration_data()
{
    sensor_error_code_t err_code;

    /** Clear the current value for all the calibration variables */
    memset(&m_calib_data, 0, sizeof(bmp180_callibration_t));

    /** Set the target register address */
    static uint8_t reg_addr = BMP180_REG_CALIB_START;

    err_code = nrf_drv_twi_tx(mp_twi, BMP180_DEVICE_ADDR, &reg_addr, sizeof(reg_addr), true);
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    /** Store the received callibration data in the g_bmp180_callibration_data structure */
    err_code = nrf_drv_twi_rx(mp_twi, BMP180_DEVICE_ADDR, (uint8_t *) &m_calib_data, sizeof(bmp180_callibration_t));
    SENSOR_COMMUNICATION_ERROR_CHECK(err_code);

    bmp180_callibration_t data_buff = m_calib_data;
    uint16_t * buff;

    for(uint8_t n = 0; n < (sizeof(data_buff)/2);n++)
    {
        buff = ((uint16_t *) (&data_buff)) + n;
        *buff = BYTES_REVERSE_16BIT(*buff);
    }
    m_calib_data = data_buff;

    return SENSOR_SUCCESS;
}

static uint16_t get_conversion_time(bmp180_pwr_mode_t pwr_mode)
{
    /** Return the conversion time in ms according to the power mode */
    switch(pwr_mode)
    {
        case BMP180_ULTRA_LOW_PWR:
            return 5;
            break;

        case BMP180_STANDARD:
            return 8;
            break;

        case BMP180_HIGH_RES:
            return 14;
            break;

        case BMP180_ULTRA_HIGH_RES:
            return 26;
            break;

        default:
            break;
    }

    return 0;
}

static void calculate_result_values(int16_t raw_temp, uint32_t raw_pres)
{
    int32_t X1;
    int32_t X2;
    int32_t X3;

    int32_t B3;
    int32_t B5;
    int32_t B6;
    uint32_t B4;
    uint32_t B7;

    uint32_t local_buff = 0;;

    X1 = ((raw_temp - m_calib_data.AC6) * m_calib_data.AC5) >> 15;
    X2 = (m_calib_data.MC * 2048) / (X1 + m_calib_data.MD);
    B5 = X1 + X2;
    m_sensor_data.temperature = ((B5 + 8) >> 4) * 10;

    B6 = B5 - 4000;
    X1 = (m_calib_data.B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (m_calib_data.AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((m_calib_data.AC1 * 4) + X3) << m_pwr_mode) + 2) >> 2;
    X1 = (m_calib_data.AC3 * B6) >> 13;
    X2 = (m_calib_data.B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = m_calib_data.AC4 * (uint32_t)(X3 + 32768) >> 15;
    B7 = ((uint32_t) raw_pres - B3) * (50000 >> m_pwr_mode);

    if(B7 > 0x80000000)
    {
        local_buff = (B7 * 2) / B4;
    }
    else
    {
        local_buff = (B7 / B4) * 2;
    }

    X1 = (local_buff >> 8) * (local_buff >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * local_buff) >> 16;
    local_buff = local_buff + ((X1 + X2 + 3791) / 16);

    m_sensor_data.pressure = local_buff;
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
