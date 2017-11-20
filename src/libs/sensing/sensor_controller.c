/*
 * sensor_controller.c
 *
 *  Created on: 17 de nov de 2017
 *      Author: Gabriel
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nrf51.h"
#include "app_config.h"
#include "app_error.h"
#include "app_timer.h"

#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "sensor_public_interface.h"

#include "bmp180_drv.h"
#include "htu21d_drv.h"
#include "tsl2561_drv.h"

#include "sensor_controller.h"

/***********************************************
 * Private global variables
 ***********************************************/
APP_TIMER_DEF(m_pressure_timer);
APP_TIMER_DEF(m_luminosity_timer);
APP_TIMER_DEF(m_temp_hum_timer);

static nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);

static sensor_controller_cfg_data_t m_sc_cfg_data;

static sensor_controller_data_t m_sensor_data;

static sensor_event_callback_t m_event_callback = NULL;

/***********************************************
 * Private functions prototypes
 ***********************************************/
static void sensor_controller_twi_init();

static void sensor_event_callback(sensor_event_t *p_evt);

static void pressure_timer_callback(void *p_ctx);

static void luminosity_timer_callback(void *p_ctx);

static void temp_hum_timer_callback(void *p_ctx);

/***********************************************
 * Public functions implementation
 ***********************************************/
void sensor_controller_init()
{
    /* Common initialization procedure */
    sensor_error_code_t err_code;
    sensor_controller_twi_init();

    memset(&m_sensor_data, 0x00, sizeof(m_sensor_data));
    memset(&m_sc_cfg_data, 0x00, sizeof(m_sc_cfg_data));

    /* Pressure sensor initialization procedure */
    err_code = app_timer_create(&m_pressure_timer, APP_TIMER_MODE_SINGLE_SHOT, pressure_timer_callback);
    APP_ERROR_CHECK(err_code);

    m_sc_cfg_data.bmp_cfg.state = SENSOR_ACTIVE;
    m_sc_cfg_data.bmp_cfg.sampling_interval = PRESSURE_DEFAULT_SAMPLING_INTERVAL;

    err_code = bmp180_drv_begin(&m_twi,
            BMP180_ULTRA_LOW_PWR,
            &m_sc_cfg_data.bmp_cfg.p_pwr_mode,
            (void *) sensor_event_callback);

    err_code = app_timer_start(m_pressure_timer,
            APP_TIMER_TICKS(m_sc_cfg_data.bmp_cfg.sampling_interval, APP_TIMER_PRESCALER),
            NULL);
    APP_ERROR_CHECK(err_code);

    /* Luminosity sensor initialization procedure */
    err_code = app_timer_create(&m_luminosity_timer, APP_TIMER_MODE_SINGLE_SHOT, luminosity_timer_callback);
    APP_ERROR_CHECK(err_code);

    m_sc_cfg_data.tsl_cfg.state = SENSOR_ACTIVE;
    m_sc_cfg_data.tsl_cfg.sampling_interval = LUMINOSITY_DEFAULT_SAMPLING_INTERVAL;

    err_code = tsl2561_drv_begin(&m_twi,
            TSL2561_DEFAULT_INT_TIME,
            TSL2561_DEFAULT_GAIN,
            &m_sc_cfg_data.tsl_cfg.p_config,
            (void *) sensor_event_callback);

    err_code = app_timer_start(m_luminosity_timer,
            APP_TIMER_TICKS(m_sc_cfg_data.tsl_cfg.sampling_interval, APP_TIMER_PRESCALER),
            NULL);
    APP_ERROR_CHECK(err_code);

    /* Temperature and Humidity sensor initialization procedure */
    err_code= app_timer_create(&m_temp_hum_timer, APP_TIMER_MODE_SINGLE_SHOT, temp_hum_timer_callback);
    APP_ERROR_CHECK(err_code);

    m_sc_cfg_data.htu_cfg.state = SENSOR_ACTIVE;
    m_sc_cfg_data.htu_cfg.sampling_interval = TEMP_HUM_DEFAULT_SAMPLING_INTERVAL;

    err_code = htu21d_drv_begin(&m_twi,
            HTU21D_DEFAULT_RESOLUTION,
            &m_sc_cfg_data.htu_cfg.p_res,
            (void *) sensor_event_callback);

    err_code = app_timer_start(m_temp_hum_timer,
            APP_TIMER_TICKS(m_sc_cfg_data.htu_cfg.sampling_interval, APP_TIMER_PRESCALER),
            NULL);
    APP_ERROR_CHECK(err_code);

}

void sensor_controller_event_subscribe(sensor_event_callback_t p_event_callback)
{
    m_event_callback = p_event_callback;
}

sensor_controller_data_t * sensor_controller_get_sensor_data_pointer()
{
    return &m_sensor_data;
}

sensor_controller_cfg_data_t * sensor_controller_get_config_pointer()
{
    return &m_sc_cfg_data;
}

sensor_error_code_t sensor_controller_set_tsl_gain(tsl2561_gain_t gain)
{
    return SENSOR_SUCCESS;
}

sensor_error_code_t sensor_controller_set_tsl_int_time(tsl2561_integration_time_t int_time)
{
    return SENSOR_SUCCESS;
}

sensor_error_code_t sensor_controller_set_htu_res(htu21d_resolution_t res)
{
    return SENSOR_SUCCESS;
}

sensor_error_code_t sensor_controller_set_bmp_pwr_mode(bmp180_pwr_mode_t pwr_mode)
{
    sensor_error_code_t err_code;
    err_code = bmp180_set_pwr_mode(pwr_mode);

    return err_code;
}

sensor_error_code_t sensor_controller_set_sensor_sampling_interval(sensor_type_t sensor, uint32_t interval)
{
    sensor_error_code_t err_code;
    switch(sensor)
    {
        case SENSOR_BMP180:
            err_code = app_timer_stop(m_pressure_timer);
            m_sc_cfg_data.bmp_cfg.sampling_interval = interval;
            pressure_timer_callback(NULL);
            break;
        case SENSOR_TSL2561:
            err_code = app_timer_stop(m_luminosity_timer);
            m_sc_cfg_data.bmp_cfg.sampling_interval = interval;
            luminosity_timer_callback(NULL);
            break;
        case SENSOR_HTU21D:
            err_code = app_timer_stop(m_temp_hum_timer);
            m_sc_cfg_data.htu_cfg.sampling_interval = interval;
            temp_hum_timer_callback(NULL);
            break;
        default:
            return SENSOR_INVALID_PARAMETER;
            break;
    }
    SENSOR_TIMER_ERROR_CHECK(err_code);
    return SENSOR_SUCCESS;
}

sensor_error_code_t sensor_controller_set_sensor_state(sensor_type_t sensor, sensor_state_t state)
{
    if(state != SENSOR_ACTIVE &&
            state != SENSOR_ERROR &&
            state != SENSOR_OFF)
    {
        return SENSOR_INVALID_PARAMETER;
    }

    switch(sensor)
    {
        case SENSOR_BMP180:
            m_sc_cfg_data.bmp_cfg.state = state;
            break;
        case SENSOR_TSL2561:
            m_sc_cfg_data.tsl_cfg.state = state;
            break;
        case SENSOR_HTU21D:
            m_sc_cfg_data.htu_cfg.state = state;
            break;
        default:
            return SENSOR_INVALID_PARAMETER;
            break;
    }
    return SENSOR_SUCCESS;
}

/***********************************************
 * Private functions implementation
 ***********************************************/
static void sensor_controller_twi_init()
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t config = {
            .scl                = TWI_SCL_PIN,
            .sda                = TWI_SDA_PIN,
            .frequency          = NRF_TWI_FREQ_400K,
            .interrupt_priority = APP_IRQ_PRIORITY_LOW,
            .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

static void sensor_event_callback(sensor_event_t *p_evt)
{
    if(p_evt->evt_type == SENSOR_EVT_ERROR)
    {
        sensor_controller_set_sensor_state(p_evt->sensor, SENSOR_ERROR);
        return;
    }
    else
    {
        switch(p_evt->sensor)
        {
            case SENSOR_BMP180:
                if(p_evt->evt_type == SENSOR_EVT_DATA_READY)
                {
                    bmp180_data_t *sensor_data = (bmp180_data_t*) p_evt->p_sensor_data;
                    m_sensor_data.bmp180_data.pressure = sensor_data->pressure;
                    m_sensor_data.bmp180_data.temperature = sensor_data->temperature;
                }
                break;
            case SENSOR_TSL2561:
                if(p_evt->evt_type == SENSOR_EVT_DATA_READY)
                {
                    tsl2561_data_t *sensor_data = (tsl2561_data_t*) p_evt->p_sensor_data;
                    m_sensor_data.tsl2561_data.infrared_lux = sensor_data->infrared_lux;
                    m_sensor_data.tsl2561_data.visible_lux = sensor_data->visible_lux;
                }
                break;
            case SENSOR_HTU21D:
                if(p_evt->evt_type == SENSOR_EVT_DATA_READY)
                {
                    htu21d_data_t *sensor_data = (htu21d_data_t*) p_evt->p_sensor_data;
                    m_sensor_data.htu21d_data.temperature = sensor_data->temperature;
                    m_sensor_data.htu21d_data.humidity = sensor_data->humidity;
                }
                break;

            default:
                break;
        }
        if(m_event_callback != NULL)
        {
            m_event_callback(p_evt);
        }
    }
}

static void pressure_timer_callback(void *p_ctx)
{
    if(m_sc_cfg_data.bmp_cfg.state == SENSOR_ACTIVE)
    {
        uint32_t err_code;
        err_code = bmp180_drv_convert_data();
        app_timer_start(m_pressure_timer,
                APP_TIMER_TICKS(m_sc_cfg_data.bmp_cfg.sampling_interval, APP_TIMER_PRESCALER),
                NULL);
        APP_ERROR_CHECK(err_code);
    }
}

static void luminosity_timer_callback(void *p_ctx)
{
    if(m_sc_cfg_data.tsl_cfg.state == SENSOR_ACTIVE)
    {
        uint32_t err_code;
        tsl2561_drv_convert_data();
        err_code = app_timer_start(m_luminosity_timer,
                APP_TIMER_TICKS(m_sc_cfg_data.tsl_cfg.sampling_interval, APP_TIMER_PRESCALER),
                NULL);
        APP_ERROR_CHECK(err_code);
    }
}

static void temp_hum_timer_callback(void *p_ctx)
{
    if(m_sc_cfg_data.htu_cfg.state == SENSOR_ACTIVE)
    {
        uint32_t err_code;
        htu21d_drv_convert_data();
        err_code = app_timer_start(m_temp_hum_timer,
                APP_TIMER_TICKS(m_sc_cfg_data.htu_cfg.sampling_interval, APP_TIMER_PRESCALER),
                NULL);
        APP_ERROR_CHECK(err_code);
    }
}
