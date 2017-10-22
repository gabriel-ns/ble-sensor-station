/**
 * \file sensor_manager.c
 * \brief   Implements the the sensor manager module.
 *
 *          The sensor manager module is responsible for handling all the
 *          application sensors, setting it's configurations, holding
 *          the data and for periodically requiring new sensor readings.
 *
 */
/*
 *  UNIVERSIDADE FEDERAL DO ABC
 *  Aluno: Gabriel Nascimento dos Santos
 *  Curso: Engenharia de Instrumentação, Automação e Robótica
 *
 *  Projeto: DESENVOLVIMENTO DE REDE DE SENSORES BLUETOOTH LOW ENERGY CONECTADOS À NUVEM
 */

#include <stdint.h>
#include <stddef.h>

#include "nrf51.h"
#include "app_config.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_drv_twi.h"

#include "nrf5-bmp180-drv.h"
#include "nrf5-htu21d-drv.h"
#include "nrf5-tsl2561-drv.h"

#define NRF_LOG_MODULE_NAME "SENSOR_MANAGER"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "sensor_manager.h"

APP_TIMER_DEF(pressure_timer);
APP_TIMER_DEF(th_timer);
APP_TIMER_DEF(lum_timer);

typedef enum sensor_ctx
{
    SENSOR_CTX_NO_OP = 0x00,
    SENSOR_CTX_CONV_A,
    SENSOR_CTX_CONV_B,
}sensor_ctx_t;

static sensor_ctx_t htu21d_last_op = SENSOR_CTX_NO_OP;
static sensor_ctx_t bmp180_last_op = SENSOR_CTX_NO_OP;
static sensor_ctx_t tsl2561_last_op = SENSOR_CTX_NO_OP;
static nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
static sensor_manager_data_buff_t data_buffer = {0,0,0,0};

static sensor_manager_cfg_data_t sensor_cfg_data =
{
        .bmp180_pwr = BMP180_DEFAULT_PWR_MODE,
        .htu21d_res = HTU21D_DEFAULT_RESOLUTION,
        .tsl2561_gain = TSL2561_DEFAULT_GAIN,
        .tsl2561_int_time = TSL2561_DEFAULT_INT_TIME,
        .tsl2561_pwr = TSL2561_POWER_DOWN,
        .lum_sampling_interval = LUMINOSITY_DEFAULT_SAMPLING_INTERVAL,
        .th_sampling_interval = TEMP_HUM_DEFAULT_SAMPLING_INTERVAL,
        .pres_sampling_interval = PRESSURE_DEFAULT_SAMPLING_INTERVAL,
        .pres_state = SENSOR_STATE_UNINIT,
        .th_state = SENSOR_STATE_UNINIT,
        .lum_state = SENSOR_STATE_UNINIT,
};

#if 0
static void sensor_manager_twi_init();
static void sensor_manager_lum_timer_cb(void * p_context);
static void sensor_manager_th_timer_cb(void * p_context);
static void sensor_manager_pres_timer_cb(void * p_context);

void sensor_manager_init()
{
    uint32_t err_code;

    NRF_LOG_INFO("Initializing TWI\n");
    NRF_LOG_FLUSH();
    sensor_manager_twi_init();

    NRF_LOG_INFO("Initializing BMP180\n");
    NRF_LOG_FLUSH();
    err_code = bmp180_drv_begin(&m_twi, sensor_cfg_data.bmp180_pwr);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Initializing HTU21D\n");
    NRF_LOG_FLUSH();
    err_code = htu21d_drv_begin(&m_twi, sensor_cfg_data.htu21d_res);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Initializing TSL2561\n");
    NRF_LOG_FLUSH();
    err_code = tsl2561_drv_begin(&m_twi);
    APP_ERROR_CHECK(err_code);
    err_code = tsl2561_drv_set_gain(sensor_cfg_data.tsl2561_gain);
    APP_ERROR_CHECK(err_code);
    err_code = tsl2561_drv_set_integration_time(sensor_cfg_data.tsl2561_int_time);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Initializing timers\n");
    NRF_LOG_FLUSH();
    err_code = app_timer_create(&pressure_timer,
            APP_TIMER_MODE_SINGLE_SHOT,
            sensor_manager_pres_timer_cb);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&th_timer,
            APP_TIMER_MODE_SINGLE_SHOT,
            sensor_manager_th_timer_cb);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&lum_timer,
            APP_TIMER_MODE_SINGLE_SHOT,
            sensor_manager_lum_timer_cb);
    APP_ERROR_CHECK(err_code);

    sensor_manager_set_sensor_state(SENSOR_BMP180, SENSOR_STATE_ACTIVE);
    sensor_manager_set_sensor_state(SENSOR_TSL2561, SENSOR_STATE_ACTIVE);
    sensor_manager_set_sensor_state(SENSOR_HTU21D, SENSOR_STATE_ACTIVE);
}

/* Sensor configuration functions */
ret_code_t sensor_manager_set_pres_cfg(bmp180_pwr_mode_t pwr_mode)
{
    return bmp180_drv_set_pwr_mode(pwr_mode);
}

ret_code_t sensor_manager_set_th_cfg(htu21d_resolution_t res)
{
    return htu21d_drv_set_resolution(res);
}

ret_code_t sensor_manager_set_lum_int_time(tsl2561_integration_time_t int_time)
{
    return tsl2561_drv_set_integration_time(int_time);
}

ret_code_t sensor_manager_set_lum_set_gain(tsl2561_gain_t gain)
{
    return tsl2561_drv_set_gain(gain);
}

ret_code_t sensor_manager_set_lum_set_pwr(tsl2561_power_t pwr)
{
    return tsl2561_drv_set_power(pwr);
}

ret_code_t sensor_manager_set_sensor_sampling_interval(sensor_t sensor, uint32_t time_ms)
{
    switch(sensor)
    {
        case SENSOR_BMP180:
            sensor_cfg_data.pres_sampling_interval = time_ms;
            break;
        case SENSOR_HTU21D:
            sensor_cfg_data.th_sampling_interval = time_ms;
            break;
        case SENSOR_TSL2561:
            sensor_cfg_data.lum_sampling_interval = time_ms;
            break;
        default:
            break;
    }
    return NRF_SUCCESS;
}

sensor_manager_cfg_data_t sensor_manager_get_curr_cfg()
{
    return sensor_cfg_data;
}


ret_code_t sensor_manager_set_sensor_state(sensor_t sensor, sensor_state_t state)
{
    ret_code_t err_code;
    sensor_state_t * p_sensor_st = NULL;
    app_timer_id_t * p_timer_id = NULL;
//    uint32_t time = 0;
    sensor_ctx_t *ctx = NULL;

    switch(sensor)
        {
            case SENSOR_BMP180:
                p_sensor_st = &sensor_cfg_data.pres_state;
                p_timer_id = &pressure_timer;
//                time = bmp180_drv_get_conv_time();
                ctx = &bmp180_last_op;
                break;
            case SENSOR_HTU21D:
                p_sensor_st = &sensor_cfg_data.th_state;
                p_timer_id = &th_timer;
//                time = htu21d_drv_get_hum_conversion_time();
                ctx = &htu21d_last_op;
                break;
            case SENSOR_TSL2561:
                p_sensor_st = &sensor_cfg_data.lum_state;
                p_timer_id = &lum_timer;
//                time = tsl2561_get_read_time();
                ctx = &tsl2561_last_op;
                break;
            default:
                return NRF_ERROR_INVALID_PARAM;
                break;
        }

    if(*p_sensor_st == state)
    {
        NRF_LOG_INFO("New state = Current state\n");
        NRF_LOG_FLUSH();
        return NRF_SUCCESS;
    }
    else
    {
        if((*p_sensor_st == SENSOR_STATE_ERROR &&
                state != SENSOR_STATE_ERROR) ||
                state == SENSOR_STATE_UNINIT)
        {
            NRF_LOG_INFO("Forbidden state change operation\n");
            NRF_LOG_FLUSH();
            return NRF_ERROR_FORBIDDEN;
        }
        else if(state != SENSOR_STATE_ACTIVE)
        {
            NRF_LOG_INFO("Setting state to %d\n", state);
            NRF_LOG_FLUSH();
            *p_sensor_st = state;
            err_code = app_timer_stop(*p_timer_id);
            APP_ERROR_CHECK(err_code);
        }
        else if(state == SENSOR_STATE_ACTIVE)
        {
            NRF_LOG_INFO("Setting state to active\n");
            NRF_LOG_FLUSH();
            *p_sensor_st = state;
            *ctx = SENSOR_CTX_NO_OP;
            err_code = app_timer_start(*p_timer_id,10, NULL);
            APP_ERROR_CHECK(err_code);
        }
    }
    return NRF_SUCCESS;
}

sensor_state_t sensor_manager_get_sensor_state(sensor_t sensor)
{
    switch(sensor)
        {
            case SENSOR_BMP180:
                return sensor_cfg_data.pres_state;
                break;
            case SENSOR_HTU21D:
                return sensor_cfg_data.th_state;
                break;
            case SENSOR_TSL2561:
                return sensor_cfg_data.lum_state;
                break;
            default:
                break;
        }
    return SENSOR_INVALID;
}

/* Sensor data handling functions */
sensor_manager_data_buff_t sensor_manager_get_data_buff()
{
    return data_buffer;
}

int16_t sensor_manager_last_get_temperature()
{
    return data_buffer.temperature;
}

uint16_t sensor_manager_last_get_humidity()
{
    return data_buffer.humidity;
}

int32_t sensor_manager_last_get_pressure()
{
    return data_buffer.pressure;
}

int32_t sensor_manager_last_get_luminosity()
{
    return data_buffer.luminosity;
}

static void sensor_manager_twi_init()
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

static void sensor_manager_lum_timer_cb(void * p_context)
{
    static tsl2561_adc_data_t adc_data = {0,0};
    switch(tsl2561_last_op)
    {
        default:
        case SENSOR_CTX_NO_OP:
            NRF_LOG_INFO("Starting luminosity acquire.\n");
            NRF_LOG_FLUSH();
            tsl2561_last_op = SENSOR_CTX_CONV_A;
            tsl2561_drv_set_power(TSL2561_POWER_UP);
            app_timer_start(lum_timer,
                        APP_TIMER_TICKS(tsl2561_get_read_time(), APP_TIMER_PRESCALER),
                        NULL);
            break;

        case SENSOR_CTX_CONV_A:

            tsl2561_drv_read_data(&adc_data);
            tsl2561_drv_set_power(TSL2561_POWER_DOWN);
            data_buffer.luminosity = tsl2561_drv_calculate_lux(&adc_data);
            NRF_LOG_FLUSH();
            tsl2561_last_op = SENSOR_CTX_NO_OP;
            NRF_LOG_INFO("LUX = %d\n",data_buffer.luminosity);
            app_timer_start(lum_timer,
                    APP_TIMER_TICKS(sensor_cfg_data.lum_sampling_interval, APP_TIMER_PRESCALER),
                    NULL);
            break;
    }
}

static void sensor_manager_th_timer_cb(void * p_context)
{
    static uint16_t buffer;
    switch(htu21d_last_op)
    {
        default:
        case SENSOR_CTX_NO_OP:
            NRF_LOG_INFO("Starting humidity acquire.\n");
            NRF_LOG_FLUSH();
            htu21d_last_op = SENSOR_CTX_CONV_A;
            htu21d_drv_convert_hum_no_hold();
            app_timer_start(th_timer,
                        APP_TIMER_TICKS(htu21d_drv_get_hum_conversion_time(), APP_TIMER_PRESCALER),
                        NULL);
            break;

        case SENSOR_CTX_CONV_A:
            NRF_LOG_INFO("Starting temperature acquire.\n");
            NRF_LOG_FLUSH();
            htu21d_get_last_conversion(&buffer);
            htu21d_drv_convert_temp_no_hold();

            app_timer_start(th_timer,
                                APP_TIMER_TICKS(htu21d_drv_get_temp_conversion_time(), APP_TIMER_PRESCALER),
                                NULL);
            data_buffer.humidity = htu21d_calculate_rh(buffer);
            NRF_LOG_INFO("Humidity = %d\n",data_buffer.humidity);
            NRF_LOG_FLUSH();
            tsl2561_last_op = SENSOR_CTX_CONV_B;
            break;

        case SENSOR_CTX_CONV_B:
            htu21d_get_last_conversion(&buffer);
            data_buffer.temperature = htu21d_calculate_temperature(buffer);
            tsl2561_last_op = SENSOR_CTX_NO_OP;
            NRF_LOG_INFO("TEMPERATURE = %d\n",data_buffer.temperature);
            NRF_LOG_FLUSH();
            app_timer_start(lum_timer,
                    APP_TIMER_TICKS(sensor_cfg_data.th_sampling_interval, APP_TIMER_PRESCALER),
                    NULL);
            break;
    }
}

static void sensor_manager_pres_timer_cb(void * p_context)
{
    int16_t buffer;

    switch(bmp180_last_op)
    {
        default:
        case SENSOR_CTX_NO_OP:
            NRF_LOG_INFO("Starting pressure acquire.\n");
            NRF_LOG_FLUSH();
            bmp180_last_op = SENSOR_CTX_CONV_A;
            bmp180_drv_convert_temp();
            app_timer_start(pressure_timer,
                    APP_TIMER_TICKS(bmp180_drv_get_conv_time(), APP_TIMER_PRESCALER),
                    NULL);
            break;

        case SENSOR_CTX_CONV_A:
            bmp180_drv_get_temp(&buffer);
            bmp180_drv_convert_pres();
            bmp180_last_op = SENSOR_CTX_CONV_B;
            app_timer_start(pressure_timer,
                    APP_TIMER_TICKS(bmp180_drv_get_conv_time(), APP_TIMER_PRESCALER),
                    NULL);
            break;

        case SENSOR_CTX_CONV_B:
            bmp180_drv_get_pres(&data_buffer.pressure);
            bmp180_last_op = SENSOR_CTX_NO_OP;
            NRF_LOG_INFO("Pressure = %d\n",data_buffer.pressure);
            NRF_LOG_FLUSH();
            app_timer_start(pressure_timer,
                    APP_TIMER_TICKS(sensor_cfg_data.pres_sampling_interval, APP_TIMER_PRESCALER),
                    NULL);
            break;
    }
}

#endif


