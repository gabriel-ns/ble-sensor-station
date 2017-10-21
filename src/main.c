/**
 * UNIVERSIDADE FEDERAL DO ABC
 * Aluno: Gabriel Nascimento dos Santos
 * Curso: Engenharia de Instrumentação, Automação e Robótica
 *
 * Projeto: DESENVOLVIMENTO DE REDE DE SENSORES BLUETOOTH LOW ENERGY CONECTADOS À NUVEM
 * 
 */


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "app_config.h"
#include "softdevice_handler.h"

#include "app_timer.h"
#include "app_timer_appsh.h"
#include "app_scheduler.h"
#include "app_error.h"

#include "BLE400_board.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"


/**
 * @brief Function for doing power management.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{

    /** Start the application scheduler */
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    /** Start the application timer */
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, app_timer_evt_schedule);

    /** Set the clock configuration */
    nrf_clock_lf_cfg_t lf_clock_config;
    lf_clock_config.source = NRF_CLOCK_LF_SRC_XTAL;
    lf_clock_config.rc_ctiv = 0;
    lf_clock_config.rc_temp_ctiv = 0;
    lf_clock_config.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&lf_clock_config, NULL);

//    ble_manager_init();
//    sensor_manager_init();

    // Enter main loop.
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }
}

