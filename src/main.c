/** @file main.c
 * @brief Initialize all modules.
 * @author Gabriel Nascimento dos Santos
 *
 * The main starts the Sensing Module, the Advertising cotroller and the BLE Services.
 * \n In addition, the application scheduler, application timer and the low frequency clock are also set by the main.
 * \n This device will be interrupt driven, so the microcontroller will be in sleep state
 * until some interrupt (radio or timer) wake it up to perform a task and sleep again.
 *
 * \n UNIVERSIDADE FEDERAL DO ABC
 * \n Curso: Engenharia de Instrumentação, Automação e Robótica
 * \n Projeto: Desenvolvimento de rede de sensores bluetooth conectados à nuvem
 * \n\n Desenvolvido durante as disciplinas de Trabalho de Graduação 1, 2 e 3 do curso.
 */

/**
 * @mainpage Bluetooth Low Energy Sensor Station
 *
 * \n UNIVERSIDADE FEDERAL DO ABC
 * \n Curso: Engenharia de Instrumentação, Automação e Robótica
 * \n Projeto: Desenvolvimento de rede de sensores bluetooth conectados à nuvem
 * \n\n Desenvolvido durante as disciplinas de Trabalho de Graduação 1, 2 e 3 do curso.
 *
 * \dot
 * digraph Architecture{
 * label="Red line: Data flow\nBlue lines: Command flow\nGreen lines:Configuration flow"
 * App [shape=box, label=Application]
 * ADVCfg [label="Advertising \n Configuration \n Service"]
 * ADVCtrl [label="Advertising \n Controller"]
 * ADVEnc [label="Advertising\nframe\nencoder"]
 * BLEMan [label="Bluetooth Manager"]
 * BMPDrv [label="BMP180\ndriver"]
 * HTUDrv [label="HTU21D\ndriver"]
 * SC [label="Sensor Controller"]
 * SDKTWI [label="SDK I2C\ndriver"]
 * SDRadio [label="Softdevice\nRadio"]
 * SensSvc [label="Sensor\nconfig\nservice"]
 * TSLDrv [label="TSL2561\ndriver"]
 *
 * App -> SC [color=blue]
 * ADVCfg -> ADVCtrl [color=green]
 * ADVCfg -> ADVCtrl [color=red]
 * ADVCtrl -> ADVEnc [color=blue]
 * ADVCtrl -> SDRadio [color=blue]
 * ADVCtrl -> SDRadio [color=red]
 * ADVEnc -> ADVCtrl [color=red]
 * App -> BLEMan [color=blue]
 * BLEMan -> ADVCfg [color=blue]
 * BLEMan -> ADVCtrl [color=blue]
 * BLEMan -> SensSvc [color=blue]
 * BMPDrv -> SC [color=red]
 * BMPDrv -> SDKTWI [color=blue]
 * HTUDrv -> SC [color=red]
 * HTUDrv -> SDKTWI [color=blue]
 * SC -> ADVCtrl [color=red]
 * SC -> BMPDrv [color=blue]
 * SC -> BMPDrv [color=green]
 * SC -> HTUDrv [color=blue]
 * SC -> HTUDrv [color=green]
 * SC -> SensSvc [color=green, dir=both]
 * SC -> SensSvc [color=red]
 * SC -> TSLDrv [color=blue]
 * SC -> TSLDrv [color=green]
 * SDKTWI -> BMPDrv [color=red]
 * SDKTWI -> HTUDrv [color=red]
 * SDKTWI -> TSLDrv [color=red]
 * TSLDrv -> SC [color=red]
 * TSLDrv -> SDKTWI [color=blue]
 * }
 * \enddot
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "nrf51.h"
#include "app_config.h"

#include "app_timer.h"
#include "app_timer_appsh.h"
#include "app_scheduler.h"
#include "app_error.h"

#include "nrf_drv_twi.h"
#include "sensor_public_interface.h"

#include "bmp180_drv.h"
#include "htu21d_drv.h"
#include "tsl2561_drv.h"

#include "ble_manager.h"

#include "softdevice_handler.h"
#include "sensor_controller.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


/**
 * @brief Function for doing power management.
 */
static void power_manage(void);

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Start the application scheduler
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    // Start the application timer
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, app_timer_evt_schedule);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    NRF_LOG_INFO("nrf51 test\n");
    NRF_LOG_FLUSH();

    // Set the clock configuration
    nrf_clock_lf_cfg_t lf_clock_config;
    lf_clock_config.source = NRF_CLOCK_LF_SRC_XTAL;
    lf_clock_config.rc_ctiv = 0;
    lf_clock_config.rc_temp_ctiv = 0;
    lf_clock_config.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM;

    // Initialize the SoftDevice handler module.
     SOFTDEVICE_HANDLER_INIT(&lf_clock_config, NULL);
    NRF_LOG_INFO("Initializing FW\n");
    NRF_LOG_FLUSH();

    sensor_controller_init();
    ble_manager_init();

    // Enter main loop.
    for (;;)
    {
        app_sched_execute();
        power_manage();
        NRF_LOG_FLUSH();
    }
}

static void power_manage(void)
{
     ret_code_t err_code = sd_app_evt_wait();
     APP_ERROR_CHECK(err_code);
}

