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

#include "BLE400_board.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "twi_interface.h"

/**
 * @brief Function for doing power management.
 */
//static void power_manage(void)
//{
//    ret_code_t err_code = sd_app_evt_wait();
//    APP_ERROR_CHECK(err_code);
//}


/**@brief Function for application main entry.
 */
int main(void)
{
    twi_init();
	nrf_gpio_cfg_output(BLE400_LED_2_PIN);
    // Enter main loop.
    for (;;)
    {
    	nrf_gpio_pin_toggle(BLE400_LED_2_PIN);
    	nrf_delay_ms(100);
    }
}

