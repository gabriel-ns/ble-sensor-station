/*
 * bmp180_drv.h
 *
 *  Created on: 16 de nov de 2017
 *      Author: Gabriel
 */

#ifndef SRC_LIBS_SENSING_BMP180_DRV_BMP180_DRV_H_
#define SRC_LIBS_SENSING_BMP180_DRV_BMP180_DRV_H_

/*******************************************
 * BMP180 Public Enums
 ******************************************/
typedef enum pwr_mode
{
    BMP180_ULTRA_LOW_PWR = 0x00,
    BMP180_STANDARD,
    BMP180_HIGH_RES,
    BMP180_ULTRA_HIGH_RES,
    BMP180_SHUTDOWN
}bmp_pwr_mode_t;

/*******************************************
 * BMP180 Public Structs
 ******************************************/


/*******************************************
 * BMP180 Public Functions
 ******************************************/
/* Function to start sensor */
sensor_err_code_t bmp180_drv_begin(nrf_drv_twi_t *p_twi,
        bmp_pwr_mode_t pwr_mode,
        bmp_pwr_mode_t ** p_p_pwr_mode,
        sensor_evt_callback_t (* bmp180_event_cb)(sensor_evt_t * event_data));

sensor_err_code_t bmp180_drv_convert_data();

bmp180_data_t bmp180_drv_get_last_converson();

sensor_err_code_t bmp180_set_pwr_mode(bmp_pwr_mode_t pwr_mode);

#endif /* SRC_LIBS_SENSING_BMP180_DRV_BMP180_DRV_H_ */
