/*
 * tsl2561_driver.h
 *
 *  Created on: 12 de ago de 2017
 *      Author: Gabriel
 */

#ifndef SRC_SENSING_TSL2561_DRIVER_H_
#define SRC_SENSING_TSL2561_DRIVER_H_

void tsl2561_drv_init();
void tsl2561_drv_enable();
void tsl2561_drv_disable();
void tsl2561_drv_set_integration_time();
void tsl2561_drv_set_gain_time();
void tsl2561_drv_read_luminosity();



#endif /* SRC_SENSING_TSL2561_DRIVER_H_ */
