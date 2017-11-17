/*
 * sensor_public_interface.h
 *
 *  Created on: 16 de nov de 2017
 *      Author: Gabriel
 */

#ifndef SRC_LIBS_SENSING_SENSOR_PUBLIC_INTERFACE_H_
#define SRC_LIBS_SENSING_SENSOR_PUBLIC_INTERFACE_H_

typedef struct sensor_event sensor_event_t;

/*******************************************
 * Sensor public enums
 ******************************************/
typedef enum sensor_error
{
    SENSOR_SUCCESS = 0,
    SENSOR_COMMUNICATION_ERROR,
    SENSOR_INVALID_PARAMETER,
}sensor_error_code_t;

typedef enum sensor_state
{
    SENSOR_IDLE = 0,
    SENSOR_BUSY,
    SENSOR_ERROR,
    SENSOR_OFF
}sensor_state_t;

/* Definition of possible events */
typedef enum sensor_evt_type
{
    NO_EVT,
    EVT_DATA_READY,
    EVT_ERROR,
}sensor_evt_type_t;

/* Definition of possible sensors */
typedef enum sensor_type
{
    SENSOR_BMP180,
    SENSOR_HTU21D,
    SENSOR_TSL2561
}sensor_type_t;

/*******************************************
 * Sensor public structs
 ******************************************/
/* Sensor event struct */
struct sensor_event
{
        sensor_type_t sensor;
        sensor_evt_type_t evt_type;
        void * p_sensor_data;
};

#endif /* SRC_LIBS_SENSING_SENSOR_PUBLIC_INTERFACE_H_ */
