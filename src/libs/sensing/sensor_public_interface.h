/*
 * sensor_public_interface.h
 *
 *  Created on: 16 de nov de 2017
 *      Author: Gabriel
 */

#ifndef SRC_LIBS_SENSING_SENSOR_PUBLIC_INTERFACE_H_
#define SRC_LIBS_SENSING_SENSOR_PUBLIC_INTERFACE_H_

/*******************************************
 * Sensor public MACROS
 ******************************************/
#define BYTES_REVERSE_32BIT(x) ((x << 24) | ((x << 8) & 0x00FF0000) | ((x >> 8) & 0x0000FF00) | (x >> 24))
#define BYTES_REVERSE_16BIT(x) (((x << 8) & 0xFF00) | ((x >> 8) & 0x00FF))

#define RETURN_IF_ERROR(ERR)     \
    do{                                 \
        if(ERR != SENSOR_SUCCESS)       \
        {                               \
            return ERR;                 \
        }                               \
    }while(0)

#define SENSOR_TIMER_ERROR_CHECK(ERR)          if(ERR != NRF_SUCCESS) return SENSOR_TIMER_ERROR
#define SENSOR_COMMUNICATION_ERROR_CHECK(ERR)  if(ERR != NRF_SUCCESS) return SENSOR_COMMUNICATION_ERROR

typedef struct sensor_event sensor_event_t;

/*******************************************
 * Sensor public enums
 ******************************************/
typedef enum sensor_error
{
    SENSOR_SUCCESS = 0,
    SENSOR_COMMUNICATION_ERROR,
    SENSOR_INVALID_PARAMETER,
    SENSOR_TIMER_ERROR,
    SENSOR_UNKNOWN_ERROR
}sensor_error_code_t;

typedef enum sensor_state
{
    SENSOR_ACTIVE = 0,
    SENSOR_ERROR,
    SENSOR_OFF
}sensor_state_t;

/* Definition of possible events */
typedef enum sensor_evt_type
{
    SENSOR_NO_EVT,
    SENSOR_EVT_DATA_READY,
    SENSOR_EVT_ERROR,
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

/*******************************************
 * Sensor public callbacks
 ******************************************/
/* Sensor event callback type */
typedef void (*sensor_event_callback_t)(sensor_event_t * event_data);

#endif /* SRC_LIBS_SENSING_SENSOR_PUBLIC_INTERFACE_H_ */
