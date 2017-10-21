/*
 * BLE400_board.h
 *
 *  Created on: 25 de ago de 2017
 *      Author: Gabriel
 */

#ifndef BOARD_CUSTOM
#define BOARD_CUSTOM

/** TWI PINS */
#define BLE400_TWI_SMBA_PIN     2
#define BLE400_TWI_SCL_PIN      1
#define BLE400_TWI_SDA_PIN      0

/** SPI PINS */
#define BLE400_SPI_NSS_PIN      30
#define BLE400_SPI_SCK_PIN      25
#define BLE400_SPI_MOSI_PIN     24
#define BLE400_SPI_MISO_PIN     23

/** UART PINS */
#define BLE400_UART_TX_PIN      5
#define BLE400_UART_RX_PIN      6
#define BLE400_UART_CTS_PIN     7
#define BLE400_UART_RTS_PIN     12

/** LED PINS */
#define BLE400_LED_0_PIN        18
#define BLE400_LED_1_PIN        19
#define BLE400_LED_2_PIN        20
#define BLE400_LED_3_PIN        21
#define BLE400_LED_4_PIN        22

/** KEY PINS */
#define BLE400_KEY_1_PIN        16
#define BLE400_KEY_2_PIN        17

#endif /* BOARD_CUSTOM */
