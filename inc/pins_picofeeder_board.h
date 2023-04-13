#ifndef PINS_PICOSTROBE_BOARD_H
#define PINS_PICOSTROBE_BOARD_H


#define BOARD_TYPE "Blockhoming"
#define HW_VERSION "HwVer2_1"


/*
* GPIO Pins declaration for strobing
*/

#define i2c0_sda_pin    4
#define i2c0_scl_pin    5
#define i2c1_sda_pin    6
#define i2c1_scl_pin    7

#define SPI_PORT    spi1
#define MOSI        11
#define SCK         10
#define CS          13

#define UART_ID     uart0
#define UART_RX_PIN   17
#define UART_TX_PIN   16
#define DXL_DIR_PIN   19

#define CAN_RX      5 
#define CAN_TX      4

#define LED_DRV_PIN     18
#define SERVO_PIN       16
#define BOOST_EN        22
#define LED_STAT        25
#define LED_SIG         28
#define MOTOR_STEP      20
#define MOTOR_DIR       21
#define MOTOR_ENA       13
#define LS1             15
#define LS2             14
#define ENC1_A           2
#define ENC1_B           3

#endif