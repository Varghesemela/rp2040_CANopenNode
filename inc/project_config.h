#ifndef CONFIG_H
#define CONFIG_H

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <pico/multicore.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "pico/bootrom.h"


#define FW_TYPE "PicoFeeder"
#define FW_VERSION "FwVer1_0 stepper"
#define AUTHOR      "SNJY"

// #define _DEBUG
#ifdef _DEBUG
#define printf_debug  printf
#else
#define printf_debug(s, ...) ((void)0)
#endif


#define SECRET_CODE 64209

#define HIGH        1
#define LOW         0

#define CW          HIGH
#define CCW         LOW


#define ENC_PIO         pio0
#define CAN_PIO         1
#define CAN_Bitrate     1000000

#define TICK_RATE   500.0

// vars

extern int directions[];
extern const uint encoder_pin[];
extern uint number_of_encoders;
extern int32_t encoder_resolution[];
 
extern float encoder_mm_per_revolution[];
extern volatile float encoder_mm_per_click[];
extern volatile float axis_offset[] , added_offset[];
extern float homing_position[];               // the absolute position of the loader when it is homed
extern volatile int32_t capture_buf[];

extern PIO pio;
extern int servo_angle;


#endif