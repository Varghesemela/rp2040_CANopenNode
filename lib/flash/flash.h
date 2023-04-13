#ifndef FLASH_H
#define FLASH_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "pico/multicore.h"

extern int servo_angle, servo_current_limit;


union {
    uint8_t axis_val[4]; float axis_fval;
    }axis_Union;
extern const uint8_t *flash_target_contents;

int update_flash_data();

#endif