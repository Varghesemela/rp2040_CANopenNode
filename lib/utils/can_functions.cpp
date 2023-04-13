#include <RP2040.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <hardware/pio.h>
#include <hardware/dma.h>
#include <hardware/uart.h>
#include <hardware/flash.h>
#include <hardware/watchdog.h>
#include <pico/bootrom.h>
#include <hardware/irq.h>
#include <core_cm0plus.h>
#include "hardware/clocks.h"
#include <hardware/regs/intctrl.h>
#include "can2040.h"
#include "can_functions.h"
#include "project_config.h"
#include "pins_picofeeder_board.h"
// #include "can_frames.h"
// CAN_frames CAN1;

can2040 cbus;
uint16_t curr_pos, target_pos;

struct can2040_msg tx_msg, rx_msg;

volatile bool cb_called = false;
volatile uint32_t cb_notify;

char *msg_to_str(struct can2040_msg *msg) {

    static char buf[64], buf2[8];

    sprintf(buf, "[ %x ] [ %x ] [ ", msg->id, msg->dlc);
    for (uint32_t i = 0; i < msg->dlc && i < 8; i++) {
    sprintf(buf2, "%x ", msg->data[i]);
    strcat(buf, buf2);
    }
    strcat(buf, " ] ");

    if (msg->id & CAN2040_ID_RTR) {
    strcat(buf, "R");
    }

    if (msg->id & CAN2040_ID_EFF) {
    strcat(buf, "X");
    }
    
    return buf;
}

void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg){
    // Add message processing code here...
    cb_called = true;
    cb_notify = notify;
    if (notify == CAN2040_NOTIFY_RX) {
    rx_msg = *msg;
    }
}

void PIOx_IRQHandler(void){
    can2040_pio_irq_handler(&cbus);
}


void canbus_setup(void){

    // Setup canbus
    can2040_setup(&cbus, CAN_PIO);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO1_IRQ_0_IRQn, PIOx_IRQHandler);
    NVIC_SetPriority(PIO1_IRQ_0_IRQn, 1);

    // Start canbus
    can2040_start(&cbus, clock_get_hz(clk_sys), CAN_Bitrate, CAN_RX, CAN_TX);
    printf("Clockspeed is %d \n", clock_get_hz(clk_sys));
    NVIC_EnableIRQ(PIO1_IRQ_0_IRQn);
}
