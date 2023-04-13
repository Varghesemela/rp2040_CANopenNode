#ifndef CAN_FUNCTIONS_H
#define CAN_FUNCTIONS_H
#include <cstdint>
#include "can2040.h"
#include "can_frames.h"

typedef struct can2040 CANHandle;
typedef struct can2040_msg CANMsg;

extern uint16_t curr_pos, target_pos;
extern volatile bool cb_called;
extern volatile uint32_t cb_notify;
extern struct can2040_msg tx_msg, rx_msg;

extern can2040 cbus;

char *msg_to_str(struct can2040_msg *msg);
void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg);
void PIOx_IRQHandler(void);
void canbus_setup(void);


#endif