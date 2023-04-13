#define PICO_FLASH_SPI_CLKDIV   4

#include <cmath>
#include <cstring>
#include <string>
using namespace std;

#include "encoders.pio.h"
#include "project_config.h"
#include "pins_picofeeder_board.h"
#include "encoder_config.h"
#include "functions.h"
#include "can_functions.h"
// #include "pico/unique_id.h"

int global_count;

typedef struct can2040_msg CANMsg;
#define LOG( msg ) printf(( __FILE__, msg ))
// pico_unique_board_id_t id_out;

PIO pio;
struct repeating_timer timer1, timer2;

// vars
int directions[num_of_axis] = {encoder_dir};
const uint encoder_pin[num_of_axis] = {ENC1_A};
uint number_of_encoders = num_of_axis;
int32_t encoder_resolution[num_of_axis] = {encoder_PPR * 4};
float encoder_mm_per_revolution[num_of_axis] = {(DEFAULT_LEADSCREW_PITCH)};
volatile float encoder_mm_per_click[num_of_axis];
volatile float axis_offset[num_of_axis] = {0}, added_offset[num_of_axis] = {0};
float homing_position[num_of_axis] = {0};               // the absolute position of the loader when it is homed
volatile int32_t capture_buf[num_of_axis] = {0};



void GPIO_init() {
    gpio_init(LED_SIG);
    gpio_set_dir(LED_SIG, GPIO_OUT);
    gpio_put(LED_SIG, LOW);

    gpio_init(LED_STAT);
    gpio_set_dir(LED_STAT, GPIO_OUT);
    gpio_put(LED_STAT, LOW);

}

void dma_handler() {
    uint i = 1;
    int interrupt_channel = 0;
    while ((i & dma_hw->ints0) == 0) {
        i = i << 1;
        ++interrupt_channel;
    }
    dma_hw->ints0 = 1u << interrupt_channel;
    dma_channel_set_read_addr(interrupt_channel, &pio->rxf[interrupt_channel], true);
}


#define BUFFER_LENGTH   50
char buffer[BUFFER_LENGTH];
const char s[2] = ",";
const char start_of_frame = '<', end_of_frame = '>';
string command_str, subcommand_str;
string data_str;
uint16_t buffer_index = 0, EOF_index = 0;
bool SOF_flag = false, EOF_flag = false;
CANMsg msg = {0};

union {
    uint8_t axis_val[4];
    float axis_fval;
} buffer_union;

// Core 1 interrupt Handler
void core1_interrupt_handler() {
    // Receive Raw Value, Convert and Print Temperature Value
    // while (multicore_fifo_rvalid()){
    //     printf("Core 1");
    // }
    multicore_fifo_clear_irq(); // Clear interrupt
}

/**
 * \brief Core 1 main loop
 *
 * This function serves as the main loop for the secondary core when initialised in the primary core.
 *
 */
void core1_entry() {
    // Configure Core 1 Interrupt
    multicore_fifo_clear_irq();
    multicore_lockout_victim_init();
    // irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_interrupt_handler);
    // irq_set_enabled(SIO_IRQ_PROC1, true);

    while (true) {
        tight_loop_contents();
        gpio_put(LED_STAT, true);
        sleep_ms(333);
        gpio_put(LED_STAT, false);
        sleep_ms(333);

    }
}


inline int can_loop() {
    msg.dlc = 8;
    if (cb_called) {
        switch (cb_notify) {
            // received message
            case CAN2040_NOTIFY_RX:
                printf("cb: received ID %x \n", rx_msg.id);
                switch (rx_msg.id) {
                    case (sendIDCluster + SteppermotorID):
                        switch (rx_msg.data[0]) {
                            case 0:
                                for (int i = 0; i < 4; i++) {
                                    buffer_union.axis_val[i] = rx_msg.data[CAN_start_data + i];
                                }

                                break;
                        }
                }
                break;

                // message sent ok
            case CAN2040_NOTIFY_TX:
                // printf("cb: message sent ok\n");
                break;

                // error
            case CAN2040_NOTIFY_ERROR:
                printf("cb: an error occurred\n");
                break;

                // unknown
            default:
                printf("cb: unknown notification = %lu\n", cb_notify);
        }
        cb_called = false;
    }
    return true;
}


bool get_block(struct repeating_timer *t) {
    can_loop();
    // buffer_index = 0;
    // while(true){
    int c = getchar_timeout_us(0);
    // printf("%c\n", c);
    if (c == PICO_ERROR_TIMEOUT){
        printf("[%s %d] Error! getchar timeout\n", __FUNCTION__, __LINE__);
        return true;
    }
    if((buffer_index > BUFFER_LENGTH)){
        printf("[%s %d] Error! Buffer filled\n", __FUNCTION__, __LINE__);
        buffer_index = 0;
        return true;
    }
    if(EOF_flag){
        printf("[%s %d] Error! EOF true\n",  __FUNCTION__, __LINE__);
        return true;
    }
    if (c == start_of_frame) {
        SOF_flag = true;
        command_str.clear();
        subcommand_str.clear();
//            data_str.clear();
        std::fill( std::begin(data_str), std::end(data_str), '\0');
        std::fill( std::begin(buffer), std::end(buffer), '\0' );
        buffer_index = 0;
    } else if (c == end_of_frame) {
        EOF_flag = true;
        EOF_index = buffer_index;

        command_str = strtok(buffer, s);
        if (command_str == "#RP64209"){
            reset_usb_boot(0, 0);
        }
        if(!is_only_alphabets(command_str)){
            printf("Invalid command, change later\n");
        }
    }
    if (SOF_flag && !EOF_flag && (c != start_of_frame)) {
        buffer[buffer_index++] = (c & 0xFF);
    }
    return true;
}


int main() {
    set_sys_clock_khz(250000, true);
    stdio_init_all();
    multicore_launch_core1(core1_entry);
    GPIO_init();
    // Set up the state machine.
    pio = ENC_PIO;

    for (int i = 0; i < num_of_axis; i++) {
        assert(encoder_resolution[i] > 0);
        encoder_mm_per_click[i] = encoder_mm_per_revolution[i] / (float) encoder_resolution[i];
    }

    uint offset = pio_add_program(pio, &encoders_program);
    for (uint i = 0; i < number_of_encoders; i++) {
        encoders_program_init(pio, i, offset, encoder_pin[i], false);

        dma_channel_config c = dma_channel_get_default_config(i);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(pio, i, false));
        dma_channel_configure(i, &c,
                              &capture_buf[i],    // Destinatinon pointer
                              &pio->rxf[i],       // Source pointer
                              0xffff,             // Number of transfers
                              true                // Start immediately
        );
        irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
        irq_set_enabled(DMA_IRQ_0, true);
        dma_channel_set_irq0_enabled(i, true);
    }

    axis_offset[0] = 0;


    adc_init();
    adc_gpio_init(26);// Make sure GPIO is high-impedance, no pullups etc
    adc_select_input(0); //Or Select ADC input 1 (GPIO27)
    adc_set_clkdiv(25600);

    sleep_ms(3000);
    canbus_setup();

    printf("Status: %d\n", add_repeating_timer_us(-10 * TICK_RATE, get_block, NULL, &timer1));

    printf("Hello\n");
    gpio_put(BOOST_EN, HIGH);
    while (true) {
        if (!EOF_flag) {

        } else {
            if (command_str == "value") {
                data_str = strtok(nullptr, s);
                printf(data_str.c_str());
                printf("\n");
                double pos_in_mm;
                pos_in_mm = strtod(data_str.c_str(), NULL);
                printf("value: %f", pos_in_mm);
                memset(buffer, '\0', sizeof(buffer));

            } else if (command_str == "u2") {
                printf(BOARD_TYPE "\n");
                printf("ok\n");

            } else if (command_str == "help") {
                printCommandlist();
            }

            buffer_index = 0;
            EOF_index = 0;
            memset(buffer, '\0', sizeof(buffer));
            SOF_flag = false;
            EOF_flag = false;
        }
    }
}