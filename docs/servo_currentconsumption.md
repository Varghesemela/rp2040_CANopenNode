# Servo OCP routine

## Prerequisites

### Files and libraries
- [servo.cpp](../lib/servo.cpp)
- [config.h](../config.h)
- [main.cpp](../main.cpp)
- hardware gpio library
- hardware timer library
- hardware adc library

### Variables directly related to servo
- [boolean](../main.cpp#L141): servo_flag, enable_servo, last_enable_servo
- [integers](../main.cpp#L138): temp_current_consumption, servo_angle, CURRENT_LIMIT_count, servo_current_consumption, servo_current_failure, servo_current_integrate, servo_counter, servo_current_limit
- [constants](../main.cpp#L10): CURRENT_LIMIT

## Functions concerned with servo OCP interface
1. [gpio_init()](../main.cpp#L29): This function initialises many gpios as per the requirement which includes brake pin. 
2. [parse_numeric_value()](../functions.cpp#L22): parse ascii data from user and convert it to integer data. 

## servo OCP code routine
1. The adc peripheral is initialised for the adc pin connected to the output of the current sense opamp.
2. The adc value of the pin is read in every iteration and integrated for a 1000 samples.
3. A servo counter runs for a 100 iterations after which the servo current limit check happens.
4. If the current consumption of the servo exceeds the preset current limit, a current limit counter is incremented.
5. If the current limit counter value exceeds 100, the current consumption value is noted and the SMPS power is turned OFF which cuts the power to the servo.
6. A flag **enable_servo** is disabled to block any new servo command execution until the servo power is reset by issuing a reset command '?r' or by hard-reset of the microcontroller.