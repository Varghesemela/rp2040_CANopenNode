# Servo routine

## Prerequisites

### Files and libraries
- [servo.cpp](../lib/servo.cpp)
- [config.h](../config.h)
- [main.cpp](../main.cpp)
- hardware gpio library
- hardware timer library
- hardware pwm library

### Variables directly related to servo
- [integers](../main.cpp#L27): angle, servo_angle
- [constants](../lib/servo.cpp#L34): SERVO_TOP_MAX

## Functions concerned with servo interface
1. [gpio_init()](../main.cpp#L29): This function initialises many gpios as per the requirement which includes brake pin. 
2. [parse_numeric_value()](../functions.cpp#L22): parse ascii data from user and convert it to integer data. 
3. [servo_set_position(const uint gpio_pin, const uint16_t degree)](../lib/servo.cpp#L88): to set a servo position based on calculated pwm value.
4. [servo_enable(const uint gpio_pin)](../lib/servo.cpp#L45): to enable servo funntionality on a specific gpio pin

## servo code routine
1. Servo GPIO pin is initialised and function mode is set as PWM for the pin.
2. The slice and channel numbers for the pin are derived and PWM is enabled for the corresponding channel position.
3. The frequency for the PWM signal is set based on the slice and channel number and servo functionality is enabled. Then the default servo position is set.
4. The user/master can then assign any position to the servo using the 's' command followed by the servo position.
5. The servo position is set by calling **servo_set_position** function. This function converts the position to timer values and updates the registers using the inbuilt PWM librariy functions.
