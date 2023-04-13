#ifndef ENCODER_CONFIG_H
#define ENCODER_CONFIG_H

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>

//#define Y_BELT_PITCH  2.0
//#define Y_PULLEY_TEETH 20
#define DEFAULT_LEADSCREW_PITCH 14.0f   //mm_per_rev
#define motor_1_8_DEGREE 200.0  //steps_per_rot
#define DEFAULT_MICROSTEPPING 16
#define encoder_PPR   2048

#define num_of_axis 1
#define encoder_dir CW
#define HOME_DIR LOW
#define motor_orientation HIGH

#endif
