#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <string>
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
using namespace std;

float parse_numeric_value();
uint32_t pwm_set_freq_duty(uint pwm_num, uint32_t frequency, int d);
void printCommandlist();
bool is_only_alphabets(string& str);


#endif