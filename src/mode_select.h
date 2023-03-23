#ifndef __MODE_SELECT_H__
#define __MODE_SELECT_H__

#include <stm32l4xx_hal.h>
#include <stdlib.h>
#include <Arduino.h>

#define NORMAL_MODE  0
#define ADSR_MODE    1
#define FITER_MODE   2
#define ECHO_MODE    3
#define POLYPHONY    4

class mode_select{
    private:
        int32_t Joystick_Y;
        uint8_t Actual_mode = 0;
        uint8_t mode_display = 0;

};

#endif