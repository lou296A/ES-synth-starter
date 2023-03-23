#ifndef __POLYPHONY_H__
#define __POLYPHONY_H__

#include <stm32l4xx_hal.h>
#include <stdlib.h>
#include <Arduino.h>

#define MAX_TIME_STEP 5

extern const std::array<uint32_t, 12> stepSizes;

class POLYPHONY{
    private:
    uint32_t Poly_stepsize[12];
    uint8_t key_on_max = 0;
    uint8_t time_step_count = 0;
    uint8_t choose_size_count = 0;
    uint32_t zero_or_not;
    public:
    POLYPHONY();
    void Polyphony_mapStepsize(uint32_t keys);
    uint32_t do_Polyphony(void);
};


#endif