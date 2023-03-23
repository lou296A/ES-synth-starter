#include "POLYPHONY.h"

POLYPHONY::POLYPHONY(void)
{
    for(int i = 0; i < 12; i++)
    {
        Poly_stepsize[i] = 0;
    }

}

void  POLYPHONY::Polyphony_mapStepsize(uint32_t keys)
{
    key_on_max = 0;
    uint32_t localStepSize = 0;
    for (int i = 0; i < 12; i++)
    {
        Poly_stepsize[i] = 0;
        if((keys & 0x1 << i) > 0)
        {
            key_on_max = key_on_max;
        }
        else
        {
            Poly_stepsize[key_on_max] = stepSizes[i];
            key_on_max += 1;
        }
        localStepSize = ((keys & 0x1 << i) > 0) ? localStepSize : stepSizes[i];
        zero_or_not = localStepSize;
    }
}

uint32_t POLYPHONY::do_Polyphony(void)
{
    if(zero_or_not != 0)
    {
    if(time_step_count == MAX_TIME_STEP)
    {
        time_step_count = 0;
        choose_size_count += 1;
        if(Poly_stepsize[choose_size_count] == 0)
        {
            choose_size_count = 0;
            return Poly_stepsize[choose_size_count];
        }
        else
        {
            return Poly_stepsize[choose_size_count];
        }
    }
    else
    {
        choose_size_count = choose_size_count;
        time_step_count += 1;
        return Poly_stepsize[choose_size_count];
    }
    }
    else{
        return 0;
    }
}