#include "ADSR.h"

ADSR::ADSR(float DelayTime, float AtackTime, float DecayTime, float SustainTime, float ReleaseTime)
{
    NUM_DELAY = (uint32_t) (DelayTime*sampleRate);
    //NUM_ATACK = (uint32_t) (AtackTime*sampleRate)>>3;
    NUM_ATACK = (uint32_t) (AtackTime*sampleRate);
    //NUM_DECAY = (uint32_t) (DecayTime*sampleRate)>>3;
    NUM_DECAY = (uint32_t) (DecayTime*sampleRate);
    NUM_SUSTAIN = (uint32_t) (SustainTime*sampleRate);
    NUM_RELEASE = (uint32_t) (ReleaseTime*sampleRate);

} 

int32_t ADSR::Do_ADSR(uint32_t phaseAcc, volatile uint32_t currentStepSize)
{
    continue_phaseAcc = (currentStepSize == 0) ? (continue_phaseAcc += preStepSize) : phaseAcc;
    //STATE = (currentStepSize == 0) ? ADSR_IDLE : STATE;
    int32_t V_in = (continue_phaseAcc >> 24);
    int32_t V_out = 0;  


    if(STATE == ADSR_IDLE)
    {
        if(!NUM_DELAY)
        {
            STATE = ADSR_ATACK;  
        }
        else
        {
            STATE = ADSR_DELAY;
        }
        V_out = 0;
    }
    else if(STATE == ADSR_DELAY)
    {
        STATE = (count.DELAY_COUNT < NUM_DELAY) ? ADSR_DELAY : ADSR_ATACK;
        count.DELAY_COUNT += 1;
        V_out = 0;
    }
    else if(STATE == ADSR_ATACK)
    {
        count.ATACK_COUNT += 1;
        STATE = (count.ATACK_COUNT < NUM_ATACK) ? ADSR_ATACK : ADSR_DECAY;
        ampcount.AMP_ATACK_COUNT = count.ATACK_COUNT/(NUM_ATACK>>3);
        check = count.ATACK_COUNT/(NUM_ATACK/8);
        V_out = V_in - V_in>>(ampcount.AMP_ATACK_COUNT);
    }
    else if(STATE == ADSR_DECAY)
    {
        //V_out = V_in - V_in>>(ampcount.AMP_ATACK_COUNT);
        STATE = (count.DECAY_COUNT < NUM_DECAY) ? ADSR_DECAY : ADSR_SUSTAIN;
        ampcount.AMP_DECAY_COUNT = count.DECAY_COUNT/(NUM_DECAY>>2);
        count.DECAY_COUNT += 1;
        check = count.DECAY_COUNT/(NUM_DECAY/4);
        V_out = V_in>>(ampcount.AMP_DECAY_COUNT);
    }
    else if(STATE == ADSR_SUSTAIN)
    {
        //V_out = V_in - V_in>>(ampcount.AMP_ATACK_COUNT);
        V_out = V_in>>(ampcount.AMP_DECAY_COUNT);
        STATE = (count.SUSTAIN_COUNT < NUM_SUSTAIN) ? ADSR_SUSTAIN : ADSR_RELEASE;
        count.SUSTAIN_COUNT += 1;
        check = check;
        V_out = V_out;
    }
    else if(STATE == ADSR_RELEASE)
    {
        V_out = V_in>>(ampcount.AMP_DECAY_COUNT);
        STATE = (count.RELEASE_COUNT < NUM_RELEASE) ? ADSR_RELEASE : ADSR_FINISH;
        ampcount.AMP_RELEASE_COUNT = count.RELEASE_COUNT/(NUM_RELEASE>>2);
        count.RELEASE_COUNT += 1;
        check = count.RELEASE_COUNT/(NUM_RELEASE>>2);
        //finish_flag = (count.RELEASE_COUNT < NUM_RELEASE) ? 0 : 1;
        V_out = V_out>>(ampcount.AMP_RELEASE_COUNT);
    }
    else
    {
        count.DELAY_COUNT = 0;
        count.ATACK_COUNT = 0;
        count.DECAY_COUNT = 0;
        count.SUSTAIN_COUNT = 0;
        count.RELEASE_COUNT = 0;
        ampcount.AMP_ATACK_COUNT = 0;
        ampcount.AMP_DECAY_COUNT = 0;
        ampcount.AMP_SUSTAIN_COUNT = 0;
        ampcount.AMP_RELEASE_COUNT = 0;

        continue_phaseAcc = 0;
        if(!preStepSize)
        {
            STATE = (!currentStepSize) ? ADSR_FINISH : ADSR_IDLE;
        }
        else if((currentStepSize == preStepSize) && (currentStepSize != 0))
        {
            STATE = ADSR_FINISH;
        }
        else
        {
            STATE = ADSR_FINISH;
            preStepSize = 0;
        }
        
        

        V_out = 0;

    }
    preStepSize = (currentStepSize == 0) ? preStepSize : currentStepSize;
    //preStepSize = currentStepSize;

    return V_out;

} 
