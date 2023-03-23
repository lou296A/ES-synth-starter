#ifndef __ADSR_H__
#define __ADSR_H__

#include <Arduino.h>
#include <stdlib.h>


#define ADSR_IDLE      (0x00)
#define ADSR_DELAY     (0x01)
#define ADSR_ATACK     (0x02)
#define ADSR_DECAY     (0x03)
#define ADSR_SUSTAIN   (0x04)
#define ADSR_RELEASE   (0x05)
#define ADSR_FINISH    (0x06)

extern const uint32_t sampleRate;

class ADSR{
    private:

        /*struct COUNT{
            uint32_t DELAY_COUNT = 0;
            uint32_t ATACK_COUNT = 0;
            uint32_t DECAY_COUNT = 0;
            uint32_t SUSTAIN_COUNT = 0;
            uint32_t RELEASE_COUNT = 0;
        }; */

        //COUNT count;

        struct AMP_COUNT{
            //uint8_t AMP_DELAY_COUNT = 0;
            uint32_t AMP_ATACK_COUNT = 0;
            uint32_t AMP_DECAY_COUNT = 0;
            uint32_t AMP_SUSTAIN_COUNT = 0;
            uint32_t AMP_RELEASE_COUNT = 0;
        };

        AMP_COUNT ampcount;

        //uint32_t pre_phaseAcc = 0;
        
        uint32_t continue_phaseAcc = 0;
        uint32_t Amp = 0;
        uint32_t NUM_DELAY = 0;
        uint32_t NUM_ATACK = 0;
        uint32_t NUM_DECAY = 0;
        uint32_t NUM_SUSTAIN = 0;
        uint32_t NUM_RELEASE = 0; 
        uint32_t TOTAL_COUNT = 0;
        uint8_t STATE = ADSR_FINISH;
        volatile uint32_t preStepSize = 0;
        
        uint8_t finish_flag = 0;
    public:
        ADSR(float DelayTime, float AtackTime, float DecayTime, float SustainTime, float ReleaseTime);
       // ~ADSR();


        int32_t Do_ADSR(uint32_t phaseAcc, volatile uint32_t currentStepSize);
        //for testing
        uint32_t check;
        
        struct COUNT{
            uint32_t DELAY_COUNT = 0;
            uint32_t ATACK_COUNT = 0;
            uint32_t DECAY_COUNT = 0;
            uint32_t SUSTAIN_COUNT = 0;
            uint32_t RELEASE_COUNT = 0;
        }; 

        COUNT count;
        
        //volatile uint8_t Check(volatile uint32_t currentStepSize);

};





#endif
