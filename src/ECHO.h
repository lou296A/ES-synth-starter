#ifndef __ECHO_H__
#define __ECHO_H__

#include <STM32FreeRTOS.h>


using namespace std;

#define SAMPLESIZE 3500


class Echo{
    private:
        //QueueHandle_t FIFO;
        uint32_t Max_Length;
        
        
    public:
        Echo(void);
        int32_t do_Echo(int32_t V_in, uint32_t phaseAcc);
        int32_t FIFO0[SAMPLESIZE];
        int32_t FIFO1[SAMPLESIZE];
        //int32_t* FIFO1 = FIFO0 + SAMPLESIZE;
        int writeFIFO1 = 0;
        int index_write = 0;
        int index_read = 0;
        int32_t Element_out;
        uint32_t prephase = 0;
        int32_t Element_in;

};



#endif