#include "ECHO.h"

Echo::Echo(void)
{
    //FIFO = xQueueCreate(FIFO_length,4);
    //Max_Length = FIFO_length;
    for(int i=0; i<SAMPLESIZE; i++)
    {
        FIFO0[i] = 0;
        FIFO1[i] = 0;
    }
}

int32_t Echo::do_Echo(int32_t V_in, uint32_t phaseAcc)
{
    Element_in = V_in;
    //xQueueReceive(FIFO, &Element_out, 0);
    if((!prephase) && (!phaseAcc))
    {
        return 0;
    }
    else{
    if(index_read == SAMPLESIZE)
    {
        index_read = 0;
        index_write = 0;
        writeFIFO1 = !writeFIFO1;
    }
    if(!writeFIFO1)
    {
        Element_out = FIFO1[index_read];
        Element_out = V_in + ((Element_out == 0) ? 0 : (Element_out >> 1));
        FIFO0[index_write] = Element_out;
        index_read = index_read + 1;
        index_write = index_write + 1;
        prephase = phaseAcc;
        return Element_out;
    }
    else
    {
        Element_out = FIFO0[index_read];
        Element_out = V_in + ((Element_out == 0) ? 0 : (Element_out >> 1));
        FIFO1[index_write] = Element_out;
        index_read = index_read + 1;
        index_write = index_write + 1;
        prephase = phaseAcc;
        return Element_out;
    }
    }
    
    //xQueueSendFromISR(FIFO, &Element_in, NULL);
    
}