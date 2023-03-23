#include "FILTER.h"

FILTER::FILTER(void)
{
    //__HAL_RCC_DSP_CLK_ENABLE();
    arm_biquad_cascade_df1_init_f32(&S_LP_1, numStages, (float32_t *)&IIRCoeffs32LP_1, (float32_t *)&IIRStateF32_LP_1);
    arm_biquad_cascade_df1_init_f32(&S_HP_1, numStages, (float32_t *)&IIRCoeffs32HP_1, (float32_t *)&IIRStateF32_HP_1);
    arm_biquad_cascade_df1_init_f32(&S_LP_2, numStages, (float32_t *)&IIRCoeffs32LP_2, (float32_t *)&IIRStateF32_LP_2);
    arm_biquad_cascade_df1_init_f32(&S_HP_2, numStages, (float32_t *)&IIRCoeffs32HP_2, (float32_t *)&IIRStateF32_HP_2);
    arm_biquad_cascade_df1_init_f32(&S_LP_3, numStages, (float32_t *)&IIRCoeffs32LP_3, (float32_t *)&IIRStateF32_LP_3);
    arm_biquad_cascade_df1_init_f32(&S_HP_3, numStages, (float32_t *)&IIRCoeffs32HP_3, (float32_t *)&IIRStateF32_HP_3);
    arm_biquad_cascade_df1_init_f32(&S_LP_4, numStages, (float32_t *)&IIRCoeffs32LP_4, (float32_t *)&IIRStateF32_LP_4);
    arm_biquad_cascade_df1_init_f32(&S_HP_4, numStages, (float32_t *)&IIRCoeffs32HP_4, (float32_t *)&IIRStateF32_HP_4);
    arm_biquad_cascade_df1_init_f32(&S_LP_5, numStages, (float32_t *)&IIRCoeffs32LP_5, (float32_t *)&IIRStateF32_LP_5);
    arm_biquad_cascade_df1_init_f32(&S_HP_5, numStages, (float32_t *)&IIRCoeffs32HP_5, (float32_t *)&IIRStateF32_HP_5);
    arm_biquad_cascade_df1_init_f32(&S_LP_6, numStages, (float32_t *)&IIRCoeffs32LP_6, (float32_t *)&IIRStateF32_LP_6);
    arm_biquad_cascade_df1_init_f32(&S_HP_6, numStages, (float32_t *)&IIRCoeffs32HP_6, (float32_t *)&IIRStateF32_HP_6);
    arm_biquad_cascade_df1_init_f32(&S_LP_7, numStages, (float32_t *)&IIRCoeffs32LP_7, (float32_t *)&IIRStateF32_LP_7);
    arm_biquad_cascade_df1_init_f32(&S_HP_7, numStages, (float32_t *)&IIRCoeffs32HP_7, (float32_t *)&IIRStateF32_HP_7);
    arm_biquad_cascade_df1_init_f32(&S_LP_8, numStages, (float32_t *)&IIRCoeffs32LP_8, (float32_t *)&IIRStateF32_LP_8);
    arm_biquad_cascade_df1_init_f32(&S_HP_8, numStages, (float32_t *)&IIRCoeffs32HP_8, (float32_t *)&IIRStateF32_HP_8);

    ADDRESS_LP[0] = &S_LP_1;
    ADDRESS_HP[0] = &S_HP_1;
    ADDRESS_LP[1] = &S_LP_2;
    ADDRESS_HP[1] = &S_HP_2;
    ADDRESS_LP[2] = &S_LP_3;
    ADDRESS_HP[2] = &S_HP_3;
    ADDRESS_LP[3] = &S_LP_4;
    ADDRESS_HP[3] = &S_HP_4;
    ADDRESS_LP[4] = &S_LP_5;
    ADDRESS_HP[4] = &S_HP_5;
    ADDRESS_LP[5] = &S_LP_6;
    ADDRESS_HP[5] = &S_HP_6;
    ADDRESS_LP[6] = &S_LP_7;
    ADDRESS_HP[6] = &S_HP_7;
    ADDRESS_LP[7] = &S_LP_8;
    ADDRESS_HP[7] = &S_HP_8;

}

int32_t FILTER::LowPassFilter(int32_t input, uint8_t knob)
{
    float32_t output = 0;
    INPUT_LP = input;
    if(!knob)
    {
        return output = input;
    }
    else
    {
        arm_biquad_cascade_df1_f32(ADDRESS_LP[8 - (int)knob], &INPUT_LP, &OUTPUT_LP, 1);
        check = (int)knob-1;
        return output = (int32_t) (OUTPUT_LP*ScaleValue_LP[8 - (int)knob]);
    }
    
    
}

int32_t FILTER::HighPassFilter(int32_t input, uint8_t knob)
{
    float32_t output = 0;
    INPUT_HP = input;
    if(!knob)
    {
        return output = input;
    }
    else
    {
        arm_biquad_cascade_df1_f32(ADDRESS_HP[(int)knob-1], &INPUT_HP, &OUTPUT_HP, 1);
        return output = (int32_t) (OUTPUT_HP*ScaleValue_HP[(int)knob-1]);
    }
    
}