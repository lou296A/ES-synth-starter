#ifndef __FILTER_H__
#define __FILTER_H__

#include <arm_math.h>
#include <stm32l4xx_hal_cortex.h>
#include <stm32l4xx_hal_rcc.h>
#include <stm32l4xx_hal.h>

#define numStages 2
extern const uint32_t sampleRate;

class FILTER{

    private:
        float32_t IIRStateF32_LP_1[4*numStages];
        float32_t IIRStateF32_HP_1[4*numStages];
        float32_t IIRStateF32_LP_2[4*numStages];
        float32_t IIRStateF32_HP_2[4*numStages];
        float32_t IIRStateF32_LP_3[4*numStages];
        float32_t IIRStateF32_HP_3[4*numStages];
        float32_t IIRStateF32_LP_4[4*numStages];
        float32_t IIRStateF32_HP_4[4*numStages];
        float32_t IIRStateF32_LP_5[4*numStages];
        float32_t IIRStateF32_HP_5[4*numStages];
        float32_t IIRStateF32_LP_6[4*numStages];
        float32_t IIRStateF32_HP_6[4*numStages];
        float32_t IIRStateF32_LP_7[4*numStages];
        float32_t IIRStateF32_HP_7[4*numStages];
        float32_t IIRStateF32_LP_8[4*numStages];
        float32_t IIRStateF32_HP_8[4*numStages];

        arm_biquad_casd_df1_inst_f32 S_LP_1;
        arm_biquad_casd_df1_inst_f32 S_HP_1;
        arm_biquad_casd_df1_inst_f32 S_LP_2;
        arm_biquad_casd_df1_inst_f32 S_HP_2;
        arm_biquad_casd_df1_inst_f32 S_LP_3;
        arm_biquad_casd_df1_inst_f32 S_HP_3;
        arm_biquad_casd_df1_inst_f32 S_LP_4;
        arm_biquad_casd_df1_inst_f32 S_HP_4;
        arm_biquad_casd_df1_inst_f32 S_LP_5;
        arm_biquad_casd_df1_inst_f32 S_HP_5;
        arm_biquad_casd_df1_inst_f32 S_LP_6;
        arm_biquad_casd_df1_inst_f32 S_HP_6;
        arm_biquad_casd_df1_inst_f32 S_LP_7;
        arm_biquad_casd_df1_inst_f32 S_HP_7;
        arm_biquad_casd_df1_inst_f32 S_LP_8;
        arm_biquad_casd_df1_inst_f32 S_HP_8;

        // Define the coefficients for knob rotation 1
        const float32_t IIRCoeffs32LP_1[5*numStages] = {
            1.0f, 2.0f, 1.0f, 1.9039099239599475f, -0.9164015363523371f,
            1.0f, 2.0f, 1.0f, 1.7976456252351991f, -0.8094400342493649f
        };
        const float32_t IIRCoeffs32HP_1[5*numStages] = {
            1.0f, -2.0f, 1.0f, 1.9039099239599477f, -0.9164015363523372f,
            1.0f, -2.0f, 1.0f, 1.7976456252351991f, -0.8094400342493647f
        };
        // Define the coefficients for knob rotation 2
        const float32_t IIRCoeffs32LP_2[5*numStages] = {
            1.0f, 2.0f, 1.0f, 1.7926436484531221f, -0.8404740406880010f,
            1.0f, 2.0f, 1.0f, 1.6109279364286584f, -0.6539098838603158f
        };
        const float32_t IIRCoeffs32HP_2[5*numStages] = {
            1.0f, -2.0f, 1.0f, 1.7926436484531223f, -0.8404740406880010f,
            1.0f, -2.0f, 1.0f, 1.6109279364286584f, -0.6539098838603159f
        };
        // Define the coefficients for knob rotation 3
        const float32_t IIRCoeffs32LP_3[5*numStages] = {
            1.0f, 2.0f, 1.0f, 1.6690482313821227f, -0.7721061948414686f,
            1.0f, 2.0f, 1.0f, 1.4374155379520059f, -0.5261709826424708f
        };
        const float32_t IIRCoeffs32HP_3[5*numStages] = {
            1.0f, -2.0f, 1.0f, 1.6690482313821224f, -0.7721061948414685f,
            1.0f, -2.0f, 1.0f, 1.4374155379520059f, -0.5261709826424708f
        };
        // Define the coefficients for knob rotation 4
        const float32_t IIRCoeffs32LP_4[5*numStages] = {
            1.0f, 2.0f, 1.0f, 1.5355263036814090f, -0.7110864634793341f,
            1.0f, 2.0f, 1.0f, 1.2750440794802564f, -0.4208227234580789f
        };
        const float32_t IIRCoeffs32HP_4[5*numStages] = {
            1.0f, -2.0f, 1.0f, 1.5355263036814082f, -0.7110864634793342f,
            1.0f, -2.0f, 1.0f, 1.2750440794802564f, -0.4208227234580789f
        };
        // Define the coefficients for knob rotation 5
        const float32_t IIRCoeffs32LP_5[5*numStages] = {
            1.0f, 2.0f, 1.0f, 1.3940799027595556f, -0.6571459712839184f,
            1.0f, 2.0f, 1.0f, 1.1220551306692834f, -0.3337895020697072f
        };
        const float32_t IIRCoeffs32HP_5[5*numStages] = {
            1.0f, -2.0f, 1.0f, 1.3940799027595554f, -0.6571459712839184f,
            1.0f, -2.0f, 1.0f, 1.1220551306692832f, -0.3337895020697072f
        };
        // Define the coefficients for knob rotation 6
        const float32_t IIRCoeffs32LP_6[5*numStages] = {
            1.0f, 2.0f, 1.0f, 1.2463606150037343f, -0.6099904692822512f,
            1.0f, 2.0f, 1.0f, 0.9769413173436217f, -0.2619671955588596f
        };
        const float32_t IIRCoeffs32HP_6[5*numStages] = {
            1.0f, -2.0f, 1.0f, 1.2463606150037341f, -0.6099904692822512f,
            1.0f, -2.0f, 1.0f, 0.9769413173436214f, -0.2619671955588595f
        };
        // Define the coefficients for knob rotation 7
        const float32_t IIRCoeffs32LP_7[5*numStages] = {
            1.0f, 2.0f, 1.0f, 1.0937205329178585f, -0.5693235407414439f,
            1.0f, 2.0f, 1.0f, 0.8383992503414319f, -0.2029761172999780f
        };
        const float32_t IIRCoeffs32HP_7[5*numStages] = {
            1.0f, -2.0f, 1.0f, 1.0937205329178581f, -0.5693235407414438f,
            1.0f, -2.0f, 1.0f, 0.8383992503414317f, -0.2029761172999779f
        };
        // Define the coefficients for knob rotation 8
        const float32_t IIRCoeffs32LP_8[5*numStages] = {
            1.0f, 2.0f, 1.0f, 0.9372608541822494f, -0.5348630289878381f,
            1.0f, 2.0f, 1.0f, 0.7052899328356489f, -0.1549862962870072f
        };
        const float32_t IIRCoeffs32HP_8[5*numStages] = {
            1.0f, -2.0f, 1.0f, 0.9372608541822495f, -0.5348630289878381f,
            1.0f, -2.0f, 1.0f, 0.7052899328356491f, -0.1549862962870073f
        };


        const float32_t ScaleValue_LP[8] = {
            0.0031229030980974f*0.0029486022535414f,
            0.0119575980587197f*0.0107454868579144f,
            0.0257644908648365f*0.0221888611726162f,
            0.0438900399494816f*0.0364446609944556f,
            0.0657665171310907f*0.0529335928501060f,
            0.0909074635696293f*0.0712564695538095f,
            0.1189007519558964f*0.0911442167396365f,
            0.1494005437013972f*0.1124240908628395f
        };
        const float32_t ScaleValue_HP[8] = {
            0.9550778650780712f*0.9017714148711410f,
            0.9082794222852808f*0.8162094550722436f,
            0.8602886065558977f*0.7408966301486191f,
            0.8116531917901857f*0.6739667007345839f,
            0.7628064685108684f*0.6139611581847476f,
            0.7140877710714963f*0.5597271282256202f,
            0.6657610184148254f*0.5103438419103524f,
            0.6180309707925219f*0.4650690572806641f
        };

        arm_biquad_casd_df1_inst_f32 *ADDRESS_LP[8];
        arm_biquad_casd_df1_inst_f32 *ADDRESS_HP[8];

        float32_t INPUT_LP;
        float32_t OUTPUT_LP;

        float32_t INPUT_HP;
        float32_t OUTPUT_HP;

    public:
        FILTER();
        int32_t LowPassFilter(int32_t input, uint8_t knob);
        int32_t HighPassFilter(int32_t input, uint8_t knob);
        uint8_t check;

};


#endif

