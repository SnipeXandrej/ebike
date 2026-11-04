/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    portab.c
 * @brief   Application portability module code.
 *
 * @addtogroup application_portability
 * @{
 */

#include "hal.h"
#include "portab.h"

/*
 * GPT configuration.
 */

void timer_2khz_callback(GPTDriver *gptp);
const GPTConfig timer_2khz_config = {
    .frequency = 1000000U,
    .callback  = timer_2khz_callback,
    .cr2       = TIM_CR2_MMS_1, // TRGO on timer update.
    .dier      = 0U
};

const GPTConfig portab_gptcfg1 = {
  .frequency    =  20000U,
  .callback     =  NULL,
  .cr2          =  TIM_CR2_MMS_1,   /* MMS = 010 = TRGO on Update Event.    */
  .dier         =  0U
};

const ADCConfig portab_adccfg1 = {
  .difsel       = 0U,
  .calibration  = ADC_CR_ADCALLIN
};

void adccallback(ADCDriver *adcp);

/*
 * ADC errors callback, should never happen.
 */
void adcerrorcallback(ADCDriver *adcp, adcerror_t err);

// /*
//  * ADC conversion group 1.
//  * Mode:        One shot, 2 channels, SW triggered.
//  * Channels:    IN0, IN5.
//  */
// const ADCConversionGroup portab_adcgrpcfg1 = {
//   .circular     = false,
//   .num_channels = ADC_GRP1_NUM_CHANNELS,
//   .end_cb       = NULL,
//   .error_cb     = adcerrorcallback,
//   .cfgr         = 0U,
//   .cfgr2        = 0U,
//   .ccr          = 0U,
//   .pcsel        = ADC_SELMASK_IN5 | ADC_SELMASK_IN16,
//   .ltr1         = 0x00000000U,
//   .htr1         = 0x03FFFFFFU,
//   .ltr2         = 0x00000000U,
//   .htr2         = 0x03FFFFFFU,
//   .ltr3         = 0x00000000U,
//   .htr3         = 0x03FFFFFFU,
//   .smpr         = {
//     ADC_SMPR1_SMP_AN5(ADC_SMPR_SMP_384P5),
//     ADC_SMPR2_SMP_AN16(ADC_SMPR_SMP_384P5)
//   },
//   .sqr          = {
//     ADC_SQR1_SQ1_N(ADC_CHANNEL_IN5) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN16),
//     0U,
//     0U,
//     0U
//   }
// };

// code for use in main.c
// #define ADC_GRP1_BUF_DEPTH      1
// #if CACHE_LINE_SIZE > 0
// CC_ALIGN_DATA(CACHE_LINE_SIZE)
// #endif
// adcsample_t samples1[CACHE_SIZE_ALIGN(adcsample_t, ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH)];
// /* Performing a one-shot conversion on two channels.*/
// adcConvert(&PORTAB_ADC1, &portab_adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
// cacheBufferInvalidate(samples1, sizeof (samples1) / sizeof (adcsample_t));

/*
 * ADC conversion group 2.
 * Mode:        Continuous, 2 channels, HW triggered by GPT4-TRGO.
 * Channels:    IN0, IN5.
 */
const ADCConversionGroup portab_adcgrpcfg1 = {
  .circular     = true,
  .num_channels = ADC_GRP2_NUM_CHANNELS,
  .end_cb       = adccallback,
  .error_cb     = adcerrorcallback,
  .cfgr         = ADC_CFGR_CONT_DISABLED | ADC_CFGR_EXTEN_RISING | ADC_CFGR_EXTSEL_SRC(13), // TIM6_TRGO is defined as 13 in ST's HAL ADC driver
  .cfgr2        = 0U,
  .ccr          = 0U,
  .pcsel        = ADC_SELMASK_IN16 | ADC_SELMASK_IN17 | ADC_SELMASK_IN14 | ADC_SELMASK_IN15,
  .ltr1         = 0x00000000U,
  .htr1         = 0x03FFFFFFU,
  .ltr2         = 0x00000000U,
  .htr2         = 0x03FFFFFFU,
  .ltr3         = 0x00000000U,
  .htr3         = 0x03FFFFFFU,
  .smpr         = {
    0U,
    ADC_SMPR2_SMP_AN16(ADC_SMPR_SMP_64P5) |
    ADC_SMPR2_SMP_AN17(ADC_SMPR_SMP_64P5) |
    ADC_SMPR2_SMP_AN14(ADC_SMPR_SMP_64P5) |
    ADC_SMPR2_SMP_AN15(ADC_SMPR_SMP_64P5)
  },
  .sqr          = {
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN16) |
    ADC_SQR1_SQ2_N(ADC_CHANNEL_IN17) |
    ADC_SQR1_SQ3_N(ADC_CHANNEL_IN14) |
    ADC_SQR1_SQ4_N(ADC_CHANNEL_IN15),
    0U,
    0U,
    0U
  }
};

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void portab_setup(void) {
    /* ADC inputs.*/
    // in stm32h723 datasheet at page 60
    palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
}

/** @} */
