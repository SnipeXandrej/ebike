// CLANKED

#include "adc3.hpp"

static bool adc3_wait(volatile uint32_t *reg, uint32_t mask, uint32_t expected) {
  systime_t started;

  started = chVTGetSystemTimeX();
  while ((*reg & mask) != expected) {
    if (chVTTimeElapsedSinceX(started) >= TIME_MS2I(ADC3_TIMEOUT_MS)) {
      return false;
    }
    chThdSleepMilliseconds(ADC3_POLL_INTERVAL_MS);
  }
  return true;
}

bool adc3_internal_start(void) {
  rccEnableADC3(false);
  rccResetADC3();

  /* Asynchronous /4, temperature sensor and VREFINT enabled. No DMA/IRQ. */
  ADC3_COMMON->CCR = ADC3_CLOCK_ASYNC_DIV4 | ADC_CCR_TSEN | ADC_CCR_VREFEN;
  ADC3->CR = ADC_CR_ADVREGEN;
  chThdSleepMilliseconds(ADC3_STARTUP_DELAY_MS); /* Regulator and internal sensors stabilize. */

  /* Hardware single-ended offset calibration, without software correction. */
  ADC3->LTR2_DIFSEL = ADC3_SINGLE_ENDED;
  ADC3->CR |= ADC_CR_ADCAL;
  if (!adc3_wait(&ADC3->CR, ADC_CR_ADCAL, 0U)) {
    return false;
  }
  chThdSleepMilliseconds(ADC3_POST_CALIBRATION_DELAY_MS); /* More than four ADC clocks after calibration. */

  /* ADC3 RES=00 is 12-bit, right aligned, software-triggered, one shot. */
  ADC3->CFGR = ADC3_RESOLUTION_12BIT;
  ADC3->CFGR2 = ADC3_OVERSAMPLING_DISABLED;
  ADC3->SMPR2 = (ADC3_SAMPLE_640P5_CYCLES << ADC_SMPR2_SMP17_Pos) |
                (ADC3_SAMPLE_640P5_CYCLES << ADC_SMPR2_SMP18_Pos);
  ADC3->ISR = ADC_ISR_ADRDY;
  ADC3->CR |= ADC_CR_ADEN;
  return adc3_wait(&ADC3->ISR, ADC_ISR_ADRDY, ADC_ISR_ADRDY);
}

bool adc3_read_raw(uint32_t channel, uint32_t *raw) {
  /* One conversion per sequence; no PCSEL register on this ADC. */
  ADC3->SQR1 = channel << ADC_SQR1_SQ1_Pos;
  ADC3->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
  ADC3->CR |= ADC_CR_ADSTART;
  if (!adc3_wait(&ADC3->ISR, ADC_ISR_EOS, ADC_ISR_EOS) ||
      !adc3_wait(&ADC3->CR, ADC_CR_ADSTART, 0U)) {
    return false;
  }
  *raw = ADC3->DR & ADC3_FULL_SCALE;
  return true;
}

bool adc3_calculate(uint32_t vrefint_raw, uint32_t temperature_raw, uint32_t *reference_mv, uint32_t *vrefint_mv, int32_t *temperature_c) {
  uint32_t vref_cal, ts_cal1, ts_cal2;
  int64_t numerator, denominator;

  vref_cal = ADC3_VREFINT_CAL;
  ts_cal1 = ADC3_TS_CAL1;
  ts_cal2 = ADC3_TS_CAL2;
  if ((vrefint_raw == 0U) || (vrefint_raw > ADC3_FULL_SCALE) ||
      (temperature_raw > ADC3_FULL_SCALE) || (vref_cal == 0U) ||
      (vref_cal > ADC3_FULL_SCALE) || (ts_cal1 == 0U) ||
      (ts_cal2 > ADC3_FULL_SCALE) || (ts_cal2 <= ts_cal1)) {
    return false;
  }

  *reference_mv = (ADC3_CAL_REFERENCE_MV * vref_cal + vrefint_raw / 2U) / vrefint_raw;
  *vrefint_mv = (ADC3_CAL_REFERENCE_MV * vref_cal + (ADC3_FULL_SCALE / 2U)) / ADC3_FULL_SCALE;

  /* Normalize TS counts to the factory 3.3 V reference before interpolation.
   * Preserve the ratio until the final division, including below-zero temps.
   */
  numerator = ((int64_t)temperature_raw * vref_cal - (int64_t)ts_cal1 * vrefint_raw) *
              (ADC3_TS_CAL2_C - ADC3_TS_CAL1_C) * ADC3_CENTI_C_PER_C;
  denominator = (int64_t)vrefint_raw * (ts_cal2 - ts_cal1);
  *temperature_c = ADC3_TS_CAL1_C * ADC3_CENTI_C_PER_C + (int32_t)(numerator / denominator);
  return true;
}