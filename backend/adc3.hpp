// CLANKED

#ifndef APP_ADC3_HPP
#define APP_ADC3_HPP

#include "ch.h"
#include "hal.h"

/* STM32H723 ADC3 settings (this ADC differs from ADC1/2). */
#define ADC3_CLOCK_DIVISOR                  4U
#define ADC3_CLOCK_ASYNC_DIV4               (2U << ADC_CCR_PRESC_Pos)
#define ADC3_CLOCK_MIN_HZ                   1500000U
#define ADC3_CLOCK_MAX_HZ                   50000000U
#define ADC3_SAMPLE_640P5_CYCLES            7U
#define ADC3_CHANNEL_TEMPERATURE            17U
#define ADC3_CHANNEL_VREFINT                18U
#define ADC3_RESOLUTION_12BIT               0U
#define ADC3_FULL_SCALE                     4095U
#define ADC3_SINGLE_ENDED                   0U
#define ADC3_OVERSAMPLING_DISABLED          0U
#define ADC3_TIMEOUT_MS                     20U
#define ADC3_POLL_INTERVAL_MS               1U
#define ADC3_STARTUP_DELAY_MS               1U
#define ADC3_POST_CALIBRATION_DELAY_MS      1U
#define ADC3_CAL_REFERENCE_MV               3300U
#define ADC3_TS_CAL1_C                      30
#define ADC3_TS_CAL2_C                      130
#define ADC3_CENTI_C_PER_C                  100

/* H723 factory calibration uses 12-bit counts at VREF+=3.3 V.
 * These 0x values represent memory addresses
 */
#define ADC3_VREFINT_CAL (*(const uint16_t *)0x1FF1E860UL)
#define ADC3_TS_CAL1     (*(const uint16_t *)0x1FF1E820UL)
#define ADC3_TS_CAL2     (*(const uint16_t *)0x1FF1E840UL)

#if (STM32_ADCCLK / ADC3_CLOCK_DIVISOR < ADC3_CLOCK_MIN_HZ) || (STM32_ADCCLK / ADC3_CLOCK_DIVISOR > ADC3_CLOCK_MAX_HZ)
#error "Adjust the ADC3 polling prescaler for the configured ADC kernel clock"
#endif

bool adc3_internal_start(void);

bool adc3_read_raw(uint32_t channel, uint32_t *raw);

bool adc3_calculate(uint32_t vrefint_raw, uint32_t temperature_raw, uint32_t *reference_mv, uint32_t *vrefint_mv, int32_t *temperature_c);

#endif
