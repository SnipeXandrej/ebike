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

#include "ch.h"
#include "hal.h"
#include "portab.h"
#include "usbcfg.h"
#include "chprintf.h"

#include "adc3.hpp"
#include "abstractions.hpp"
#include "commonUtils.hpp"
#include "myUart.hpp"
#include "VescUart/VescUart.h"

#include <string>

MyUart uartVesc;
VescUart VESC;

#define PIN_LED PAL_LINE(GPIOE, 3)
#define PIN_BUTTON_K1 PAL_LINE(GPIOC, 13)

/* Testing in progress.*/
static volatile bool testing = false;
#define ADC_FULL_SCALE 65535U

// ADC12
static volatile double voltages_mv[ADC_CHANNELS] = {};
static volatile double voltage = 0.0f;
static volatile double current = 0.0f;
static volatile double current_2 = 0.0f;
static volatile double current_ah = 0.0f;
static volatile double current_amp_seconds = 0.0;

// clock
static volatile uint32_t uptime_seconds = 0; // can count up to 136.19 years, I think thats enough
static volatile uint32_t clockSecondsSinceBoot = 0;
static volatile uint32_t clockMinutesSinceBoot = 0;
static volatile uint32_t clockHoursSinceBoot   = 0;
static volatile uint32_t clockDaysSinceBoot    = 0;

size_t n= 0, nx = 0, ny = 0, nxx = 0;
static volatile uint32_t cycles_per_second = 0;

// ADC3
static uint32_t vrefint_raw, temperature_raw;
static uint32_t reference_mv, vrefint_mv;
static int32_t temperature_c;
static volatile bool adc3_ok;

static THD_WORKING_AREA(waCalibrationThread, 768);
static THD_FUNCTION(CalibrationThread, arg) {
  bool adc3_ready = false, adc3_cal_ok;

  (void)arg;
  chRegSetThreadName("CalibrationThread");

  MovingAverage movingAverage_vrefint_raw;
  MovingAverage movingAverage_temperature_raw;
  movingAverage_vrefint_raw.smoothingFactor = 0.3f;
  movingAverage_temperature_raw.smoothingFactor = 0.3f;

  uint32_t _vrefint_raw = 0;
  uint32_t _temperature_raw = 0;

  while (true) {
    if (!adc3_ready) {
      adc3_ready = adc3_internal_start();
    }

    adc3_ok = adc3_ready && adc3_read_raw(ADC3_CHANNEL_VREFINT, &_vrefint_raw) && adc3_read_raw(ADC3_CHANNEL_TEMPERATURE, &_temperature_raw);

    if (!adc3_ok) {
      adc3_ready = false;
      vrefint_raw = 0U;
      temperature_raw = 0U;
    }

    movingAverage_vrefint_raw.initInput(_vrefint_raw);
    vrefint_raw = movingAverage_vrefint_raw.moveAverage(_vrefint_raw);
    movingAverage_temperature_raw.initInput(_temperature_raw);
    temperature_raw = movingAverage_temperature_raw.moveAverage(_temperature_raw);

    adc3_cal_ok = adc3_ok && adc3_calculate(vrefint_raw, temperature_raw, &reference_mv, &vrefint_mv, &temperature_c);

    if (!adc3_cal_ok) {
      chprintf((BaseSequentialStream*)&SDU1, "ADC3 voltage/temperature unavailable\r\n");
    }
    nxx++;
  }
}

static THD_WORKING_AREA(waUptimeCountingThread, 256);
static THD_FUNCTION(UptimeCountingThread, arg) {
  (void)arg;
  systime_t prev = chVTGetSystemTimeX();

  while (true) {
      systime_t next = chTimeAddX(prev, TIME_MS2I(1000));

      uptime_seconds = uptime_seconds + 1;
      clockSecondsSinceBoot = uptime_seconds % 60;
      clockMinutesSinceBoot = (uint32_t)(uptime_seconds / 60.0f) % 60;
      clockHoursSinceBoot   = (uint32_t)(uptime_seconds / 60.0f / 60.0f) % 24;
      clockDaysSinceBoot    = uptime_seconds / 60.0f / 60.0f / 24;

      prev = chThdSleepUntilWindowed(prev, next);
  }
}

static THD_WORKING_AREA(waLEDIntegratedThread, 256);
static THD_FUNCTION(LEDIntegratedThread, arg) {
  (void)arg;
  chRegSetThreadName("blinker");

  pinMode(PIN_LED, OUTPUT);

  while (true) {
    digitalToggle(PIN_LED);
    testing = digitalRead(PIN_BUTTON_K1);
    chThdSleepMilliseconds(testing ? 100 : 500);
  }
}

static THD_WORKING_AREA(waUSBSerialThread, 768);
static THD_FUNCTION(USBSerialThread, arg) {
  (void)arg;
  chRegSetThreadName("USBSerialThread");

  while (true) {
    chprintf((BaseSequentialStream*)&SDU1, "\n\rtesting value: %d, n: %d, ny: %d, nx: %d, adc3_ok: %d, vrefint_raw: %u, temperature_raw: %u, VREFINT_CAL: %u, NXX: %d\r\n", testing, n, ny, nx, adc3_ok, (unsigned)vrefint_raw, (unsigned)temperature_raw, (unsigned)ADC3_VREFINT_CAL, nxx);
    chprintf((BaseSequentialStream*)&SDU1,
              "vrefplus: %.3f V, vrefint: %.3f V, temperature: %.2f C, voltage_A0: %.2f, voltage_A1: %.2f, voltage_A2: %.2f, voltage_A3: %.2f, current: %.5f, current_2: %.5f, current_ah: %.10f, v_calc: %.3f, cycles_per_second: %d, calc: %f\r\n",
              (float)reference_mv / 1000.0f,
              (float)vrefint_mv / 1000.0f,
              (float)temperature_c / 100.0f,
              voltages_mv[0] / 1000.0,
              voltages_mv[1] / 1000.0,
              voltages_mv[2] / 1000.0,
              voltages_mv[3] / 1000.0,
              current,
              current_2,
              current_ah,
              voltage,
              cycles_per_second,
            0.005f/0.0025f);
    chprintf((BaseSequentialStream*)&SDU1,
              "Uptime: %02d:%02d:%02d and %01d days, raw seconds: %d\r\n",
              clockHoursSinceBoot,
              clockMinutesSinceBoot,
              clockSecondsSinceBoot,
              clockDaysSinceBoot,
              uptime_seconds);

    chThdSleepMilliseconds(100);

    std::string test = "tetstst";
    chprintf((BaseSequentialStream*)&SDU1, "String: %s\r\n", test.c_str());


    VESC.setCurrent(1.0f);
  }
}

/*
 * ADC using DMA.
 */
#define ADC_GRP2_BUF_DEPTH      8

/* Buffers are allocated with size and address aligned to the cache line size.*/
#if CACHE_LINE_SIZE > 0
CC_ALIGN_DATA(CACHE_LINE_SIZE)
#endif
adcsample_t samples[CACHE_SIZE_ALIGN(adcsample_t, ADC_GRP2_NUM_CHANNELS * ADC_GRP2_BUF_DEPTH)];

MovingAverage movingAverage_current;
MovingAverage movingAverage_current_2;

// this callback is partially clanked
extern "C" void adccallback(ADCDriver *adcp) {
  movingAverage_current.smoothingFactor = 0.0002f;
  movingAverage_current_2.smoothingFactor = movingAverage_current.smoothingFactor;


  const size_t sequences = ADC_GRP2_BUF_DEPTH / 2U;
  const size_t half_samples = sequences * ADC_GRP2_NUM_CHANNELS;

  adcsample_t *completed = adcIsBufferComplete(adcp) ? &samples[half_samples] : &samples[0];

  cacheBufferInvalidate(completed, half_samples * sizeof(*completed));

  uint32_t sums[ADC_CHANNELS] = {};
  float raw_values[ADC_CHANNELS] = {};

  for (size_t i = 0; i < half_samples; i += ADC_GRP2_NUM_CHANNELS) {
    for (size_t channel = 0; channel < ADC_CHANNELS; channel++) {
      sums[channel] += completed[i + channel];
    }
  }

  for (size_t channel = 0; channel < ADC_CHANNELS; channel++) {
    raw_values[channel] = (float)sums[channel] / (float)sequences;
    voltages_mv[channel] = (float)(reference_mv * raw_values[channel]) / ((float)ADC_FULL_SCALE);
  }

  // // throttleLevel
  // static float throttleMinVoltage = 0.95;
  // static float throttleMaxVoltage = 2.5;

  // // brakeLevel
  // static float brakeMinVoltage = 0.845;
  // static float brakeMaxVoltage = 2.48;

  // battery.voltage
  static float batteryVoltageR1 = 220000 + 2000 /* +2000 is the correction value */;
  static float batteryVoltageR2 = 4700;

  // throttleLevel = map_f(analogReadings.analog0, throttleMinVoltage, throttleMaxVoltage, 0.0, 100.0);
  // brakeLevel    = map_f(analogReadings.analog1, brakeMinVoltage, brakeMaxVoltage, 0.0, 100.0);
  voltage       = voltages_mv[0] / (batteryVoltageR2 / (batteryVoltageR1 + batteryVoltageR2));

  // CALIBRATED!!! on pin A1
  {
    double measuring_zero_input_offset_mv = ((double)reference_mv * 0.5) + 0.002; // 0.04
    double amplifier_gain = 50.0;
    double shunt_resistance_mohms = 0.241; // setting this to 1.0 will make the "current" output show the number of milivolts at the analog input pin
    current = movingAverage_current.moveAverage(
      (voltages_mv[1] - measuring_zero_input_offset_mv) / (amplifier_gain * shunt_resistance_mohms)
    );
  }

  // CALIBRATED!!! on pin A0
  {
    double measuring_zero_input_offset_mv = ((double)reference_mv * 0.5) + 0.002; // 0.04
    double amplifier_gain = 50.0;
    double shunt_resistance_mohms = 0.0867; // setting this to 1.0 will make the "current" output show the number of milivolts at the analog input pin
    current_2 = movingAverage_current_2.moveAverage(
      (voltages_mv[0] - measuring_zero_input_offset_mv) / (amplifier_gain * shunt_resistance_mohms)
    );
  }


  n++;
  if (adcIsBufferComplete(adcp)) {
    nx++;
  } else {
    ny++;
  }
}

// ADC12 errors callback, should never happen.
extern "C" void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;

  chSysHalt("it happened");
}

// the main logic will run here
extern "C" void timer_2khz_callback(GPTDriver *gptp) {
  (void)gptp;
  double dt = 1.0/2000.0;

  current_amp_seconds += current * dt;
  current_ah = current_amp_seconds / 3600.0;
}

int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the RTOS is active.
   */
  halInit();
  chSysInit();
  portab_setup();

  // GPIO setup
  pinMode(PIN_BUTTON_K1, INPUT_PULLDOWN);

  // USART3 (VESC)
  // PB10 (TX)
  // PB11 (RX)
  palSetLineMode(PAL_LINE(GPIOB, 10U), PAL_MODE_ALTERNATE(7U) | PAL_STM32_OSPEED_HIGHEST);
  palSetLineMode(PAL_LINE(GPIOB, 11U), PAL_MODE_ALTERNATE(7U) | PAL_STM32_OSPEED_HIGHEST);
  uartVesc.setUartPort(&SD3);
  uartVesc.begin(115200);
  VESC.setSerialPort(&uartVesc);

  // starts the cdc serial driver
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(500);

  // threads
  chThdCreateStatic(waCalibrationThread, sizeof(waCalibrationThread), NORMALPRIO, CalibrationThread, NULL);
  chThdCreateStatic(waUptimeCountingThread, sizeof(waUptimeCountingThread), NORMALPRIO, UptimeCountingThread, NULL);
  chThdCreateStatic(waLEDIntegratedThread, sizeof(waLEDIntegratedThread), NORMALPRIO, LEDIntegratedThread, NULL);
  chThdCreateStatic(waUSBSerialThread, sizeof(waUSBSerialThread), NORMALPRIO, USBSerialThread, NULL);

  // ADC12
  adcStart(&PORTAB_ADC1, &portab_adccfg1);
  gptStart(&GPTD6, &timer_2khz_config);
  // Starting an ADC continuous conversion triggered with a period of 1/2000 second.
  adcStartConversion(&PORTAB_ADC1, &portab_adcgrpcfg1, samples, ADC_GRP2_BUF_DEPTH);
  gptStartContinuous(&GPTD6, 500U);

  // systime_t prev = chVTGetSystemTimeX();
  // uint32_t _temp_iterations = 0;
  while (true) {
    // // count how many _temp_iterations++ it can do every second
    // _temp_iterations++;
    // systime_t now = chVTGetSystemTimeX();
    // sysinterval_t elapsed = chTimeDiffX(prev, now);

    // if (elapsed >= TIME_MS2I(1000)) {
    //   uint32_t time_elapsed_ms = chTimeI2MS(elapsed);

    //   // using floats so we can avoid multiplying by 1000, which can overflow the 32-bit integer value
    //   cycles_per_second = (uint32_t) ((float)_temp_iterations / ((float)time_elapsed_ms / 1000.0f));

    //   _temp_iterations = 0;
    //   prev = now;
    // }

    chThdSleepMilliseconds(1000);
  }
}
