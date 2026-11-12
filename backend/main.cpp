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

extern "C" {
  void* __dso_handle = &__dso_handle;
  void _fini() {};
}

#include "ch.h"
#include "hal.h"
#include "portab.h"
#include "usbcfg.h"

#include "adc3.hpp"
#include "abstractions.hpp"
#include "commonUtils.hpp"
#include "myUart.hpp"
#include "VescUart/VescUart.h"
#include "messagingUtils.hpp"
#include "battery.hpp"
#include "comm.h"
#include "rollingRangeEstimation.hpp"
#include "magic_enum.hpp"

#define EBIKE_NAME "EBIKE"
#define EBIKE_VERSION "1.0.0"

// voltages[ADC_PAx]
#define ADC_PA0 0
#define ADC_PA1 1
#define ADC_PA2 2
#define ADC_PA3 3
#define ADC_PA6 4
#define ADC_PA7 5

time_measurement_t vesc_tm;

static volatile uint32_t _loopRateCount_VESC = 0;
static volatile uint32_t loopRate_VESC = 0;

static volatile uint32_t _timerCallbackCount = 0;
static volatile uint32_t timerCallbacksLastSecond = 0;
mutex_t mtx_toSendExtra;

Battery battery;
// ThrottleMap throttleMap;
// ThrottleMap brakeMap;
// PowerProfiles PP;
RollingRangeEstimation rollingRangeEstimation;
VescUart::dataPackage VESCCombined;
VescUart::dataPackage VESCPrimary;
VescUart::dataPackage VESCSecondary;

struct Trip {
    double distance; // in km
    double wattHoursUsed;
    double wattHoursConsumed;
    double wattHoursRegenerated;

    double range; // calculated at runtime
    double WhPerKm; // calculated at runtime
    double rideTime; // in seconds
};
Trip trip_A, trip_B;

struct {
    double rideTime; // in seconds
    double distance; // in km
} odometer;

struct Settings {
    bool batteryPercentageVoltageBased = 0;
    bool automaticRegenerativeBraking = 0;
    bool minimizeDrivetrainBacklash = 0;
    bool enableDualVESC = 1; //TODO: REVERT TO 0
    int dualMotorDriveStyle = 0;
    // only for Dual VESC
    int primaryVESCID = 0;
    int secondaryVESCID = 0;
    int setSecondaryVescAsFirstAccelerationMotor = 0;
    int setSecondaryVescAsFirstBrakingMotor = 0;
};
Settings settings;

struct {
    double analog0; // Throttle
    double analog1; // Throtle brake
    double analog2;
    double analog3;
    double analog4;
    double analog5;
    double analog6;
    double analog7; // Battery voltage
} analogReadings;

struct {
    int poles = 18;
    int magnetPairs = 3;
    float rpmPerKmh = 0; // calculated at runtime
} motor;

struct {
    float diameter = 63.0;
    float gear_ratio = 8.90625; // (25/10)*(57/16)
    float rpmPerKmh = 0; // calculated at runtime
} wheel;

void estimatedRangeCalculateStats(Trip *trip, double availableWatthours) {
    trip->WhPerKm = trip->wattHoursUsed / trip->distance;
    double tmp = availableWatthours / trip->WhPerKm;
    tmp != tmp ? trip->range = 0.0 : trip->range = tmp;
}

void tripReset(Trip *trip) {
    trip->distance = 0;
    trip->wattHoursConsumed = 0;
    trip->wattHoursRegenerated = 0;
    trip->range = 0;
    trip->WhPerKm = 0;
    trip->rideTime = 0;
}

static float acceleration = 0;
static float motor_rpm = 0;
static float speed_kmh = 0;
static float throttleLevel = 0;
static float brakeLevel = 0;
static bool  powerOn = false;
static float maxBrakingCurrent = 200.0;
static int throttleLoopRate = 100.0;
std::string toSendExtra;
std::string notes;
std::string debuginfo;
int throttleState = 0;
float primaryVESCSaturatedLeftoverCurrent;
float primaryCurrent;
float secondaryCurrent;
MyUart uartVesc;
VescUart VESC;

#include <cstring>
#include <string>

#define PIN_LED PAL_LINE(GPIOE, 3)
#define PIN_BUTTON_K1 PAL_LINE(GPIOC, 13)

/* Testing in progress.*/
static volatile bool testing = false;
#define ADC_FULL_SCALE 65535U

// ADC12
static volatile double voltages_mv[ADC_CHANNELS] = {};
static volatile double _voltage = 0.0f;
static volatile double _currentLowSens = 0.0f;
static volatile double _currentHighSens = 0.0f;

// clock
static volatile uint32_t uptimeInSeconds = 0; // can count up to 136.19 years, I think thats enough
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
  bool adc3_ready = false;

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

    if (adc3_ok) {
      adc3_calculate(vrefint_raw, temperature_raw, &reference_mv, &vrefint_mv, &temperature_c);
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

      uptimeInSeconds = uptimeInSeconds + 1;
      clockSecondsSinceBoot = uptimeInSeconds % 60;
      clockMinutesSinceBoot = (uint32_t)(uptimeInSeconds / 60.0f) % 60;
      clockHoursSinceBoot   = (uint32_t)(uptimeInSeconds / 60.0f / 60.0f) % 24;
      clockDaysSinceBoot    = uptimeInSeconds / 60.0f / 60.0f / 24;

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

std::string ProcessRead(const std::string& message) {
  std::string toSend;

  if (message.empty())
    return {};

  auto readStringPacket = msg::split(message, msg::messageEnd);

  for (int i = 0; i < (int)readStringPacket.size(); i++) {
    auto packet = msg::split(readStringPacket[i], ";");

    int index = 0;

    int packet_command_id = -1;
    packet_command_id = msg::getValueFromSplit_uint64(packet, index);

    switch (packet_command_id) {
      case COMMAND_ID::GET_BATTERY:
        msg::start(toSend, COMMAND_ID::GET_BATTERY);
        msg::addValue(toSend, battery.voltage, 4);
        msg::addValue(toSend, battery.current, 4);
        msg::addValue(toSend, battery.watts, 1);
        msg::addValue(toSend, battery.wattHoursUsed, 15);
        msg::addValue(toSend, battery.wattHoursFullyDischarged, 15);
        msg::addValue(toSend, battery.getAmpHoursUsed(), 6);
        msg::addValue(toSend, battery.getAmpHoursUsedLifetime(), 2);
        msg::addValue(toSend, battery.getAmpHoursFullyCharged(), 2);
        msg::addValue(toSend, battery.getAmpHoursFullyChargedWhenNew(), 2);
        msg::addValue(toSend, battery.percentage, 1);
        msg::addValue(toSend, battery.voltage_min, 1);
        msg::addValue(toSend, battery.voltage_max, 1);
        msg::addValue(toSend, battery.voltage_nominal, 1);
        msg::addValue(toSend, battery.amphours_min_voltage, 1);
        msg::addValue(toSend, battery.amphours_max_voltage, 1);
        msg::addValue(toSend, battery.charging, 0);
        msg::addValue(toSend, battery.getCycleCount(), 2);
        msg::end(toSend);
        break;

      case COMMAND_ID::ARE_YOU_ALIVE:
        msg::start(toSend, COMMAND_ID::ARE_YOU_ALIVE);
        msg::end(toSend);
        break;

      case COMMAND_ID::GET_STATS:
        msg::start(toSend, COMMAND_ID::GET_STATS);
        msg::addValue(toSend, speed_kmh, 1);
        msg::addValue(toSend, motor_rpm, 0);
        msg::addValue(toSend, motor.rpmPerKmh, 7);
        msg::addValue(toSend, motor.magnetPairs, 0);
        msg::addValue(toSend, odometer.distance, 7);
        msg::addValue(toSend, odometer.rideTime, 7);
        msg::addValue(toSend, trip_A.distance, 15);
        msg::addValue(toSend, trip_A.wattHoursUsed, 15);
        msg::addValue(toSend, trip_A.wattHoursConsumed, 15);
        msg::addValue(toSend, -(trip_A.wattHoursRegenerated), 15);
        msg::addValue(toSend, trip_A.range, 15);
        msg::addValue(toSend, trip_A.rideTime, 15);
        msg::addValue(toSend, trip_B.distance, 15);
        msg::addValue(toSend, trip_B.wattHoursUsed, 15);
        msg::addValue(toSend, trip_B.wattHoursConsumed, 15);
        msg::addValue(toSend, -(trip_B.wattHoursRegenerated), 15);
        msg::addValue(toSend, trip_B.range, 15);
        msg::addValue(toSend, trip_B.rideTime, 15);
        msg::addValue(toSend, VESCPrimary.avgMotorCurrent, 1);
        msg::addValue(toSend, VESCPrimary.avgCurrentDAxis, 1);
        msg::addValue(toSend, VESCPrimary.avgCurrentQAxis, 1);
        msg::addValue(toSend, VESCPrimary.dutyCycleNow * 100.0, 1); // value is now between 0 and 100
        msg::addValue(toSend, VESCPrimary.tempMotor, 1);
        msg::addValue(toSend, VESCPrimary.tempMosfet, 1);
        msg::addValue(toSend, VESCSecondary.avgMotorCurrent, 1);
        msg::addValue(toSend, VESCSecondary.avgCurrentDAxis, 1);
        msg::addValue(toSend, VESCSecondary.avgCurrentQAxis, 1);
        msg::addValue(toSend, VESCSecondary.dutyCycleNow * 100.0, 1); // value is now between 0 and 100
        msg::addValue(toSend, VESCSecondary.tempMotor, 1);
        msg::addValue(toSend, VESCSecondary.tempMosfet, 1);
        msg::addValue(toSend, VESCCombined.avgMotorCurrent, 1);
        msg::addValue(toSend, VESCCombined.avgCurrentDAxis, 1);
        msg::addValue(toSend, VESCCombined.avgCurrentQAxis, 1);
        msg::addValue(toSend, VESCCombined.dutyCycleNow * 100.0, 1); // value is now between 0 and 100
        msg::addValue(toSend, VESCCombined.tempMotor, 1);
        msg::addValue(toSend, VESCCombined.tempMosfet, 1);
        msg::addValue(toSend, uptimeInSeconds, 0);
        msg::addValue(toSend, -1.0 / 1000.0, 1); // whileLoopUsElapsed.count()
        msg::addValue(toSend, 1000.0 / loopRate_VESC, 1); // threadThrottle.getLoopRate()
        msg::addValue(toSend, 1000.0 / loopRate_VESC, 1); // threadVescProcessing.getLoopRate()
        msg::addValue(toSend, acceleration, 1);
        msg::addValue(toSend, powerOn, 0);
        msg::addValue(toSend, settings.automaticRegenerativeBraking, 0);
        msg::addValue(toSend, -1, 0); // PP.getProfile()
        msg::addValue(toSend, settings.minimizeDrivetrainBacklash, 0);
        msg::addValue(toSend, rollingRangeEstimation.getRange(), 15);
        msg::addValue(toSend, rollingRangeEstimation.getWhPerKm(), 15);
        msg::end(toSend);
        break;

      case COMMAND_ID::SET_ODOMETER:
        {
        odometer.distance = msg::getValueFromSplit_double(packet, index);

        msg::start(toSend, COMMAND_ID::BACKEND_LOG);
        msg::addString(toSend, "Odometer was set to: {} km", odometer.distance);
        msg::end(toSend);
        }
        break;

      case COMMAND_ID::SAVE_PREFERENCES:
        // TOMLSave(table, SETTINGS_FILEPATH);

        msg::start(toSend, COMMAND_ID::BACKEND_LOG);
        msg::addString(toSend, "Preferences were manually saved (not actually)");
        msg::end(toSend);
        break;

      case COMMAND_ID::RESET_TRIP_A:
        tripReset(&trip_A);

        msg::start(toSend, COMMAND_ID::BACKEND_LOG);
        msg::addString(toSend, "Trip A was reset");
        msg::end(toSend);
        break;

      case COMMAND_ID::GET_FW:
        {
        msg::start(toSend, COMMAND_ID::GET_FW);
        msg::addString(toSend, EBIKE_NAME);
        msg::addString(toSend, EBIKE_VERSION);
        msg::addString(toSend, "{} {}", __DATE__, __TIME__);
        msg::end(toSend);
        }
        break;

      case COMMAND_ID::SET_AMPHOURS_USED_LIFETIME:
        {
        battery.setAmpHoursUsedLifetime(msg::getValueFromSplit_double(packet, index));

        msg::start(toSend, COMMAND_ID::BACKEND_LOG);
        msg::addString(toSend, "Amphours used (Lifetime) was set to: {} Ah", battery.getAmpHoursUsedLifetime());
        msg::end(toSend);
        }
        break;

      case COMMAND_ID::SET_AMPHOURS_CHARGED:
        {
            float newValue = msg::getValueFromSplit(packet, index);
            battery.setAmpHoursFullyCharged(newValue);

            msg::start(toSend, COMMAND_ID::BACKEND_LOG);
            msg::addString(toSend, "Amphours charged was set to: {} Ah", newValue);
            msg::end(toSend);
        }
        break;

      case COMMAND_ID::TOGGLE_CHARGING_STATE:
        {
        battery.charging = !battery.charging;

        msg::start(toSend, COMMAND_ID::BACKEND_LOG);
        msg::addString(toSend, "Charging state was set to: {}", battery.charging);
        msg::end(toSend);
        }
        break;

      case COMMAND_ID::SET_AUTOMATIC_REGEN_BRAKING:
        {
        settings.automaticRegenerativeBraking = (bool)msg::getValueFromSplit(packet, index);

        msg::start(toSend, COMMAND_ID::BACKEND_LOG);
        msg::addString(toSend, "Regenerative braking state was set to: {}", settings.automaticRegenerativeBraking);
        msg::end(toSend);
        }
        break;

      case COMMAND_ID::RESET_TRIP_B:
        tripReset(&trip_B);

        msg::start(toSend, COMMAND_ID::BACKEND_LOG);
        msg::addString(toSend, "Trip B was reset");
        msg::end(toSend);
        break;

      case COMMAND_ID::GET_AVAILABLE_POWER_PROFILES:
        // msg::start(toSend, COMMAND_ID::GET_AVAILABLE_POWER_PROFILES);

        // listOfProfiles.clear();
        // for (int profile = 0; profile < PROFILE::PROFILE_COUNT; profile++) {
        //     listOfProfiles.append(PROFILE_TO_STRING.at(static_cast<PROFILE>(profile)));
        //     listOfProfiles += '\0';
        // }

        // msg::addString(toSend, "{}", listOfProfiles);
        // msg::end(toSend);
        break;

      case COMMAND_ID::SET_POWER_PROFILE:
        // PP.setProfile((int)msg::getValueFromSplit(packet, index));
        // setMcconfFromCurrentProfile();

        // msg::start(toSend, COMMAND_ID::BACKEND_LOG);
        // msg::addString(toSend, "Power profile was set");
        // msg::end(toSend);
        break;

      case COMMAND_ID::SET_MINIMIZE_DRIVETRAIN_BACKLASH:
        settings.minimizeDrivetrainBacklash = msg::getValueFromSplit(packet, index);

        break;

      case COMMAND_ID::GET_NOTES:
        msg::start(toSend, COMMAND_ID::GET_NOTES);
        msg::addString(toSend, "{}", notes);
        msg::end(toSend);

        break;

      case COMMAND_ID::SET_NOTES:
        notes = msg::getValueFromSplit_string(packet, index);

        break;

      case COMMAND_ID::GET_DEBUGINFO:
        {
          std::string throttleFunctionDebugInfo;
            // auto _state = magic_enum::enum_cast<STATE>(throttleState);
            // std::string _enum;
            // if (_state) {
            //     _enum = magic_enum::enum_name(*_state);
            // } else {
            //     _enum = "-";
            // }
            // throttleFunctionDebugInfo = std::format("Throttle state: {}\n", _enum);

            // char _enum[16] = "Unknown";
            // char throttleFunctionDebugInfo[64] = snprintf(throttleFunctionDebugInfo, sizeof(throttleFunctionDebugInfo), "Throttle state: %s", _enum);

        uint32_t callbacks = timerCallbacksLastSecond;
        debuginfo = std::format("Battery:\n"
                                "{}\n"
                                "Speed: {} km/h\n"
                                "{}\n"
                                "VESC Primary Current\n"
                                "       Requested: {} A\n"
                                "       Actual:    {} A\n"
                                "VESC Secondary Current\n"
                                "       Requested: {} A\n"
                                "       Actual:    {} A\n"
                                "\n"
                                "primaryVESCSaturatedLeftoverCurrent: {} A\n"
                                "2kHz callback rate: {} \n"
                                , battery.getStatus()
                                , speed_kmh
                                , throttleFunctionDebugInfo
                                , primaryCurrent //VESCPrimary.avgCurrentQAxis
                                , VESCPrimary.avgCurrentQAxis
                                , secondaryCurrent //VESCSecondary.avgCurrentQAxis
                                , VESCSecondary.avgCurrentQAxis
                                , primaryVESCSaturatedLeftoverCurrent
                                , callbacks
                                );

        debuginfo += std::format("\nAnalog Readings:\n"
                                "   Analog0: {:.6f} V\n"
                                "   Analog1: {:.6f} V\n"
                                "   Analog2: {:.6f} V\n"
                                "   Analog3: {:.6f} V\n"
                                "   Analog4: {:.6f} V\n"
                                "   Analog5: {:.6f} V\n"
                                "   Analog6: {:.6f} V\n"
                                "   Analog7: {:.6f} V\n"
                                , (double)voltages_mv[ADC_PA0] / 1000.0
                                , (double)voltages_mv[ADC_PA1] / 1000.0
                                , (double)voltages_mv[ADC_PA2] / 1000.0
                                , (double)voltages_mv[ADC_PA3] / 1000.0
                                , analogReadings.analog4
                                , analogReadings.analog5
                                , (double)voltages_mv[ADC_PA6] / 1000.0
                                , (double)voltages_mv[ADC_PA7] / 1000.0
                                );

        msg::start(toSend, COMMAND_ID::GET_DEBUGINFO);
        msg::addString(toSend, "{}", debuginfo);
        msg::end(toSend);
        }

        break;

    case COMMAND_ID::GET_POWER_PROFILE_ALL:
      // run_command_GET_POWER_PROFILE_ALL(toSend);

      break;

      case COMMAND_ID::SET_POWER_PROFILE_VALUES:
        // PP.setProfile(STRING_TO_PROFILE.at(msg::getValueFromSplit_string(packet, index)));
        // PP.set(PP.getProfile(), PP_VALS::L_CURRENT_MIN_SCALE, msg::getValueFromSplit(packet, index));
        // PP.set(PP.getProfile(), PP_VALS::L_CURRENT_MAX_SCALE, msg::getValueFromSplit(packet, index));
        // PP.set(PP.getProfile(), PP_VALS::L_MIN_ERPM, msg::getValueFromSplit(packet, index));
        // PP.set(PP.getProfile(), PP_VALS::L_MAX_ERPM, msg::getValueFromSplit(packet, index));
        // PP.set(PP.getProfile(), PP_VALS::L_MIN_DUTY, msg::getValueFromSplit(packet, index));
        // PP.set(PP.getProfile(), PP_VALS::L_MAX_DUTY, msg::getValueFromSplit(packet, index));
        // PP.set(PP.getProfile(), PP_VALS::L_WATT_MIN, msg::getValueFromSplit(packet, index));
        // PP.set(PP.getProfile(), PP_VALS::L_WATT_MAX, msg::getValueFromSplit(packet, index));
        // PP.set(PP.getProfile(), PP_VALS::L_IN_CURRENT_MIN, msg::getValueFromSplit(packet, index));
        // PP.set(PP.getProfile(), PP_VALS::L_IN_CURRENT_MAX, msg::getValueFromSplit(packet, index));
        // PP.set(PP.getProfile(), PP_VALS::C_PHASE_CURRENT_MAX, msg::getValueFromSplit(packet, index));
        // PP.set(PP.getProfile(), PP_VALS::C_FORCE_SINGLE_MOTOR_ACCELERATION, msg::getValueFromSplit(packet, index));

        // setMcconfFromCurrentProfile();
        // run_command_GET_POWER_PROFILE_ALL(toSend);

        msg::start(toSend, COMMAND_ID::BACKEND_LOG);
        msg::addString(toSend, "New profile values set!");
        msg::end(toSend);

        break;
    }
  }

  chMtxLock(&mtx_toSendExtra);
  toSend.append(toSendExtra);
  toSendExtra.clear();
  chMtxUnlock(&mtx_toSendExtra);

  return toSend;
}

// CLANKED ALERT: the insides of this function are clanked
static THD_WORKING_AREA(waUSBSerialThread, 10240);
static THD_FUNCTION(USBSerialThread, arg) {
  (void)arg;
  chRegSetThreadName("USBSerialThread");

  // Keep incomplete input between reads. A frame is removed only after its
  // complete messageEnd marker has been found and ProcessRead has consumed it.
  char inputBuffer[1024];
  std::size_t buffered = 0;

  while (true) {
    // no space left for termination
    if (buffered == sizeof(inputBuffer)) {
      buffered = 0;
    }

    const std::size_t firstByte = chnReadTimeout(
        &SDU1,
        reinterpret_cast<std::uint8_t*>(inputBuffer + buffered),
        1,
        TIME_MS2I(100));
    if (firstByte == 0)
      continue;
    buffered += firstByte;

    if (buffered < sizeof(inputBuffer)) {
      buffered += chnReadTimeout(
          &SDU1,
          reinterpret_cast<std::uint8_t*>(inputBuffer + buffered),
          sizeof(inputBuffer) - buffered,
          TIME_IMMEDIATE);
    }

    while (buffered != 0) {
      const std::string pending(inputBuffer, buffered);
      const std::size_t endPosition = pending.find(msg::messageEnd);
      if (endPosition == std::string_view::npos)
        break;

      const std::size_t frameSize = endPosition + msg::messageEnd.size();
      const std::string response = ProcessRead(pending.substr(0, frameSize));

      std::size_t written = 0;
      while (written < response.size()) {
        const std::size_t count = chnWrite(
            &SDU1,
            reinterpret_cast<const std::uint8_t*>(response.data() + written),
            response.size() - written);
        if (count == 0)
          break;
        written += count;
      }

      // Preserve data after this frame
      // if this buffer contains another frame, that frame will be completed by another read.
      const std::size_t remaining = buffered - frameSize;
      std::memmove(inputBuffer, inputBuffer + frameSize, remaining);
      buffered = remaining;
    }
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

MovingAverage movingAverage_currentHighSens;
MovingAverage movingAverage_currentLowSens;

extern "C" void adccallback(ADCDriver *adcp) {
  movingAverage_currentHighSens.smoothingFactor = 1.0f;
  movingAverage_currentLowSens.smoothingFactor = movingAverage_currentHighSens.smoothingFactor;


  // CLANKED FROM HERE
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
  // CLANKED TO HERE

  // throttleLevel
  static float throttleMinVoltage = 0.95;
  static float throttleMaxVoltage = 2.5;

  // brakeLevel
  static float brakeMinVoltage = 0.845;
  static float brakeMaxVoltage = 2.48;

  // battery.voltage
  static float batteryVoltageR1 = 220000 + 2150 /* +2150 is the correction value */;
  static float batteryVoltageR2 = 4700;

  throttleLevel = map_f((voltages_mv[ADC_PA6]/1000.0), throttleMinVoltage, throttleMaxVoltage, 0.0, 100.0);
  brakeLevel    = map_f((voltages_mv[ADC_PA2]/1000.0), brakeMinVoltage, brakeMaxVoltage, 0.0, 100.0);
  _voltage      = (voltages_mv[ADC_PA7]/1000.0) / (batteryVoltageR2 / (batteryVoltageR1 + batteryVoltageR2));

  // CALIBRATED!!! on pin A0
  {
    double measuring_zero_input_offset_mv = ((double)reference_mv * 0.5) + 0.002; // 0.04
    double amplifier_gain = 50.0;
    double shunt_resistance_mohms = 0.0867; // setting this to 1.0 will make the "current" output show the number of milivolts at the analog input pin
    _currentLowSens = movingAverage_currentLowSens.moveAverage(
      (voltages_mv[ADC_PA0] - measuring_zero_input_offset_mv) / (amplifier_gain * shunt_resistance_mohms)
    );
  }

  // CALIBRATED!!! on pin A1
  {
    double measuring_zero_input_offset_mv = ((double)reference_mv * 0.5) + 0.002; // 0.04
    double amplifier_gain = 50.0;
    double shunt_resistance_mohms = 0.241; // setting this to 1.0 will make the "current" output show the number of milivolts at the analog input pin
    _currentHighSens = movingAverage_currentHighSens.moveAverage(
      (voltages_mv[ADC_PA1] - measuring_zero_input_offset_mv) / (amplifier_gain * shunt_resistance_mohms)
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

static THD_WORKING_AREA(waVescDataProcessingThread, 3072);
static THD_FUNCTION(VescDataProcessingThread, arg) {
  (void)arg;
  chRegSetThreadName("VescDataProcessingThread");

  double timeNowS = 0.0;
  float speed_kmh_previous = 0.0f;
  vesc_tm.last = MS2RTC(SystemCoreClock, 1000);

  while(true) {
    double dt_local = (double)RTC2US(SystemCoreClock, vesc_tm.last) / 1000000.0;
    chTMStartMeasurementX(&vesc_tm);
    _loopRateCount_VESC += 1;

    VESC.setCurrent(throttleLevel, 0.0);
    VESC.setCurrent(throttleLevel, 7.0);

    if (VESC.getVescValues()) {
      VESCPrimary = VESC.data;
      VescUart::dataPackage tempData = VESCPrimary;

      if (settings.enableDualVESC && VESC.getVescValues(settings.secondaryVESCID)) {
          VESCSecondary = VESC.data;

          tempData.ampHours += VESCSecondary.ampHours;
          tempData.ampHoursCharged += VESCSecondary.ampHoursCharged;
          tempData.avgCurrentDAxis += VESCSecondary.avgCurrentDAxis;
          tempData.avgCurrentQAxis += VESCSecondary.avgCurrentQAxis;
          tempData.avgInputCurrent += VESCSecondary.avgInputCurrent;
          tempData.avgMotorCurrent += VESCSecondary.avgMotorCurrent;
          if (tempData.dutyCycleNow < VESCSecondary.dutyCycleNow) {
              tempData.dutyCycleNow = VESCSecondary.dutyCycleNow;
          }
          tempData.wattHours += VESCSecondary.wattHours;
          tempData.wattHoursCharged += VESCSecondary.wattHoursCharged;
          if (tempData.tempMotor < VESCSecondary.tempMotor) {
              tempData.tempMotor = VESCSecondary.tempMosfet;
          }
          if (tempData.tempMosfet < VESCSecondary.tempMosfet) {
              tempData.tempMosfet = VESCSecondary.tempMosfet;
          }
      }
      VESCCombined = tempData;

      static RampLimiter _tempPrimaryAvgMotorCurrent;
      if (VESCPrimary.avgMotorCurrent < primaryCurrent) {
          float _temp = primaryCurrent - VESCPrimary.avgMotorCurrent;
          if (_temp < 0.0)
              _temp = 0.0;

          primaryVESCSaturatedLeftoverCurrent = _tempPrimaryAvgMotorCurrent.getValue(_temp, 100, 200, 100, dt_local);

      } else {
          primaryVESCSaturatedLeftoverCurrent = _tempPrimaryAvgMotorCurrent.getValue(0.0, 100, 1000, 80, dt_local);
      }

      static double tachometer_abs_previous;
      static double tachometer_abs_diff;
      static double distanceDiff;

      // if the previous measurement is bigger than the current, that means
      // that the VESC was probably powered off and on, so the stats got reset...
      // So this makes sure that we do not make a tachometer_abs_diff thats suddenly a REALLY
      // large number and therefore screw up our distance measurement
      if (tachometer_abs_previous > VESCCombined.tachometerAbs) {
          tachometer_abs_previous = VESCCombined.tachometerAbs;
      }

      // prevent the diff to be something extremely big
      if ((VESCCombined.tachometerAbs - tachometer_abs_previous) >= 1000) {
          tachometer_abs_previous = VESCCombined.tachometerAbs;
      }

      if (tachometer_abs_previous < VESCCombined.tachometerAbs) {
          tachometer_abs_diff = VESCCombined.tachometerAbs - tachometer_abs_previous;
          tachometer_abs_previous = VESCCombined.tachometerAbs;

          distanceDiff = ((tachometer_abs_diff / (double)motor.poles) / (double)wheel.gear_ratio) * (double)wheel.diameter * 3.14159265 / 100000.0; // divide by 100000 for trip distance to be in kilometers

          trip_A.distance += distanceDiff;
          trip_B.distance += distanceDiff;
          odometer.distance += distanceDiff;
          rollingRangeEstimation.addDeltaDistance(distanceDiff);

      }

      motor_rpm = (VESCCombined.rpm / (float)motor.magnetPairs);
      speed_kmh = (motor_rpm / wheel.gear_ratio) * wheel.diameter * 3.14159265f * 60.0f/*minutes*/ / 100000.0f/*1 km in cm*/;

      timeNowS += dt_local;
      if (timeNowS >= 0.4 /*s*/) {

          acceleration = (speed_kmh - speed_kmh_previous) * (1.0 / timeNowS);
          speed_kmh_previous = speed_kmh;
          timeNowS = 0.0;
      }
    }
    chTMStopMeasurementX(&vesc_tm);
  }
}

// the main logic will run here
extern "C" void timer_2khz_callback(GPTDriver *gptp) {
  (void)gptp;
  _timerCallbackCount += 1;
  double dt = 1.0/2000.0;

  // TODO: wire up and interpolate when switching between _currentHighSens and _currentLowSens values
  double _current = _currentHighSens;

  battery.processLoop(_voltage, _current, false, dt);
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

  wheel.rpmPerKmh = (1.0 /*km/h*/ * 1000.0 /*meters*/) / ((3.14 * wheel.diameter) * 60 /*minutes*/) * 100.0 /*?*/;
  motor.rpmPerKmh = wheel.rpmPerKmh * wheel.gear_ratio;

  battery.setup();
  chMtxObjectInit(&mtx_toSendExtra);
  chTMObjectInit(&vesc_tm);

  // GPIO setup
  pinMode(PIN_BUTTON_K1, INPUT_PULLDOWN);

  // USART3 (VESC)
  // PB10 (TX)
  // PB11 (RX)
  // The board defaults USART3 to the ST-Link VCP on PD8/PD9.
  // Disconnect those AF pads before remapping USART3 to PB10/PB11,
  // otherwise it interferes with receiving messages.
  palSetLineMode(PAL_LINE(GPIOD, 8U), PAL_MODE_INPUT_ANALOG);
  palSetLineMode(PAL_LINE(GPIOD, 9U), PAL_MODE_INPUT_ANALOG);
  palSetLineMode(PAL_LINE(GPIOB, 10U), PAL_MODE_ALTERNATE(7U) | PAL_STM32_OSPEED_HIGHEST);
  palSetLineMode(PAL_LINE(GPIOB, 11U), PAL_MODE_ALTERNATE(7U) | PAL_STM32_OSPEED_HIGHEST);
  uartVesc.setUartPort(&SD3);
  uartVesc.begin(230400);
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
  chThdCreateStatic(waVescDataProcessingThread, sizeof(waVescDataProcessingThread), NORMALPRIO, VescDataProcessingThread, NULL);

  // ADC12
  adcStart(&PORTAB_ADC1, &portab_adccfg1);
  gptStart(&GPTD6, &timer_2khz_config);
  // Starting an ADC continuous conversion triggered with a period of 1/2000 second.
  adcStartConversion(&PORTAB_ADC1, &portab_adcgrpcfg1, samples, ADC_GRP2_BUF_DEPTH);
  gptStartContinuous(&GPTD6, 500U);

  uint32_t previous_VESC = _loopRateCount_VESC;

  uint32_t previous = _timerCallbackCount;
  while (true) {
    chThdSleepMilliseconds(1000);
    const uint32_t current = _timerCallbackCount;
    timerCallbacksLastSecond = current - previous;
    previous = current;

    const uint32_t current_VESC = _loopRateCount_VESC;
    loopRate_VESC = current_VESC - previous_VESC;
    previous_VESC = current_VESC;
  }
}
