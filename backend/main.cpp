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
#include "vescCan.hpp"
#include "messagingUtils.hpp"
#include "battery.hpp"
#include "comm.h"
#include "rollingRangeEstimation.hpp"
#include "spiFlashLittleFs.hpp"
#include "magic_enum.hpp"
#include <ArduinoJson.h>
#include "profiles.hpp"
#include "valueTransition.hpp"
#include "timer.hpp"

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
ThrottleMap throttleMap;
ThrottleMap brakeMap;
std::vector<Point> throttleCurve;
std::vector<Point> throttleCurveDualVESC;
std::vector<Point> brakeCurve;
PowerProfiles PP;
RollingRangeEstimation rollingRangeEstimation;
VescCan::dataPackage VESCCombined;
VescCan::dataPackage VESCPrimary;
VescCan::dataPackage VESCSecondary;

#define JSON_SETTINGS_SIZE 10240
JsonDocument jsonSettings;
static char jsonSettingsBuffer[JSON_SETTINGS_SIZE];

enum DUAL_MOTOR_DRIVE_STYLE {
    SIMPLE = 0,
    ADVANCED_1 = 1,
    ADVANCED_2 = 2,
};

enum STATE {
    POWER_OFF_OR_CHARGING = 0,
    THROTTLE = 1,
    BRAKING = 2,
    THROTTLE_TRANSITION_TO_BRAKING = 3,
    BRAKING_TRANSITION_TO_THROTTLE = 4,
    POWER_OFF_OR_CHARGING_BRAKING = 5,
};

struct {
    int is_accelerating = 1;
    float requested_current = 0;
    float maximum_current = 1;
} throttle_info;

struct {
    ValueTransition throttleShockCurrent;
    ValueTransition throttleReal;
    ValueTransition throttleToBrake;
    ValueTransition toRealBrake;
} valueTransition;

struct Trip {
    double distance; // in km
    double wattHoursConsumed;
    double wattHoursRegenerated;
    double rideTime; // in seconds

    // calculated at runtime
    double wattHoursUsed;
    double range;
    double WhPerKm;
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
    bool enableDualVESC = 0;
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
static bool  powerOn = true; // TODO: revert to false
static float maxBrakingCurrent = 200.0;
std::string toSendExtra;
std::string notes;
std::string debuginfo;
int throttleState = 0;
float primaryVESCSaturatedLeftoverCurrent;
float primaryCurrent;
float secondaryCurrent;
VescCan VESC;

bool settingsApply(const JsonDocument &doc) {
  JsonObjectConst tbl = doc.as<JsonObjectConst>();
  if (tbl.isNull()) {
    return false;
  }

  odometer.distance                       = tbl["odometer"]["distance"] | odometer.distance;
  odometer.rideTime                       = tbl["odometer"]["rideTime"] | odometer.rideTime;
  trip_A.distance                         = tbl["trip"]["distance"] | trip_A.distance;
  trip_A.wattHoursConsumed                = tbl["trip"]["wattHoursConsumed"] | trip_A.wattHoursConsumed;
  trip_A.wattHoursRegenerated             = tbl["trip"]["wattHoursRegenerated"] | trip_A.wattHoursRegenerated;
  trip_A.rideTime                         = tbl["trip"]["rideTime"] | trip_A.rideTime;
  trip_B.distance                         = tbl["trip_B"]["distance"] | trip_B.distance;
  trip_B.wattHoursConsumed                = tbl["trip_B"]["wattHoursConsumed"] | trip_B.wattHoursConsumed;
  trip_B.wattHoursRegenerated             = tbl["trip_B"]["wattHoursRegenerated"] | trip_B.wattHoursRegenerated;
  trip_B.rideTime                         = tbl["trip_B"]["rideTime"] | trip_B.rideTime;
  battery.setAmpHoursUsed(                  tbl["battery"]["ampHoursUsed"] | 0.0);
  battery.setAmpHoursFullyCharged(          tbl["battery"]["ampHoursFullyCharged"] | 0.0);
  battery.setAmpHoursFullyChargedWhenNew(   tbl["battery"]["ampHoursFullyChargedWhenNew"] | 0.0);
  battery.setAmpHoursUsedLifetime(          tbl["battery"]["ampHoursUsedLifetime"] | 0.0);
  battery.setCycleCount(                    tbl["battery"]["cycleCount"] | 0.0);
  battery.wattHoursUsed                   = tbl["battery"]["wattHoursUsed"] | battery.wattHoursUsed;
  battery.wattHoursFullyDischarged        = tbl["battery"]["wattHoursFullyDischarged"] | battery.wattHoursFullyDischarged;
  settings.batteryPercentageVoltageBased  = tbl["settings"]["batteryPercentageVoltageBased"] | settings.batteryPercentageVoltageBased;
  settings.automaticRegenerativeBraking   = tbl["settings"]["automaticRegenerativeBraking"] | settings.automaticRegenerativeBraking;
  settings.minimizeDrivetrainBacklash     = tbl["settings"]["minimizeDrivetrainBacklash"] | settings.minimizeDrivetrainBacklash;
  settings.enableDualVESC                 = tbl["settings"]["enableDualVESC"] | settings.enableDualVESC;
  settings.dualMotorDriveStyle            = tbl["settings"]["dualMotorDriveStyle"] | settings.dualMotorDriveStyle;
  settings.primaryVESCID                  = tbl["settings"]["primaryVESCID"] | settings.primaryVESCID;
  settings.secondaryVESCID                = tbl["settings"]["secondaryVESCID"] | settings.secondaryVESCID;
  settings.setSecondaryVescAsFirstAccelerationMotor   = tbl["settings"]["setSecondaryVescAsFirstAccelerationMotor"] | settings.setSecondaryVescAsFirstAccelerationMotor;
  settings.setSecondaryVescAsFirstBrakingMotor        = tbl["settings"]["setSecondaryVescAsFirstBrakingMotor"] | settings.setSecondaryVescAsFirstBrakingMotor;
  PP.setProfile(                            tbl["PP"]["setProfile"] | (int)0);
  notes                                   = tbl["notes"]["note1"] | "";

  for (int profile = 0; profile < PROFILE::PROFILE_COUNT; profile++) {
      for (int var = 0; var < PP_VALS::VALS_COUNT; var++) {
          double ret = tbl
                          [PROFILE_TO_STRING.at(static_cast<PROFILE>(profile))]
                          [PP_VALS_TO_STRING.at(static_cast<PP_VALS>(var))]
                          | 0.0;

          PP.set(profile, var, ret);
      }
  }

  return true;
}

bool settingsLoad(JsonDocument &doc) {
  size_t length = 0;

  if (!spiFlashReadBytes(
          "settings.json",
          jsonSettingsBuffer,
          sizeof(jsonSettingsBuffer) - 1,
          length)) {
      return false;
  }

  jsonSettingsBuffer[length] = '\0';

  const DeserializationError error = deserializeJson(doc, jsonSettingsBuffer, length);
  if (error) {
    return false;
  }

  return settingsApply(doc);
}

void settingsBuild(JsonDocument &doc) {
  JsonObject tbl = doc.to<JsonObject>();
  tbl["odometer"]["distance"] = odometer.distance;
  tbl["odometer"]["rideTime"] = odometer.rideTime;
  tbl["trip"]["distance"] = trip_A.distance;
  tbl["trip"]["wattHoursConsumed"] = trip_A.wattHoursConsumed;
  tbl["trip"]["wattHoursRegenerated"] = trip_A.wattHoursRegenerated;
  tbl["trip"]["rideTime"] = trip_A.rideTime;
  tbl["trip_B"]["distance"] = trip_B.distance;
  tbl["trip_B"]["wattHoursConsumed"] = trip_B.wattHoursConsumed;
  tbl["trip_B"]["wattHoursRegenerated"] = trip_B.wattHoursRegenerated;
  tbl["trip_B"]["rideTime"] = trip_B.rideTime;
  tbl["battery"]["ampHoursUsed"] = battery.getAmpHoursUsed();
  tbl["battery"]["ampHoursFullyCharged"] = battery.getAmpHoursFullyCharged();
  tbl["battery"]["ampHoursFullyChargedWhenNew"] = battery.getAmpHoursFullyChargedWhenNew();
  tbl["battery"]["ampHoursUsedLifetime"] = battery.getAmpHoursUsedLifetime();
  tbl["battery"]["cycleCount"] = battery.getCycleCount();
  tbl["battery"]["wattHoursUsed"] = battery.wattHoursUsed;
  tbl["battery"]["wattHoursFullyDischarged"] = battery.wattHoursFullyDischarged;
  tbl["settings"]["batteryPercentageVoltageBased"] = settings.batteryPercentageVoltageBased;
  tbl["settings"]["automaticRegenerativeBraking"] = settings.automaticRegenerativeBraking;
  tbl["settings"]["minimizeDrivetrainBacklash"] = settings.minimizeDrivetrainBacklash;
  tbl["settings"]["enableDualVESC"] = settings.enableDualVESC;
  tbl["settings"]["dualMotorDriveStyle"] = settings.dualMotorDriveStyle;
  tbl["settings"]["primaryVESCID"] = settings.primaryVESCID;
  tbl["settings"]["secondaryVESCID"] = settings.secondaryVESCID;
  tbl["settings"]["setSecondaryVescAsFirstAccelerationMotor"] = settings.setSecondaryVescAsFirstAccelerationMotor;
  tbl["settings"]["setSecondaryVescAsFirstBrakingMotor"] = settings.setSecondaryVescAsFirstBrakingMotor;
  tbl["PP"]["setProfile"] = PP.getProfile();
  tbl["notes"]["note1"] = notes;

  for (int profile = 0; profile < PROFILE::PROFILE_COUNT; profile++) {
      for (int var = 0; var < PP_VALS::VALS_COUNT; var++) {
          double ret = PP.get(profile, var);

          tbl[PROFILE_TO_STRING.at(static_cast<PROFILE>(profile)).c_str()]
             [PP_VALS_TO_STRING.at(static_cast<PP_VALS>(var)).c_str()] = ret;
      }
  }
}

bool settingsSave(JsonDocument &doc) {
  settingsBuild(doc);

  const size_t length = serializeJsonPretty(doc, jsonSettingsBuffer, sizeof(jsonSettingsBuffer));

  if (length == 0 || length >= sizeof(jsonSettingsBuffer)) {
      return false;
  }

  return spiFlashWriteBytes(
      "settings.json",
      jsonSettingsBuffer,
      length
  );
}

void setMcconfFromCurrentProfile() {
    int profile = PP.getProfile();
    bool distributeSettingsEvenlyBetweenDrivers = (!PP.get(PP.getProfile(), PP_VALS::C_FORCE_SINGLE_MOTOR_ACCELERATION) && settings.enableDualVESC) ? true : false;

    VESC.data_mcconf.l_current_min_scale = PP.get(profile, PP_VALS::L_CURRENT_MIN_SCALE);
    VESC.data_mcconf.l_current_max_scale = PP.get(profile, PP_VALS::L_CURRENT_MAX_SCALE);
    VESC.data_mcconf.l_min_erpm = PP.get(profile, PP_VALS::L_MIN_ERPM) / 1000.0 * 1.234625970641421;
    VESC.data_mcconf.l_max_erpm = PP.get(profile, PP_VALS::L_MAX_ERPM) / 1000.0 * 1.234625970641421;
    VESC.data_mcconf.l_min_duty = PP.get(profile, PP_VALS::L_MIN_DUTY);
    VESC.data_mcconf.l_max_duty = PP.get(profile, PP_VALS::L_MAX_DUTY);
    if (distributeSettingsEvenlyBetweenDrivers) {
        VESC.data_mcconf.l_watt_min = PP.get(profile, PP_VALS::L_WATT_MIN) / 2.0;
        VESC.data_mcconf.l_watt_max = PP.get(profile, PP_VALS::L_WATT_MAX) / 2.0;
        VESC.data_mcconf.l_in_current_min = PP.get(profile, PP_VALS::L_IN_CURRENT_MIN) / 2.0;
        VESC.data_mcconf.l_in_current_max = PP.get(profile, PP_VALS::L_IN_CURRENT_MAX) / 2.0;
        VESC.data_mcconf.c_phase_current_max = PP.get(profile, PP_VALS::C_PHASE_CURRENT_MAX) / 2.0;
    } else {
        VESC.data_mcconf.l_watt_min = PP.get(profile, PP_VALS::L_WATT_MIN);
        VESC.data_mcconf.l_watt_max = PP.get(profile, PP_VALS::L_WATT_MAX);
        VESC.data_mcconf.l_in_current_min = PP.get(profile, PP_VALS::L_IN_CURRENT_MIN);
        VESC.data_mcconf.l_in_current_max = PP.get(profile, PP_VALS::L_IN_CURRENT_MAX);
        VESC.data_mcconf.c_phase_current_max = PP.get(profile, PP_VALS::C_PHASE_CURRENT_MAX);
    }
    VESC.data_mcconf.c_force_single_motor_acceleration = PP.get(profile, PP_VALS::C_FORCE_SINGLE_MOTOR_ACCELERATION);
    VESC.data_mcconf.name = PROFILE_TO_STRING.at(static_cast<PROFILE>(profile)).c_str();

    VESC.setMcconfTempValues();
    if (distributeSettingsEvenlyBetweenDrivers)
        VESC.setMcconfTempValues(settings.secondaryVESCID);
}

void run_command_GET_POWER_PROFILE_ALL(std::string& toSend) {
    VescCan::mcconf_t _mcconf_vesc1;
    VescCan::mcconf_t _mcconf_vesc2;
    // int VESC1Success = false;
    // int VESC2Success = false;

    if (VESC.getMcconfTempValues(settings.primaryVESCID)) {
        _mcconf_vesc1 = VESC.data_mcconf;

        // VESC1Success = true;
    }

    if (settings.enableDualVESC) {
        if (VESC.getMcconfTempValues(settings.secondaryVESCID)) {
            _mcconf_vesc2 = VESC.data_mcconf;

            // VESC2Success = true;
        }
    }

    int profile = PP.getProfile();

    msg::start(toSend, COMMAND_ID::GET_POWER_PROFILE_ALL);
    // Profile values
    msg::addString(toSend, "{}", PROFILE_TO_STRING.at(static_cast<PROFILE>(profile)));
    msg::addValue(toSend, PP.get(profile, PP_VALS::L_CURRENT_MIN_SCALE), 7);
    msg::addValue(toSend, PP.get(profile, PP_VALS::L_CURRENT_MAX_SCALE), 7);
    msg::addValue(toSend, PP.get(profile, PP_VALS::L_MIN_ERPM), 7);
    msg::addValue(toSend, PP.get(profile, PP_VALS::L_MAX_ERPM), 7);
    msg::addValue(toSend, PP.get(profile, PP_VALS::L_MIN_DUTY), 7);
    msg::addValue(toSend, PP.get(profile, PP_VALS::L_MAX_DUTY), 7);
    msg::addValue(toSend, PP.get(profile, PP_VALS::L_WATT_MIN), 7);
    msg::addValue(toSend, PP.get(profile, PP_VALS::L_WATT_MAX), 7);
    msg::addValue(toSend, PP.get(profile, PP_VALS::L_IN_CURRENT_MIN), 7);
    msg::addValue(toSend, PP.get(profile, PP_VALS::L_IN_CURRENT_MAX), 7);
    msg::addValue(toSend, PP.get(profile, PP_VALS::C_PHASE_CURRENT_MAX), 7);
    msg::addValue(toSend, PP.get(profile, PP_VALS::C_FORCE_SINGLE_MOTOR_ACCELERATION), 7);
    // VESC ONE
    msg::addValue(toSend, _mcconf_vesc1.l_current_min_scale, 7);
    msg::addValue(toSend, _mcconf_vesc1.l_current_max_scale, 7);
    msg::addValue(toSend, _mcconf_vesc1.l_min_erpm, 7);
    msg::addValue(toSend, _mcconf_vesc1.l_max_erpm, 7);
    msg::addValue(toSend, _mcconf_vesc1.l_min_duty, 7);
    msg::addValue(toSend, _mcconf_vesc1.l_max_duty, 7);
    msg::addValue(toSend, _mcconf_vesc1.l_watt_min, 7);
    msg::addValue(toSend, _mcconf_vesc1.l_watt_max, 7);
    msg::addValue(toSend, _mcconf_vesc1.l_in_current_min, 7);
    msg::addValue(toSend, _mcconf_vesc1.l_in_current_max, 7);
    msg::addValue(toSend, _mcconf_vesc1.c_phase_current_max, 7);
    // VESC TWO
    msg::addValue(toSend, _mcconf_vesc2.l_current_min_scale, 7);
    msg::addValue(toSend, _mcconf_vesc2.l_current_max_scale, 7);
    msg::addValue(toSend, _mcconf_vesc2.l_min_erpm, 7);
    msg::addValue(toSend, _mcconf_vesc2.l_max_erpm, 7);
    msg::addValue(toSend, _mcconf_vesc2.l_min_duty, 7);
    msg::addValue(toSend, _mcconf_vesc2.l_max_duty, 7);
    msg::addValue(toSend, _mcconf_vesc2.l_watt_min, 7);
    msg::addValue(toSend, _mcconf_vesc2.l_watt_max, 7);
    msg::addValue(toSend, _mcconf_vesc2.l_in_current_min, 7);
    msg::addValue(toSend, _mcconf_vesc2.l_in_current_max, 7);
    msg::addValue(toSend, _mcconf_vesc2.c_phase_current_max, 7);
    msg::end(toSend);
}

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
        msg::addValue(toSend, PP.getProfile(), 0);
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
      {
        const bool saved = settingsSave(jsonSettings);

        msg::start(toSend, COMMAND_ID::BACKEND_LOG);
        msg::addString(toSend, "Preferences save {}", saved ? "succeeded" : "failed");
        msg::end(toSend);
      }
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
      {
        msg::start(toSend, COMMAND_ID::GET_AVAILABLE_POWER_PROFILES);

        std::string listOfProfiles;
        for (int profile = 0; profile < PROFILE::PROFILE_COUNT; profile++) {
            listOfProfiles.append(PROFILE_TO_STRING.at(static_cast<PROFILE>(profile)));
            listOfProfiles += '\0';
        }

        msg::addString(toSend, "{}", listOfProfiles);
        msg::end(toSend);
      }
        break;

      case COMMAND_ID::SET_POWER_PROFILE:
        PP.setProfile((int)msg::getValueFromSplit(packet, index));
        setMcconfFromCurrentProfile();

        msg::start(toSend, COMMAND_ID::BACKEND_LOG);
        msg::addString(toSend, "Power profile was set");
        msg::end(toSend);
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
        auto _state = magic_enum::enum_cast<STATE>(throttleState);
        std::string _enum;
        if (_state) {
            _enum = magic_enum::enum_name(*_state);
        } else {
            _enum = "-";
        }
        std::string throttleFunctionDebugInfo = std::format("Throttle state: {}\n", _enum);

        uint32_t callbacks = timerCallbacksLastSecond;
        const VescCan::Diagnostics canDiagnostics = VESC.getDiagnostics();
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
                                "throttle_info.requested_current: {}\n"
                                "\n"
                                "primaryVESCSaturatedLeftoverCurrent: {} A\n"
                                "2kHz callback rate: {} \n"
                                "VESC CAN: tx {}/{} ({} timeout), rx {}, bus-off recoveries {}\n"
                                "          ECR 0x{:08x}, PSR 0x{:08x}\n"
                                , battery.getStatus()
                                , speed_kmh
                                , throttleFunctionDebugInfo
                                , primaryCurrent //VESCPrimary.avgCurrentQAxis
                                , VESCPrimary.avgCurrentQAxis
                                , secondaryCurrent //VESCSecondary.avgCurrentQAxis
                                , VESCSecondary.avgCurrentQAxis
                                , throttle_info.requested_current
                                , primaryVESCSaturatedLeftoverCurrent
                                , callbacks
                                , canDiagnostics.txCompleted
                                , canDiagnostics.txAttempts
                                , canDiagnostics.txTimeouts
                                , canDiagnostics.rxFrames
                                , canDiagnostics.busOffRecoveries
                                , canDiagnostics.errorCounter
                                , canDiagnostics.protocolStatus
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
      run_command_GET_POWER_PROFILE_ALL(toSend);

      break;

      case COMMAND_ID::SET_POWER_PROFILE_VALUES:
        PP.setProfile(STRING_TO_PROFILE.at(msg::getValueFromSplit_string(packet, index)));
        PP.set(PP.getProfile(), PP_VALS::L_CURRENT_MIN_SCALE, msg::getValueFromSplit(packet, index));
        PP.set(PP.getProfile(), PP_VALS::L_CURRENT_MAX_SCALE, msg::getValueFromSplit(packet, index));
        PP.set(PP.getProfile(), PP_VALS::L_MIN_ERPM, msg::getValueFromSplit(packet, index));
        PP.set(PP.getProfile(), PP_VALS::L_MAX_ERPM, msg::getValueFromSplit(packet, index));
        PP.set(PP.getProfile(), PP_VALS::L_MIN_DUTY, msg::getValueFromSplit(packet, index));
        PP.set(PP.getProfile(), PP_VALS::L_MAX_DUTY, msg::getValueFromSplit(packet, index));
        PP.set(PP.getProfile(), PP_VALS::L_WATT_MIN, msg::getValueFromSplit(packet, index));
        PP.set(PP.getProfile(), PP_VALS::L_WATT_MAX, msg::getValueFromSplit(packet, index));
        PP.set(PP.getProfile(), PP_VALS::L_IN_CURRENT_MIN, msg::getValueFromSplit(packet, index));
        PP.set(PP.getProfile(), PP_VALS::L_IN_CURRENT_MAX, msg::getValueFromSplit(packet, index));
        PP.set(PP.getProfile(), PP_VALS::C_PHASE_CURRENT_MAX, msg::getValueFromSplit(packet, index));
        PP.set(PP.getProfile(), PP_VALS::C_FORCE_SINGLE_MOTOR_ACCELERATION, msg::getValueFromSplit(packet, index));

        setMcconfFromCurrentProfile();
        run_command_GET_POWER_PROFILE_ALL(toSend);

        msg::start(toSend, COMMAND_ID::BACKEND_LOG);
        msg::addString(toSend, "New profile values set!");
        msg::end(toSend);

        break;

    case COMMAND_ID::GET_SETTINGS:
    {
      settingsBuild(jsonSettings);

      std::string json;
      const size_t length = serializeJsonPretty(jsonSettings, json);

      // remove whitespace
      std::erase(json, '\r');

      msg::start(toSend, COMMAND_ID::GET_SETTINGS);
      if (length != 0) {
        msg::addString(toSend, "{}", json);
      } else {
        msg::addString(toSend, "");
      }
      msg::end(toSend);

      msg::start(toSend, COMMAND_ID::BACKEND_LOG);
      msg::addString(toSend, "GET_SETTINGS!");
      msg::end(toSend);
    }
    break;

    case COMMAND_ID::SET_SETTINGS:
    {
      const std::string json = msg::getValueFromSplit_string(packet, index);
      const DeserializationError error = deserializeJson(jsonSettings, json);

      bool success = false;

      if (!error) {
          success = settingsApply(jsonSettings) && settingsSave(jsonSettings);
      }

      msg::start(toSend, COMMAND_ID::BACKEND_LOG);
      msg::addString(toSend, "SET_SETTINGS! Success??: {}", success ? "yes!!" : "no!!!!");
      msg::end(toSend);
    }
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
static THD_WORKING_AREA(waUSBSerialThread, 20480);
static THD_FUNCTION(USBSerialThread, arg) {
  (void)arg;
  chRegSetThreadName("USBSerialThread");

  // Keep incomplete input between reads. A frame is removed only after its
  // complete messageEnd marker has been found and ProcessRead has consumed it.
  char inputBuffer[16384];
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
  static float brakeMinVoltage = 1.2;// 0.845;
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

float clampValue(float input, float clampTo) {
    float output = 0.0;

    if (input < clampTo) {
        output = input;
    }

    if (input >= clampTo) {
        output = clampTo;
    }

    return output;
}

void ThrottleApplyThrottle(float current, float maxCurrent) {
    throttle_info.is_accelerating = 1;
    throttle_info.requested_current = current;
    throttle_info.maximum_current = maxCurrent;
}

void ThrottleApplyBraking(float current) {
    throttle_info.is_accelerating = 0;
    throttle_info.requested_current = current;
    throttle_info.maximum_current = 100; // TODO
}

/*
* Primary Motor = noisy at braking, silent at acceleration
* Secondary Motor = noisy at acceleration, silent at braking
*/
void throttleFunction(double elapsedMs) {   
    static float throttleCurrentToApply = 0.0;
    static float brakingCurrentToApply = 0.0;
    static float minCurrent = 5.5; // used when minimizeDrivetrainBacklash is enabled
    static float initialShockTransitionTime = 80.0;
    static float realThrottleTransitionTime = 60.0;
    static float throttleToBrakeTransitionTime = 40.0;
    static float realBrakeTransitionTime = 20.0;
    static float automaticRegenerativeBrakingCurrent = 12.0; // 12A
    static float minSpeedKmh = 0.1; // used for automaticRegenerativeBraking and throttle transitioning

    static bool _throttleFunctionOneTime = false;
    if (!_throttleFunctionOneTime) {
      _throttleFunctionOneTime = true;
      
      throttleState = STATE::POWER_OFF_OR_CHARGING;
      brakeMap.setCurve(brakeCurve);
    }

    valueTransition.throttleReal.timer.addElapsedTime(elapsedMs);
    valueTransition.throttleShockCurrent.timer.addElapsedTime(elapsedMs);
    valueTransition.throttleToBrake.timer.addElapsedTime(elapsedMs);
    valueTransition.toRealBrake.timer.addElapsedTime(elapsedMs);

    if (settings.enableDualVESC) {
        throttleMap.setCurve(throttleCurveDualVESC);
    } else {
        throttleMap.setCurve(throttleCurve);
    }

    static RampLimiter throttleRamp;
    float throttleClampCurrent = PP.get(PP.getProfile(), PP_VALS::C_PHASE_CURRENT_MAX);
    float throttleCurrent = throttleRamp.getValue(
                                clampValue(
                                        throttleMap.map(throttleLevel),
                                        throttleClampCurrent
                                ),
                                100,
                                15,
                                15,
                                elapsedMs
                            );

    static RampLimiter brakeRamp;
    float brakeCurrent = brakeRamp.getValue(
                            clampValue(
                                    brakeMap.map(brakeLevel),
                                    maxBrakingCurrent
                            ),
                            100,
                            40,
                            15,
                            elapsedMs
                          );

    if (!powerOn || battery.charging) {
        if (brakeLevel > 0.0) {
            throttleState = STATE::POWER_OFF_OR_CHARGING_BRAKING;
        } else {
            throttleState = STATE::POWER_OFF_OR_CHARGING;
        }
    }

    switch (throttleState) {
        case STATE::POWER_OFF_OR_CHARGING:
            ThrottleApplyThrottle(0.0, throttleClampCurrent);
            if (powerOn && !battery.charging) {
                throttleState = STATE::THROTTLE;
            }
            break;

        case STATE::THROTTLE:
            if (!settings.minimizeDrivetrainBacklash) {
                if (settings.automaticRegenerativeBraking && throttleLevel == 0.0 && speed_kmh > minSpeedKmh) {
                    throttleState = STATE::THROTTLE_TRANSITION_TO_BRAKING;
                    valueTransition.throttleToBrake.start();
                    break;
                }

                if (brakeLevel == 0.0) {
                    ThrottleApplyThrottle(throttleCurrent, throttleClampCurrent);
                    break;
                } else {
                    // ThrottleApplyBraking(brakeCurrent);
                }
            }

            if (brakeLevel > 0.0) {
                throttleState = STATE::THROTTLE_TRANSITION_TO_BRAKING;
                valueTransition.throttleToBrake.start();
                break;
            }

            static bool movingBecauseThrottle = false;

            if (throttleLevel == 0.0) {
                if (speed_kmh > minSpeedKmh) {
                    if (movingBecauseThrottle)
                        throttleCurrentToApply = minCurrent;
                    else
                        throttleCurrentToApply = 0;

                    if (settings.automaticRegenerativeBraking) {
                        throttleState = STATE::THROTTLE_TRANSITION_TO_BRAKING;
                        valueTransition.throttleToBrake.start();
                    }
                } else {
                    movingBecauseThrottle = false;
                    valueTransition.throttleShockCurrent.start();
                    throttleCurrentToApply = 0.0;
                }

                ThrottleApplyThrottle(throttleCurrentToApply, throttleClampCurrent);
                break;
            }

            if (throttleLevel > 0.0) {
                movingBecauseThrottle = true;
                if (valueTransition.throttleShockCurrent.timer.getTime_ms_now() < initialShockTransitionTime) {
                    throttleCurrentToApply = valueTransition.throttleShockCurrent.getValueDifference(0.0, minCurrent, initialShockTransitionTime);
                    valueTransition.throttleReal.start();
                } else {
                    if (throttleCurrent < minCurrent)
                        throttleCurrentToApply = minCurrent;
                    else {
                        if (valueTransition.throttleReal.timer.getTime_ms_now() < realThrottleTransitionTime) {
                            throttleCurrentToApply = valueTransition.throttleReal.getValueDifference(minCurrent, throttleCurrent, realThrottleTransitionTime);
                        } else {
                            throttleCurrentToApply = throttleCurrent;
                        }
                    }
                }
            }

            ThrottleApplyThrottle(throttleCurrentToApply, throttleClampCurrent);
            break;

        case STATE::BRAKING:
            static float _brakeCurrent;
            if (brakeLevel == 0.0 && (throttleLevel != 0.0 || speed_kmh <= minSpeedKmh || (settings.minimizeDrivetrainBacklash && !settings.automaticRegenerativeBraking))) {
                throttleState = STATE::BRAKING_TRANSITION_TO_THROTTLE;
                break;
            }

            if (settings.automaticRegenerativeBraking) {
                _brakeCurrent = automaticRegenerativeBrakingCurrent;

                if (brakeCurrent > automaticRegenerativeBrakingCurrent) {
                    _brakeCurrent += brakeCurrent - automaticRegenerativeBrakingCurrent;
                }
            } else {
                _brakeCurrent = brakeCurrent;
            }

            if (valueTransition.toRealBrake.timer.getTime_ms_now() < realBrakeTransitionTime) {
                brakingCurrentToApply = valueTransition.toRealBrake.getValueDifference(0.0, _brakeCurrent, realBrakeTransitionTime);

                ThrottleApplyBraking(brakingCurrentToApply); //brakingCurrentToApply
            } else {
                ThrottleApplyBraking(_brakeCurrent); //_brakeCurrent
            }

            break;

        case STATE::THROTTLE_TRANSITION_TO_BRAKING:
            static float _currentToApply;

            if (valueTransition.throttleToBrake.timer.getTime_ms_now() < throttleToBrakeTransitionTime) {
                _currentToApply = valueTransition.throttleToBrake.getValueDifference(throttleCurrentToApply, 0.0, throttleToBrakeTransitionTime);

                ThrottleApplyThrottle(_currentToApply, throttleClampCurrent);
            } else {
                throttleState = STATE::BRAKING;
                valueTransition.toRealBrake.start();
            }

            break;

        case STATE::BRAKING_TRANSITION_TO_THROTTLE:
            valueTransition.throttleShockCurrent.start();

            throttleState = STATE::THROTTLE;
            break;

        case STATE::POWER_OFF_OR_CHARGING_BRAKING:
            ThrottleApplyBraking(brakeCurrent);
            if (powerOn && !battery.charging) {
                throttleState = STATE::THROTTLE;
            }
            break;
    }
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

    throttleFunction(dt_local * 1000.0);

    const int _primaryAcceleratorVESCID = settings.setSecondaryVescAsFirstAccelerationMotor ? settings.secondaryVESCID : settings.primaryVESCID;
    const int _secondaryAcceleratorVESCID = settings.setSecondaryVescAsFirstAccelerationMotor ? settings.primaryVESCID : settings.secondaryVESCID;

    const int _primaryBrakerVESCID = settings.setSecondaryVescAsFirstBrakingMotor ? settings.secondaryVESCID : settings.primaryVESCID;
    const int _secondaryBrakerVESCID = settings.setSecondaryVescAsFirstBrakingMotor ? settings.primaryVESCID : settings.secondaryVESCID;

    static bool _wasBraking = false;
    static bool _wasAccelerating = true;

    switch (throttle_info.is_accelerating) {
        case true:
            // make sure the motors are not braking
            // if force_single_motor_acceleration is enabled and we are breaking, and now we apply throttle on the other motor that
            // wasnt braking, the braking motor will still be braking for a few moments, because when we stop sending any signal to
            // the previously braking motor it will stop braking only after a certain timeout
            // This sounds so dumb and wrongly worded that I am not even gonna bother....
            if (_wasBraking) {
                _wasBraking = false;

                VESC.setBrakeCurrent(0.0, _primaryBrakerVESCID);
                VESC.setBrakeCurrent(0.0, _secondaryBrakerVESCID);
            }
            _wasAccelerating = true;

            if (settings.enableDualVESC && !VESC.data_mcconf.c_force_single_motor_acceleration) {
                if (settings.dualMotorDriveStyle == DUAL_MOTOR_DRIVE_STYLE::SIMPLE) {
                    primaryCurrent = throttle_info.requested_current / 2.0;
                    secondaryCurrent = throttle_info.requested_current / 2.0;
                } else if (settings.dualMotorDriveStyle == DUAL_MOTOR_DRIVE_STYLE::ADVANCED_1) {
                    primaryCurrent = throttle_info.requested_current;
                    secondaryCurrent = primaryVESCSaturatedLeftoverCurrent;
                } else if (settings.dualMotorDriveStyle == DUAL_MOTOR_DRIVE_STYLE::ADVANCED_2) {
                    float singleMotorCurrent = 200;
                    const float defaultRatio = 1.0;
                    float ratioMaximumCurrent = 225;

                    if (throttle_info.requested_current > throttle_info.maximum_current)
                        throttle_info.requested_current = throttle_info.maximum_current;

                    float ratio = defaultRatio - (((0.5 - (1.0 - defaultRatio)) / (ratioMaximumCurrent - singleMotorCurrent)) * (throttle_info.requested_current - singleMotorCurrent));
                    if (ratio > defaultRatio)
                        ratio = defaultRatio;

                    if (ratio < 0.5)
                        ratio = 0.5;

                    primaryCurrent = throttle_info.requested_current * ratio;
                    secondaryCurrent = (throttle_info.requested_current * (1.0 - ratio)) + primaryVESCSaturatedLeftoverCurrent;

                    // if (throttle_info.maximum_current >= (500.0 / 2.0)) {
                    //     if (primaryCurrent > throttle_info.maximum_current / 2.0)
                    //         primaryCurrent = throttle_info.maximum_current / 2.0;

                    //     if (secondaryCurrent > throttle_info.maximum_current / 2.0)
                    //         secondaryCurrent = throttle_info.maximum_current / 2.0;
                    // }
                }

                VESC.setCurrent(primaryCurrent, _primaryAcceleratorVESCID);
                VESC.setCurrent(secondaryCurrent, _secondaryAcceleratorVESCID);
            } else {
                if (VESC.data_mcconf.c_force_single_motor_acceleration) {
                    VESC.setCurrent(throttle_info.requested_current, _primaryAcceleratorVESCID);
                } else {
                    VESC.setCurrent(throttle_info.requested_current);
                }
            }
            break;

        case false:
            // make sure the motors are not accelerating
            // if force_single_motor_acceleration is enabled and we are accelerating, and now we apply braking on the other motor that 
            // wasnt accelerating, the accelerating motor will still be accelerating for a few moments, because when we stop sending any
            // signal to the previously accelerating motor it will stop accelerating only after a certain timeout
            // This sounds so dumb and wrongly worded that I am not even gonna bother....
            if (_wasAccelerating) {
                _wasAccelerating = false;

                VESC.setCurrent(0.0, _primaryAcceleratorVESCID);
                VESC.setCurrent(0.0, _secondaryAcceleratorVESCID);
            }
            _wasBraking = true;

            if (settings.enableDualVESC) {
                if (!VESC.data_mcconf.c_force_single_motor_acceleration) {
                    VESC.setBrakeCurrent(throttle_info.requested_current * 0.9, _primaryBrakerVESCID);
                    VESC.setBrakeCurrent(throttle_info.requested_current * 0.1, _secondaryBrakerVESCID);
                } else {
                    bool areMotorsCoupled = false;
                    if (areMotorsCoupled) {
                        VESC.setBrakeCurrent(throttle_info.requested_current, _primaryBrakerVESCID);
                    } else {
                        VESC.setBrakeCurrent(throttle_info.requested_current, _primaryAcceleratorVESCID);
                    }
                }
            } else {
                VESC.setBrakeCurrent(throttle_info.requested_current);
            }
            break;
    }

    if (VESC.getVescValues(settings.primaryVESCID)) {
      VESCPrimary = VESC.data;
      VescCan::dataPackage tempData = VESCPrimary;

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

    // chThdSleepMilliseconds(5);
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

  rollingRangeEstimation.loop(battery.wattHoursRemaining);

  // should be accurate, only measures while moving
  if (!battery.charging && speed_kmh != 0.0) {
      rollingRangeEstimation.addDeltaWhUsed(battery.getWattHoursUsedInPreviousLoopCycle());

      if (battery.getWattHoursUsedInPreviousLoopCycle() >= 0.0) {
          trip_A.wattHoursConsumed += battery.getWattHoursUsedInPreviousLoopCycle();
          trip_B.wattHoursConsumed += battery.getWattHoursUsedInPreviousLoopCycle();
      }

      if (battery.getWattHoursUsedInPreviousLoopCycle() < 0.0) {
          trip_A.wattHoursRegenerated += battery.getWattHoursUsedInPreviousLoopCycle();
          trip_B.wattHoursRegenerated += battery.getWattHoursUsedInPreviousLoopCycle();
      }
  }

  trip_A.wattHoursUsed = trip_A.wattHoursConsumed + trip_A.wattHoursRegenerated;
  trip_B.wattHoursUsed = trip_B.wattHoursConsumed + trip_B.wattHoursRegenerated;

  estimatedRangeCalculateStats(&trip_A, battery.wattHoursRemaining);
  estimatedRangeCalculateStats(&trip_B, battery.wattHoursRemaining);

  {
    static double uptimeInSeconds_rideTime_tmp;
    static double uptimeInSeconds_rideTime_freeze = 0.0;
    uptimeInSeconds_rideTime_freeze += dt;
    if (speed_kmh == 0.0) {
      uptimeInSeconds_rideTime_tmp = uptimeInSeconds_rideTime_freeze;
    } else {
      double delta = uptimeInSeconds_rideTime_freeze - uptimeInSeconds_rideTime_tmp;
      trip_A.rideTime += delta;
      trip_B.rideTime += delta;
      odometer.rideTime += delta;
      uptimeInSeconds_rideTime_tmp = uptimeInSeconds_rideTime_freeze;
    }
  }
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

  // PROFILES
  PP_VALS_TO_STRING = {
    {PP_VALS::L_CURRENT_MIN_SCALE, "L_CURRENT_MIN_SCALE"},
    {PP_VALS::L_CURRENT_MAX_SCALE, "L_CURRENT_MAX_SCALE"},
    {PP_VALS::L_MIN_ERPM, "L_MIN_ERPM"},
    {PP_VALS::L_MAX_ERPM, "L_MAX_ERPM"},
    {PP_VALS::L_MIN_DUTY, "L_MIN_DUTY"},
    {PP_VALS::L_MAX_DUTY, "L_MAX_DUTY"},
    {PP_VALS::L_WATT_MIN, "L_WATT_MIN"},
    {PP_VALS::L_WATT_MAX, "L_WATT_MAX"},
    {PP_VALS::L_IN_CURRENT_MIN, "L_IN_CURRENT_MIN"},
    {PP_VALS::L_IN_CURRENT_MAX, "L_IN_CURRENT_MAX"},
    {PP_VALS::C_PHASE_CURRENT_MAX, "C_PHASE_CURRENT_MAX"},
    {PP_VALS::C_FORCE_SINGLE_MOTOR_ACCELERATION, "C_FORCE_SINGLE_MOTOR_ACCELERATION"},
  };

  STRING_TO_PP_VALS = {
      {"L_CURRENT_MIN_SCALE", PP_VALS::L_CURRENT_MIN_SCALE},
      {"L_CURRENT_MAX_SCALE", PP_VALS::L_CURRENT_MAX_SCALE},
      {"L_MIN_ERPM", PP_VALS::L_MIN_ERPM},
      {"L_MAX_ERPM", PP_VALS::L_MAX_ERPM},
      {"L_MIN_DUTY", PP_VALS::L_MIN_DUTY},
      {"L_MAX_DUTY", PP_VALS::L_MAX_DUTY},
      {"L_WATT_MIN", PP_VALS::L_WATT_MIN},
      {"L_WATT_MAX", PP_VALS::L_WATT_MAX},
      {"L_IN_CURRENT_MIN", PP_VALS::L_IN_CURRENT_MIN},
      {"L_IN_CURRENT_MAX", PP_VALS::L_IN_CURRENT_MAX},
      {"C_PHASE_CURRENT_MAX", PP_VALS::C_PHASE_CURRENT_MAX},
      {"C_FORCE_SINGLE_MOTOR_ACCELERATION", PP_VALS::C_FORCE_SINGLE_MOTOR_ACCELERATION},
  };

  PROFILE_TO_STRING = {
      {PROFILE::LEGAL, "Legal"},
      {PROFILE::ECO, "Eco"},
      {PROFILE::BALANCED, "Balanced"},
      {PROFILE::PERFORMANCE1, "Performance1"},
      {PROFILE::PERFORMANCE2, "Performance2"},
      {PROFILE::CUSTOM, "Custom"},
  };

  STRING_TO_PROFILE = {
      {"Legal", PROFILE::LEGAL},
      {"Eco", PROFILE::ECO},
      {"Balanced", PROFILE::BALANCED},
      {"Performance1", PROFILE::PERFORMANCE1},
      {"Performance2", PROFILE::PERFORMANCE2},
      {"Custom", PROFILE::CUSTOM},
  };

  // throttle/brake curves
  throttleCurve = {
      {0, 0},
      {8, 8},
      {15, 13},
      {20, 18},
      {30, 30},
      {40, 60},
      {50, 100},
      {75, 180},
      {87, 220},
      {100, 250}
  };

  throttleCurveDualVESC = {
      {0, 0},
      {8, 8},
      {15, 15},
      {20, 25},
      {30, 45},
      {40, 70},
      {50, 110},
      {60, 150},
      {70, 250},
      {80, 350},
      {90, 420},
      {100, 500}
  };

  brakeCurve = {
      {0, 0},
      {8, 8},
      {15, 13},
      {20, 18},
      {30, 30},
      {40, 45},
      {60, 72},
      {75, 105},
      {87, 150},
      {100, 200}
  };

  if (!spiFlashLittleFsInit()) {
    chSysHalt("SPI flash/LittleFS init failed");
  }
  settingsLoad(jsonSettings);

  wheel.rpmPerKmh = (1.0 /*km/h*/ * 1000.0 /*meters*/) / ((3.14 * wheel.diameter) * 60 /*minutes*/) * 100.0 /*?*/;
  motor.rpmPerKmh = wheel.rpmPerKmh * wheel.gear_ratio;

  battery.setup();
  chMtxObjectInit(&mtx_toSendExtra);
  chTMObjectInit(&vesc_tm);

  // GPIO setup
  pinMode(PIN_BUTTON_K1, INPUT_PULLDOWN);

  // VESC FDCAN2: PB12 RX, PB13 TX (AF9) 500 kbit/s.
  if (!VESC.begin(settings.primaryVESCID)) {
    chSysHalt("FDCAN2 start failed");
  }

  setMcconfFromCurrentProfile();

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
