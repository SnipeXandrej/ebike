// CLANKED ALERT MASSIVE CLANKED ALERT

#pragma once

#include "ch.h"
#include "hal.h"

#include <cstddef>
#include <cstdint>

class VescCan {
public:
  struct Diagnostics {
    uint32_t txAttempts = 0;
    uint32_t txCompleted = 0;
    uint32_t txTimeouts = 0;
    uint32_t rxFrames = 0;
    uint32_t busOffRecoveries = 0;
    uint32_t errorCounter = 0;
    uint32_t protocolStatus = 0;
  };

  struct mcconf_t {
    float l_current_min_scale = 0.0F;
    float l_current_max_scale = 0.0F;
    float l_min_erpm = 0.0F;
    float l_max_erpm = 0.0F;
    float l_min_duty = 0.0F;
    float l_max_duty = 0.0F;
    float l_watt_min = 0.0F;
    float l_watt_max = 0.0F;
    float l_in_current_min = 0.0F;
    float l_in_current_max = 0.0F;
    const char *name = "Default";
    float c_phase_current_max = 0.0F;
    bool c_force_single_motor_acceleration = false;
  };

  struct dataPackage {
    float avgMotorCurrent = 0.0F;
    float avgInputCurrent = 0.0F;
    float avgCurrentQAxis = 0.0F;
    float avgCurrentDAxis = 0.0F;
    float dutyCycleNow = 0.0F;
    float rpm = 0.0F;
    float inpVoltage = 0.0F;
    float ampHours = 0.0F;
    float ampHoursCharged = 0.0F;
    float wattHours = 0.0F;
    float wattHoursCharged = 0.0F;
    long tachometer = 0;
    long tachometerAbs = 0;
    float tempMosfet = 0.0F;
    float tempMotor = 0.0F;
    float pidPos = 0.0F;
    uint8_t id = 0;
    uint8_t error = 0;
  };

  VescCan();

  bool begin(uint8_t defaultVescId);

  void setCurrent(float current);
  void setCurrent(float current, uint8_t canId);
  void setBrakeCurrent(float current);
  void setBrakeCurrent(float current, uint8_t canId);
  void setDuty(float duty);
  void setDuty(float duty, uint8_t canId);
  void setRPM(float rpm);
  void setRPM(float rpm, uint8_t canId);

  // Generic VESC commands.c transport. The packet begins with a COMM_PACKET_ID.
  bool sendPacket(uint8_t canId, const uint8_t *packet, size_t length);
  bool requestPacket(uint8_t canId, const uint8_t *request, size_t requestLength,
                     uint8_t *response, size_t responseCapacity,
                     size_t &responseLength, uint32_t timeoutMs = 100U);

  bool getVescValues(uint8_t canId);
  bool getMcconfTempValues();
  bool getMcconfTempValues(uint8_t canId);
  bool setMcconfTempValues();
  bool setMcconfTempValues(uint8_t canId);
  Diagnostics getDiagnostics() const;

  void receiveLoop();

  dataPackage data;
  mcconf_t data_mcconf;

private:
  struct CacheEntry {
    bool used = false;
    uint8_t id = 0;
    systime_t lastUpdate = 0;
    dataPackage data;
  };

  static constexpr size_t kCacheEntries = 16;
  static constexpr uint8_t kHostCanId = 254;

  bool sendMotorCommand(uint8_t command, float value, uint8_t canId);
  bool sendRaw(uint32_t extendedId, const uint8_t *payload, size_t length);
  bool sendBuffer(uint8_t canId, const uint8_t *payload, size_t length,
                  uint8_t sendMode);
  bool deliverResponse(uint8_t source, const uint8_t *payload, size_t length);
  void processFrame(const CANRxFrame &frame);
  bool processTransportFrame(const CANRxFrame &frame);
  CacheEntry *findOrAllocate(uint8_t canId);

  CANDriver *canp_ = &CAND2;
  uint8_t defaultVescId_ = 0;
  mutex_t cacheMutex_;
  mutex_t txMutex_;
  mutex_t mcconfRequestMutex_;
  binary_semaphore_t mcconfReceived_;
  uint8_t transportBuffer_[512]{};
  uint8_t responseBuffer_[512]{};
  volatile size_t responseLength_ = 0;
  volatile uint8_t pendingResponseSource_ = 0;
  volatile uint8_t pendingResponseCommand_ = 0;
  volatile bool responsePending_ = false;
  CacheEntry cache_[kCacheEntries];
  volatile uint32_t txAttempts_ = 0;
  volatile uint32_t txCompleted_ = 0;
  volatile uint32_t txTimeouts_ = 0;
  volatile uint32_t rxFrames_ = 0;
  volatile uint32_t busOffRecoveries_ = 0;
};
