// CLANKED ALERT MASSIVE CLANKED ALERT

#include "vescCan.hpp"

extern "C" {
#include <libVescCan/VESC.h>
}

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

// FDCAN2 uses the 25 MHz HSE clock. Prescaler 5 and 10 time quanta produce
// 500 kbit/s with an 80 percent sample point.
const CANConfig canConfig = {
    OPMODE_CAN,
    FDCAN_CONFIG_NBTP_NSJW(1U) |
        FDCAN_CONFIG_NBTP_NTSEG2(1U) |
        FDCAN_CONFIG_NBTP_NTSEG1(6U) |
        FDCAN_CONFIG_NBTP_NBRP(4U),
    0U,
    0U,
    0U,
    0U,
    0U,
};

THD_WORKING_AREA(vescCanRxWorkingArea, 1024);

THD_FUNCTION(vescCanRxThread, arg) {
  auto *vesc = static_cast<VescCan *>(arg);
  chRegSetThreadName("VESC CAN RX");
  vesc->receiveLoop();
}

uint16_t crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0;
  while (length-- > 0U) {
    crc ^= static_cast<uint16_t>(*data++) << 8U;
    for (unsigned bit = 0; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

void appendUint32(uint8_t *buffer, uint32_t value, size_t &index) {
  buffer[index++] = static_cast<uint8_t>(value >> 24U);
  buffer[index++] = static_cast<uint8_t>(value >> 16U);
  buffer[index++] = static_cast<uint8_t>(value >> 8U);
  buffer[index++] = static_cast<uint8_t>(value);
}

void appendFloat32Auto(uint8_t *buffer, float value, size_t &index) {
  if (std::fabs(value) < 1.5e-38F) {
    value = 0.0F;
  }

  int exponent = 0;
  const float significand = std::frexp(value, &exponent);
  const float magnitude = std::fabs(significand);
  uint32_t encodedSignificand = 0;
  if (magnitude >= 0.5F) {
    encodedSignificand = static_cast<uint32_t>(
        (magnitude - 0.5F) * 2.0F * 8388608.0F);
    exponent += 126;
  }

  uint32_t encoded = (static_cast<uint32_t>(exponent & 0xFF) << 23U) |
                     (encodedSignificand & 0x7FFFFFU);
  if (significand < 0.0F) {
    encoded |= 1UL << 31U;
  }
  appendUint32(buffer, encoded, index);
}

uint32_t readUint32(const uint8_t *buffer, size_t &index) {
  const uint32_t value = (static_cast<uint32_t>(buffer[index]) << 24U) |
                         (static_cast<uint32_t>(buffer[index + 1U]) << 16U) |
                         (static_cast<uint32_t>(buffer[index + 2U]) << 8U) |
                         static_cast<uint32_t>(buffer[index + 3U]);
  index += 4U;
  return value;
}

int32_t readInt32(const uint8_t *buffer, size_t &index) {
  return static_cast<int32_t>(readUint32(buffer, index));
}

int16_t readInt16(const uint8_t *buffer, size_t &index) {
  const uint16_t value = (static_cast<uint16_t>(buffer[index]) << 8U) |
                         buffer[index + 1U];
  index += 2U;
  return static_cast<int16_t>(value);
}

float readFloat32Auto(const uint8_t *buffer, size_t &index) {
  const uint32_t encoded = readUint32(buffer, index);
  int exponent = static_cast<int>((encoded >> 23U) & 0xFFU);
  const uint32_t encodedSignificand = encoded & 0x7FFFFFU;
  const bool negative = (encoded & (1UL << 31U)) != 0U;

  float significand = 0.0F;
  if (exponent != 0 || encodedSignificand != 0U) {
    significand = static_cast<float>(encodedSignificand) /
                      (8388608.0F * 2.0F) +
                  0.5F;
    exponent -= 126;
  }
  return std::ldexp(negative ? -significand : significand, exponent);
}

} // namespace

VescCan::VescCan() {
  chMtxObjectInit(&cacheMutex_);
  chMtxObjectInit(&txMutex_);
  chMtxObjectInit(&mcconfRequestMutex_);
  chBSemObjectInit(&mcconfReceived_, true);
}

bool VescCan::begin(uint8_t defaultVescId) {
  defaultVescId_ = defaultVescId;

  // STM32H723: PB12=FDCAN2_RX and PB13=FDCAN2_TX on AF9.
  palSetLineMode(PAL_LINE(GPIOB, 12U),
                 PAL_MODE_ALTERNATE(9U) | PAL_STM32_OSPEED_HIGHEST);
  palSetLineMode(PAL_LINE(GPIOB, 13U),
                 PAL_MODE_ALTERNATE(9U) | PAL_STM32_OSPEED_HIGHEST);

  if (canStart(canp_, &canConfig) != MSG_OK) {
    return false;
  }

  chThdCreateStatic(vescCanRxWorkingArea, sizeof(vescCanRxWorkingArea),
                    NORMALPRIO + 1, vescCanRxThread, this);
  return true;
}

void VescCan::setCurrent(float current) {
  setCurrent(current, defaultVescId_);
}

void VescCan::setCurrent(float current, uint8_t canId) {
  sendMotorCommand(VESC_COMMAND_SET_CURRENT, current, canId);
}

void VescCan::setBrakeCurrent(float current) {
  setBrakeCurrent(current, defaultVescId_);
}

void VescCan::setBrakeCurrent(float current, uint8_t canId) {
  sendMotorCommand(VESC_COMMAND_SET_CURRENT_BRAKE, current, canId);
}

void VescCan::setDuty(float duty) {
  setDuty(duty, defaultVescId_);
}

void VescCan::setDuty(float duty, uint8_t canId) {
  sendMotorCommand(VESC_COMMAND_SET_DUTY, duty, canId);
}

void VescCan::setRPM(float rpm) {
  setRPM(rpm, defaultVescId_);
}

void VescCan::setRPM(float rpm, uint8_t canId) {
  sendMotorCommand(VESC_COMMAND_SET_RPM, rpm, canId);
}

bool VescCan::sendMotorCommand(uint8_t command, float value, uint8_t canId) {
  VESC_CommandFrame commandFrame{};
  commandFrame.vescID = canId;
  commandFrame.command = command;
  commandFrame.commandData = value;

  VESC_RawFrame raw{};
  if (!VESC_convertCmdToRaw(&raw, &commandFrame)) {
    return false;
  }
  return sendRaw((static_cast<uint32_t>(raw.command) << 8U) | raw.vescID,
                 raw.rawData, raw.can_dlc);
}

bool VescCan::sendRaw(uint32_t extendedId, const uint8_t *payload,
                      size_t length) {
  if (length > 8U) {
    return false;
  }

  CANTxFrame frame{};
  frame.ext.EID = extendedId & 0x1FFFFFFFU;
  frame.common.XTD = 1U;
  frame.common.RTR = 0U;
  frame.FDF = 0U;
  frame.DLC = static_cast<uint32_t>(length);
  if (length != 0U) {
    std::memcpy(frame.data8, payload, length);
  }

  chMtxLock(&txMutex_);
  txAttempts_ = txAttempts_ + 1U;
  const msg_t result = canTransmit(canp_, CAN_ANY_MAILBOX, &frame,
                                   TIME_MS2I(10));
  if (result == MSG_OK) {
    txCompleted_ = txCompleted_ + 1U;
  } else {
    txTimeouts_ = txTimeouts_ + 1U;
  }
  chMtxUnlock(&txMutex_);
  return result == MSG_OK;
}

VescCan::Diagnostics VescCan::getDiagnostics() const {
  Diagnostics result;
  result.txAttempts = txAttempts_;
  result.txCompleted = txCompleted_;
  result.txTimeouts = txTimeouts_;
  result.rxFrames = rxFrames_;
  result.busOffRecoveries = busOffRecoveries_;
  result.errorCounter = canp_->fdcan->ECR;
  result.protocolStatus = canp_->fdcan->PSR;
  return result;
}

bool VescCan::sendBuffer(uint8_t canId, const uint8_t *payload,
                         size_t length, uint8_t sendMode) {
  // VESC CAN_PACKET_FILL_RX_BUFFER and CAN_PACKET_PROCESS_RX_BUFFER.
  constexpr uint8_t fillRxBuffer = 5U;
  constexpr uint8_t fillRxBufferLong = 6U;
  constexpr uint8_t processRxBuffer = 7U;

  const size_t bytesPerFragment = length <= 255U ? 7U : 6U;
  for (size_t offset = 0; offset < length; offset += bytesPerFragment) {
    uint8_t fragment[8]{};
    const size_t fragmentLength =
        std::min(bytesPerFragment, length - offset);
    size_t headerLength;
    uint8_t packetType;
    if (length <= 255U) {
      fragment[0] = static_cast<uint8_t>(offset);
      headerLength = 1U;
      packetType = fillRxBuffer;
    } else {
      fragment[0] = static_cast<uint8_t>(offset >> 8U);
      fragment[1] = static_cast<uint8_t>(offset);
      headerLength = 2U;
      packetType = fillRxBufferLong;
    }
    std::memcpy(&fragment[headerLength], &payload[offset], fragmentLength);
    if (!sendRaw((static_cast<uint32_t>(packetType) << 8U) | canId,
                 fragment, fragmentLength + headerLength)) {
      return false;
    }
  }

  const uint16_t crc = crc16(payload, length);
  const uint8_t process[] = {
      kHostCanId,
      sendMode,
      static_cast<uint8_t>(length >> 8U),
      static_cast<uint8_t>(length),
      static_cast<uint8_t>(crc >> 8U),
      static_cast<uint8_t>(crc),
  };
  return sendRaw((static_cast<uint32_t>(processRxBuffer) << 8U) | canId,
                 process, sizeof(process));
}

bool VescCan::sendPacket(uint8_t canId, const uint8_t *packet, size_t length) {
  if (packet == nullptr || length == 0U || length > sizeof(transportBuffer_)) {
    return false;
  }
  if (length <= 6U) {
    uint8_t shortPacket[8] = {kHostCanId, 2U};
    std::memcpy(&shortPacket[2], packet, length);
    return sendRaw((8UL << 8U) | canId, shortPacket, length + 2U);
  }
  return sendBuffer(canId, packet, length, 2U);
}

bool VescCan::requestPacket(uint8_t canId, const uint8_t *request,
                            size_t requestLength, uint8_t *response,
                            size_t responseCapacity, size_t &responseLength,
                            uint32_t timeoutMs) {
  responseLength = 0U;
  if (request == nullptr || requestLength == 0U || response == nullptr ||
      responseCapacity == 0U || requestLength > sizeof(transportBuffer_)) {
    return false;
  }

  chMtxLock(&mcconfRequestMutex_);
  (void)chBSemWaitTimeout(&mcconfReceived_, TIME_IMMEDIATE);
  pendingResponseSource_ = canId;
  pendingResponseCommand_ = request[0];
  responseLength_ = 0U;
  responsePending_ = true;

  bool sent;
  if (requestLength <= 6U) {
    uint8_t shortPacket[8] = {kHostCanId, 0U};
    std::memcpy(&shortPacket[2], request, requestLength);
    sent = sendRaw((8UL << 8U) | canId, shortPacket, requestLength + 2U);
  } else {
    sent = sendBuffer(canId, request, requestLength, 0U);
  }

  const bool received = sent &&
      chBSemWaitTimeout(&mcconfReceived_, TIME_MS2I(timeoutMs)) == MSG_OK;
  if (received) {
    responseLength = std::min(responseCapacity,
                              static_cast<size_t>(responseLength_));
    std::memcpy(response, responseBuffer_, responseLength);
  }
  responsePending_ = false;
  chMtxUnlock(&mcconfRequestMutex_);
  return received;
}

bool VescCan::deliverResponse(uint8_t source, const uint8_t *payload,
                              size_t length) {
  if (!responsePending_ || source != pendingResponseSource_ || length == 0U ||
      payload[0] != pendingResponseCommand_ || length > sizeof(responseBuffer_)) {
    return false;
  }
  std::memcpy(responseBuffer_, payload, length);
  responseLength_ = length;
  chBSemSignal(&mcconfReceived_);
  return true;
}

bool VescCan::setMcconfTempValues() {
  return setMcconfTempValues(defaultVescId_);
}

bool VescCan::setMcconfTempValues(uint8_t canId) {
  // COMM_SET_MCCONF_TEMP_SETUP. Keep this packet aligned with the VESC
  // firmware command: four flags followed by ten auto-encoded floats.
  constexpr uint8_t commSetMcconfTempSetup = 49U;
  uint8_t payload[45]{};
  size_t index = 0;
  payload[index++] = commSetMcconfTempSetup;
  payload[index++] = 0U; // store
  payload[index++] = 0U; // forward_can
  payload[index++] = 1U; // Request an application-level acknowledgement.
  payload[index++] = 0U; // divide_by_controllers
  appendFloat32Auto(payload, data_mcconf.l_current_min_scale, index);
  appendFloat32Auto(payload, data_mcconf.l_current_max_scale, index);
  appendFloat32Auto(payload, data_mcconf.l_min_erpm, index);
  appendFloat32Auto(payload, data_mcconf.l_max_erpm, index);
  appendFloat32Auto(payload, data_mcconf.l_min_duty, index);
  appendFloat32Auto(payload, data_mcconf.l_max_duty, index);
  appendFloat32Auto(payload, data_mcconf.l_watt_min, index);
  appendFloat32Auto(payload, data_mcconf.l_watt_max, index);
  appendFloat32Auto(payload, data_mcconf.l_in_current_min, index);
  appendFloat32Auto(payload, data_mcconf.l_in_current_max, index);
  uint8_t response[8];
  size_t responseLength = 0U;
  return requestPacket(canId, payload, index, response, sizeof(response),
                       responseLength, 100U) &&
         responseLength >= 1U && response[0] == commSetMcconfTempSetup;
}

bool VescCan::getMcconfTempValues() {
  return getMcconfTempValues(defaultVescId_);
}

bool VescCan::getMcconfTempValues(uint8_t canId) {
  constexpr uint8_t commGetMcconfTemp = 91U;
  const uint8_t request[] = {commGetMcconfTemp};
  uint8_t response[64];
  size_t length = 0U;
  if (!requestPacket(canId, request, sizeof(request), response,
                     sizeof(response), length) || length < 41U) {
    return false;
  }
  size_t index = 1U;
  data_mcconf.l_current_min_scale = readFloat32Auto(response, index);
  data_mcconf.l_current_max_scale = readFloat32Auto(response, index);
  data_mcconf.l_min_erpm = readFloat32Auto(response, index);
  data_mcconf.l_max_erpm = readFloat32Auto(response, index);
  data_mcconf.l_min_duty = readFloat32Auto(response, index);
  data_mcconf.l_max_duty = readFloat32Auto(response, index);
  data_mcconf.l_watt_min = readFloat32Auto(response, index);
  data_mcconf.l_watt_max = readFloat32Auto(response, index);
  data_mcconf.l_in_current_min = readFloat32Auto(response, index);
  data_mcconf.l_in_current_max = readFloat32Auto(response, index);
  return true;
}

VescCan::CacheEntry *VescCan::findOrAllocate(uint8_t canId) {
  CacheEntry *freeEntry = nullptr;
  for (auto &entry : cache_) {
    if (entry.used && entry.id == canId) {
      return &entry;
    }
    if (!entry.used && freeEntry == nullptr) {
      freeEntry = &entry;
    }
  }
  if (freeEntry != nullptr) {
    freeEntry->used = true;
    freeEntry->id = canId;
    freeEntry->data.id = canId;
  }
  return freeEntry;
}

bool VescCan::getVescValues(uint8_t canId) {
  constexpr uint8_t commGetValues = 4U;
  const uint8_t request[] = {commGetValues};
  uint8_t response[96];
  size_t length = 0U;
  if (!requestPacket(canId, request, sizeof(request), response,
                     sizeof(response), length, 20U) || length < 59U) {
    return false;
  }

  size_t index = 1U;
  dataPackage received{};
  received.tempMosfet = static_cast<float>(readInt16(response, index)) / 10.0F;
  received.tempMotor = static_cast<float>(readInt16(response, index)) / 10.0F;
  received.avgMotorCurrent = static_cast<float>(readInt32(response, index)) / 100.0F;
  received.avgInputCurrent = static_cast<float>(readInt32(response, index)) / 100.0F;
  received.avgCurrentDAxis = static_cast<float>(readInt32(response, index)) / 100.0F;
  received.avgCurrentQAxis = static_cast<float>(readInt32(response, index)) / 100.0F;
  received.dutyCycleNow = static_cast<float>(readInt16(response, index)) / 1000.0F;
  received.rpm = static_cast<float>(readInt32(response, index));
  received.inpVoltage = static_cast<float>(readInt16(response, index)) / 10.0F;
  received.ampHours = static_cast<float>(readInt32(response, index)) / 10000.0F;
  received.ampHoursCharged = static_cast<float>(readInt32(response, index)) / 10000.0F;
  received.wattHours = static_cast<float>(readInt32(response, index)) / 10000.0F;
  received.wattHoursCharged = static_cast<float>(readInt32(response, index)) / 10000.0F;
  received.tachometer = readInt32(response, index);
  received.tachometerAbs = readInt32(response, index);
  received.error = response[index++];
  received.pidPos = static_cast<float>(readInt32(response, index)) / 1000000.0F;
  received.id = response[index];

  chMtxLock(&cacheMutex_);
  CacheEntry *entry = findOrAllocate(canId);
  if (entry != nullptr) {
    entry->data = received;
    entry->lastUpdate = chVTGetSystemTimeX();
  }
  data = received;
  chMtxUnlock(&cacheMutex_);
  return true;
}

void VescCan::processFrame(const CANRxFrame &frame) {
  if (frame.common.XTD == 0U || frame.common.RTR != 0U || frame.DLC > 8U) {
    return;
  }

  if (processTransportFrame(frame)) {
    return;
  }

  VESC_RawFrame raw{};
  raw.vescID = static_cast<uint8_t>(frame.ext.EID & 0xFFU);
  raw.command = static_cast<uint8_t>((frame.ext.EID >> 8U) & 0xFFU);
  raw.can_dlc = static_cast<uint8_t>(frame.DLC);
  std::memcpy(raw.rawData, frame.data8, raw.can_dlc);

  chMtxLock(&cacheMutex_);
  CacheEntry *entry = findOrAllocate(raw.vescID);
  if (entry == nullptr) {
    chMtxUnlock(&cacheMutex_);
    return;
  }

  bool decoded = false;
  switch (raw.command) {
  case VESC_COMMAND_STATUS_1: {
    VESC_Status_1 status{};
    decoded = VESC_convertRawToStatus1(&status, &raw);
    if (decoded) {
      entry->data.rpm = status.erpm;
      entry->data.avgMotorCurrent = status.current;
      entry->data.avgCurrentQAxis = status.current;
      entry->data.dutyCycleNow = status.dutyCycle;
    }
    break;
  }
  case VESC_COMMAND_STATUS_2: {
    VESC_Status_2 status{};
    decoded = VESC_convertRawToStatus2(&status, &raw);
    if (decoded) {
      entry->data.ampHours = status.apmHours;
      entry->data.ampHoursCharged = status.apmHoursChg;
    }
    break;
  }
  case VESC_COMMAND_STATUS_3: {
    VESC_Status_3 status{};
    decoded = VESC_convertRawToStatus3(&status, &raw);
    if (decoded) {
      entry->data.wattHours = status.wattHours;
      entry->data.wattHoursCharged = status.wattHoursChg;
    }
    break;
  }
  case VESC_COMMAND_STATUS_4: {
    VESC_Status_4 status{};
    decoded = VESC_convertRawToStatus4(&status, &raw);
    if (decoded) {
      entry->data.tempMosfet = status.tempFet;
      entry->data.tempMotor = status.tempMotor;
      entry->data.avgInputCurrent = status.currentIn;
      entry->data.pidPos = status.pidPos;
    }
    break;
  }
  case VESC_COMMAND_STATUS_5: {
    VESC_Status_5 status{};
    decoded = VESC_convertRawToStatus5(&status, &raw);
    if (decoded) {
      entry->data.tachometer = static_cast<long>(status.tachometer);
      entry->data.tachometerAbs = static_cast<long>(std::fabs(status.tachometer));
      entry->data.inpVoltage = status.voltsIn;
    }
    break;
  }
  default:
    break;
  }

  if (decoded) {
    entry->lastUpdate = chVTGetSystemTimeX();
  }
  chMtxUnlock(&cacheMutex_);
}

bool VescCan::processTransportFrame(const CANRxFrame &frame) {
  constexpr uint8_t fillRxBuffer = 5U;
  constexpr uint8_t fillRxBufferLong = 6U;
  constexpr uint8_t processRxBuffer = 7U;
  constexpr uint8_t processShortBuffer = 8U;

  const uint8_t destination = static_cast<uint8_t>(frame.ext.EID & 0xFFU);
  const uint8_t command = static_cast<uint8_t>((frame.ext.EID >> 8U) & 0xFFU);
  if (destination != kHostCanId) {
    return false;
  }

  if (command == fillRxBuffer && frame.DLC >= 1U) {
    const size_t offset = frame.data8[0];
    const size_t count = frame.DLC - 1U;
    if (offset + count <= sizeof(transportBuffer_)) {
      std::memcpy(&transportBuffer_[offset], &frame.data8[1], count);
    }
    return true;
  }

  if (command == fillRxBufferLong && frame.DLC >= 2U) {
    const size_t offset = (static_cast<size_t>(frame.data8[0]) << 8U) |
                          frame.data8[1];
    const size_t count = frame.DLC - 2U;
    if (offset + count <= sizeof(transportBuffer_)) {
      std::memcpy(&transportBuffer_[offset], &frame.data8[2], count);
    }
    return true;
  }

  if (command == processShortBuffer && frame.DLC >= 3U) {
    const uint8_t source = frame.data8[0];
    (void)deliverResponse(source, &frame.data8[2], frame.DLC - 2U);
    return true;
  }

  if (command != processRxBuffer || frame.DLC < 6U) {
    return false;
  }

  const uint8_t source = frame.data8[0];
  const size_t length = (static_cast<size_t>(frame.data8[2]) << 8U) |
                        frame.data8[3];
  const uint16_t expectedCrc =
      (static_cast<uint16_t>(frame.data8[4]) << 8U) | frame.data8[5];
  if (length == 0U || length > sizeof(transportBuffer_) ||
      crc16(transportBuffer_, length) != expectedCrc) {
    return true;
  }
  (void)deliverResponse(source, transportBuffer_, length);
  return true;
}

void VescCan::receiveLoop() {
  event_listener_t rxListener;
  event_listener_t errorListener;
  chEvtRegister(&canp_->rxfull_event, &rxListener, 0);
  chEvtRegister(&canp_->error_event, &errorListener, 1);
  while (true) {
    (void)chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100));

    const eventflags_t errors = chEvtGetAndClearFlags(&errorListener);
    if ((errors & CAN_BUS_OFF_ERROR) != 0U) {
      // The Bosch FDCAN core sets CCCR.INIT on bus-off and does not resume by
      // itself. Reinitialize it so connecting/powering the other CAN node
      // after this board has booted does not require a board reset.
      chMtxLock(&txMutex_);
      canStop(canp_);
      if (canStart(canp_, &canConfig) == MSG_OK) {
        busOffRecoveries_ = busOffRecoveries_ + 1U;
      }
      chMtxUnlock(&txMutex_);
    }

    (void)chEvtGetAndClearFlags(&rxListener);
    CANRxFrame frame{};
    while (canReceive(canp_, CAN_ANY_MAILBOX, &frame, TIME_IMMEDIATE) == MSG_OK) {
      rxFrames_ = rxFrames_ + 1U;
      processFrame(frame);
    }
  }
}
