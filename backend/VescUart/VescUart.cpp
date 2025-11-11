#include <stdint.h>
#include "VescUart.h"

VescUart::VescUart(uint32_t timeout_ms) : _TIMEOUT(timeout_ms) {
	nunchuck.valueX         = 127;
	nunchuck.valueY         = 127;
	nunchuck.lowerButton  	= false;
	nunchuck.upperButton  	= false;
}

void VescUart::setSerialPort(MyUart* port)
{
	serialPort = port;
}

void VescUart::setDebugPort(MyUart* port)
{
	debugPort = port;
}

int VescUart::receiveUartMessage(uint8_t * payloadReceived) {

	// Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	// Makes no sense to run this function if no serialPort is defined.
	if (serialPort == NULL)
		return -1;

	mtx.lock();

	uint16_t counter = 0;
	uint16_t endMessage = 256;
	bool messageRead = false;
	bool packetSizeExceeded = false;
	uint8_t messageReceived[256];
	uint16_t lenPayload = 0;
	
	uint32_t timeout = millis() + _TIMEOUT; // Defining the timestamp for timeout (100ms before timeout)

	while ( millis() < timeout && messageRead == false && packetSizeExceeded == false) {

		while (serialPort->available()) {

			messageReceived[counter++] = serialPort->read();

			if (counter == 2) {

				switch (messageReceived[0])
				{
					case 2:
						endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
						lenPayload = messageReceived[1];
					break;

					case 3:
						// ToDo: Add Message Handling > 255 (starting with 3)
						if( printDebug ){
							printf("%s\n", "Message is larger than 256 bytes - not supported");
						}
					break;

					default:
						if( printDebug ){
							printf("%s\n", "Unvalid start bit");
						}
					break;
				}
			}

			if (counter >= sizeof(messageReceived)) {
				packetSizeExceeded = true;
				break;
			}

			if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
				messageReceived[endMessage] = 0;
				if (printDebug) {
					printf("%s\n", "End of message reached!");
				}
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}
	if(messageRead == false && printDebug ) {
		printf("%s\n", "Timeout");
	}
	
	bool unpacked = false;

	if (messageRead) {
		unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
	}

	uint16_t ret;

	if (unpacked) {
		// Message was read
		ret = lenPayload;
	}
	else {
		// No Message Read
		ret = 0;
	}

	mtx.unlock();
	return ret;
}


bool VescUart::unpackPayload(uint8_t * message, int lenMes, uint8_t * payload) {

	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	if(printDebug){
		printf("SRC received: "); printf("%d", crcMessage);
	}

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	if( printDebug ){
		printf("SRC calc: "); printf("%d", crcPayload);
	}
	
	if (crcPayload == crcMessage) {
		if( printDebug ) {
			printf("Received: "); 
			serialPrint(message, lenMes);

			printf("Payload :      ");
			serialPrint(payload, message[1] - 1);
		}

		return true;
	}else{
		return false;
	}
}

int VescUart::packSendPayload(uint8_t * payload, int lenPay) {
	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];
	
	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(messageSend + count, payload, lenPay);
	count += lenPay;

	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	// messageSend[count] = NULL;
	
	if(printDebug){
		printf("Package to send: ");
		serialPrint(messageSend, count);
	}

	// Sending package
	if (serialPort != NULL)
		serialPort->write(messageSend, count);

	// Returns number of send bytes
	return count;
}


bool VescUart::processReadPacket(uint8_t * message) {

	COMM_PACKET_ID packetId;
	int32_t index = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_FW_VERSION: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			fw_version.major = message[index++];
			fw_version.minor = message[index++];
			return true;
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			data.tempMosfet 		= buffer_get_float16(message, 10.0, &index); 	// 2 bytes - mc_interface_temp_fet_filtered()
			data.tempMotor 			= buffer_get_float16(message, 10.0, &index); 	// 2 bytes - mc_interface_temp_motor_filtered()
			data.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_motor_current()
			data.avgInputCurrent 	= buffer_get_float32(message, 100.0, &index); // 4 bytes - mc_interface_read_reset_avg_input_current()
			index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_id()
			index += 4; // Skip 4 bytes - mc_interface_read_reset_avg_iq()
			data.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &index); 	// 2 bytes - mc_interface_get_duty_cycle_now()
			data.rpm 				= buffer_get_float32(message, 1.0, &index);		// 4 bytes - mc_interface_get_rpm()
			data.inpVoltage 		= buffer_get_float16(message, 10.0, &index);		// 2 bytes - GET_INPUT_VOLTAGE()
			data.ampHours 			= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours(false)
			data.ampHoursCharged 	= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_amp_hours_charged(false)
			data.wattHours			= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours(false)
			data.wattHoursCharged	= buffer_get_float32(message, 10000.0, &index);	// 4 bytes - mc_interface_get_watt_hours_charged(false)
			data.tachometer 		= buffer_get_int32(message, &index);				// 4 bytes - mc_interface_get_tachometer_value(false)
			data.tachometerAbs 		= buffer_get_int32(message, &index);				// 4 bytes - mc_interface_get_tachometer_abs_value(false)
			data.error 				= (mc_fault_code)message[index++];								// 1 byte  - mc_interface_get_fault()
			data.pidPos				= buffer_get_float32(message, 1000000.0, &index);	// 4 bytes - mc_interface_get_pid_pos_now()
			data.id					= message[index++];								// 1 byte  - app_get_configuration()->controller_id	

			return true;

		case COMM_GET_MCCONF_TEMP: // Structure defined here: https://github.com/vedderb/bldc/blob/f7c2b34e1cff2234cae98be3abf0cd50e249558f/comm/commands.c#L1051

			data_mcconf.l_current_min_scale = buffer_get_float32_auto(message, &index);
			data_mcconf.l_current_max_scale = buffer_get_float32_auto(message, &index);
			data_mcconf.l_min_erpm = buffer_get_float32_auto(message, &index);
			data_mcconf.l_max_erpm = buffer_get_float32_auto(message, &index);
			data_mcconf.l_min_duty = buffer_get_float32_auto(message, &index);
			data_mcconf.l_max_duty = buffer_get_float32_auto(message, &index);
			data_mcconf.l_watt_min = buffer_get_float32_auto(message, &index);
			data_mcconf.l_watt_max = buffer_get_float32_auto(message, &index);
			data_mcconf.l_in_current_min = buffer_get_float32_auto(message, &index);
			data_mcconf.l_in_current_max = buffer_get_float32_auto(message, &index);

			// data_mcconf.motor_poles = (uint8_t)message[index++];
			// index += 4; // Skip 4 bytes // data_mcconf.gear_ratio = buffer_get_float32_auto(message, &index);
			// index += 4; // Skip 4 bytes // data_mcconf.wheel_diameter = buffer_get_float32_auto(message, &index);

			return true;
		break;

		/* case COMM_GET_VALUES_SELECTIVE:

			uint32_t mask = 0xFFFFFFFF; */

		default:
			return false;
		break;
	}
}

bool VescUart::getFWversion(void){
	return getFWversion(0);
}

bool VescUart::getFWversion(uint8_t canId){
	
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_FW_VERSION };

	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);

	if (messageLength > 0) { 
		return processReadPacket(message); 
	}
	return false;
}

bool VescUart::getVescValues(void) {
	return getVescValues(0);
}

bool VescUart::getVescValues(uint8_t canId) {

	if (printDebug){
		printf("Command: COMM_GET_VALUES %d\n", canId);
	}

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_GET_VALUES };

	packSendPayload(payload, payloadSize);

	// printf("1\n");

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);

	// printf("2 %d\n", messageLength);

	if (messageLength > 0) {
		return processReadPacket(message); 
	}

	// printf("3\n");
	return false;
}

bool VescUart::getMcconfTempValues(void) {
	return getMcconfTempValues(0);
}

bool VescUart::getMcconfTempValues(uint8_t canId) {

	if (printDebug){
		printf("Command: COMM_GET_MCCONF_TEMP %d", canId);
	}

	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_GET_MCCONF_TEMP };

	packSendPayload(payload, payloadSize);

	uint8_t message[256];
	int messageLength = receiveUartMessage(message);

	if (messageLength > 0) {
		return processReadPacket(message);
	}
	return false;
}

void VescUart::setMcconfTempValues() {
	return setMcconfTempValues(0);
}

void VescUart::setMcconfTempValues(uint8_t canId) {

	if(printDebug){
		printf("Command: COMM_SET_MCCONF_TEMP_SETUP %d", canId);
	}
	int32_t index = 0;
	int payloadSize = 45; // 37
	uint8_t payload[payloadSize];

	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}

	payload[index++] = { COMM_SET_MCCONF_TEMP_SETUP }; // 1 byte
	payload[index++] = 0; // buffer_append_bool(payload, false, &index);
	payload[index++] = 0; // buffer_append_bool(payload, false, &index);
	payload[index++] = 0; // buffer_append_bool(payload, false, &index);
	payload[index++] = 0; // buffer_append_bool(payload, false, &index);
	buffer_append_float32_auto(payload, data_mcconf.l_current_min_scale, &index); // 4 bytes
	buffer_append_float32_auto(payload, data_mcconf.l_current_max_scale, &index);
	buffer_append_float32_auto(payload, data_mcconf.l_min_erpm, &index);
	buffer_append_float32_auto(payload, data_mcconf.l_max_erpm, &index);
	buffer_append_float32_auto(payload, data_mcconf.l_min_duty, &index);
	buffer_append_float32_auto(payload, data_mcconf.l_max_duty, &index);
	buffer_append_float32_auto(payload, data_mcconf.l_watt_min, &index);
	buffer_append_float32_auto(payload, data_mcconf.l_watt_max, &index);
	buffer_append_float32_auto(payload, data_mcconf.l_in_current_min, &index);
	buffer_append_float32_auto(payload, data_mcconf.l_in_current_max, &index);

	// Serial.printf("index size: %ld\n", index);

	// payload[index++] = data_mcconf.l_current_min_scale;
	// payload[index++] = data_mcconf.l_current_max_scale;
	// payload[index++] = data_mcconf.l_min_erpm;
	// payload[index++] = data_mcconf.l_max_erpm;
	// payload[index++] = data_mcconf.l_min_duty;
	// payload[index++] = data_mcconf.l_max_duty;
	// payload[index++] = data_mcconf.l_watt_min;
	// payload[index++] = data_mcconf.l_watt_max;  // 36??
	// // payload[index++] = data_mcconf.l_in_current_min;
	// // payload[index++] = data_mcconf.l_in_current_max;

	packSendPayload(payload, payloadSize);
}

void VescUart::setNunchuckValues() {
	return setNunchuckValues(0);
}

void VescUart::setNunchuckValues(uint8_t canId) {

	if(printDebug){
		printf("Command: COMM_SET_CHUCK_DATA %d\n", canId);
	}	
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 11 : 13);
	uint8_t payload[payloadSize];

	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_CHUCK_DATA };
	payload[index++] = nunchuck.valueX;
	payload[index++] = nunchuck.valueY;
	buffer_append_bool(payload, nunchuck.lowerButton, &index);
	buffer_append_bool(payload, nunchuck.upperButton, &index);
	
	// Acceleration Data. Not used, Int16 (2 byte)
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;
	payload[index++] = 0;

	if(printDebug){
		printf("Nunchuck Values:");
		printf("x="); printf("%d", nunchuck.valueX); printf(" y="); printf("%d\n", nunchuck.valueY);
		printf(" LBTN="); printf("%d", nunchuck.lowerButton); printf(" UBTN="); printf("%d\n", nunchuck.upperButton);
	}

	packSendPayload(payload, payloadSize);
}

void VescUart::setCurrent(float current) {
	return setCurrent(current, 0);
}

void VescUart::setCurrent(float current, uint8_t canId) {
	if(printDebug){
		printf("Command: COMM_SET_CURRENT %d\n", canId);
	}
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_CURRENT };
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::setBrakeCurrent(float brakeCurrent) {
	return setBrakeCurrent(brakeCurrent, 0);
}

void VescUart::setBrakeCurrent(float brakeCurrent, uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}

	payload[index++] = { COMM_SET_CURRENT_BRAKE };
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

	packSendPayload(payload, payloadSize);
}

void VescUart::setRPM(float rpm) {
	return setRPM(rpm, 0);
}

void VescUart::setRPM(float rpm, uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_RPM };
	buffer_append_int32(payload, (int32_t)(rpm), &index);
	packSendPayload(payload, payloadSize);
}

void VescUart::setDuty(float duty) {
	return setDuty(duty, 0);
}

void VescUart::setDuty(float duty, uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 5 : 7);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_SET_DUTY };
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, payloadSize);
}

void VescUart::sendKeepalive(void) {
	return sendKeepalive(0);
}

void VescUart::sendKeepalive(uint8_t canId) {
	int32_t index = 0;
	int payloadSize = (canId == 0 ? 1 : 3);
	uint8_t payload[payloadSize];
	if (canId != 0) {
		payload[index++] = { COMM_FORWARD_CAN };
		payload[index++] = canId;
	}
	payload[index++] = { COMM_ALIVE };
	packSendPayload(payload, payloadSize);
}

void VescUart::serialPrint(uint8_t * data, int len) {
	if(printDebug){
		for (int i = 0; i <= len; i++)
		{
			printf("%d\n", data[i]);
		}
	}
}

unsigned int VescUart::millis (void) {
  uint64_t now ;

#ifdef	OLD_WAY
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ;

#else
  struct  timespec ts ;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  now  = (uint64_t)ts.tv_sec * (uint64_t)1000 + (uint64_t)(ts.tv_nsec / 1000000L) ;
#endif

  return (uint32_t)(now - epochMilli) ;
}

void VescUart::printVescValues() {
	if(printDebug){
		printf("avgMotorCurrent: "); 	printf("%f\n", data.avgMotorCurrent);
		printf("avgInputCurrent: "); 	printf("%f\n", data.avgInputCurrent);
		printf("dutyCycleNow: "); 	printf("%f\n", data.dutyCycleNow);
		printf("rpm: "); 				printf("%f\n", data.rpm);
		printf("inputVoltage: "); 	printf("%f\n", data.inpVoltage);
		printf("ampHours: "); 		printf("%f\n", data.ampHours);
		printf("ampHoursCharged: "); 	printf("%f\n", data.ampHoursCharged);
		printf("wattHours: "); 		printf("%f\n", data.wattHours);
		printf("wattHoursCharged: "); printf("%f\n", static_cast<float>(data.wattHoursCharged));
		printf("tachometer: "); 		printf("%ld\n", data.tachometer);
		printf("tachometerAbs: "); 	printf("%f\n", static_cast<float>(data.tachometerAbs));
		printf("tempMosfet: "); 		printf("%f\n", data.tempMosfet);
		printf("tempMotor: "); 		printf("%f\n", data.tempMotor);
		// printf("error: "); 			printf(data.error);
	}
}
