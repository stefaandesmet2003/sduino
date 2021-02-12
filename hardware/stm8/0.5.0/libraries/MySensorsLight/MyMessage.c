/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */


#include "MyMessage.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*
void MyMessage_init(MyMessage *msg)
{
	MyMessage_clear(msg);
}
*/
void MyMessage_init2(MyMessage *msg, const uint8_t _sensorId, const mysensors_data_t _dataType)
{
	MyMessage_clear(msg);
	(void)MyMessage_setSensor(msg,_sensorId);
	(void)MyMessage_setType(msg, (uint8_t)(_dataType));
}

void MyMessage_clear(MyMessage *msg)
{
	msg->last                 = 0u;
	msg->sender               = 0u;
	msg->destination          = 0u; //GATEWAY_ADDRESS; // Gateway is default destination
	msg->version_length       = 0u;
	msg->command_echo_payload = 0u;
	msg->type                 = 0u;
	msg->sensor               = 0u;
	// clear data buffer
	(void)memset((void *)msg->data, 0u, sizeof(msg->data));

	// set message protocol version
	(void)MyMessage_setVersion(msg);
}

uint8_t MyMessage_getHeaderSize(MyMessage *msg)
{
	(void) msg;
	return (uint8_t)HEADER_SIZE;
}

uint8_t MyMessage_getMaxPayloadSize(MyMessage *msg)
{
	(void) msg;
	return (uint8_t)MAX_PAYLOAD_SIZE;
}

uint8_t MyMessage_getExpectedMessageSize(MyMessage *msg)
{
	return MyMessage_getHeaderSize(msg) + (MyMessage_getSigned(msg) ? MyMessage_getMaxPayloadSize(msg) : MyMessage_getLength(msg));
}

bool MyMessage_isProtocolVersionValid(MyMessage *msg)
{
	return (MyMessage_getVersion(msg) == V2_MYS_HEADER_PROTOCOL_VERSION);
}

uint8_t MyMessage_getType(MyMessage *msg)
{
	return msg->type;
}

MyMessage* MyMessage_setType(MyMessage *msg, const uint8_t messageType)
{
	msg->type = messageType;
	return msg;
}

uint8_t MyMessage_getLast(MyMessage *msg)
{
	return msg->last;
}

MyMessage* MyMessage_setLast(MyMessage *msg, const uint8_t lastId)
{
	msg->last = lastId;
	return msg;
}

uint8_t MyMessage_getSender(MyMessage *msg)
{
	return msg->sender;
}

MyMessage* MyMessage_setSender(MyMessage *msg, const uint8_t senderId)
{
	msg->sender = senderId;
	return msg;
}

uint8_t MyMessage_getSensor(MyMessage *msg)
{
	return msg->sensor;
}

MyMessage* MyMessage_setSensor(MyMessage *msg, const uint8_t sensorId)
{
	msg->sensor = sensorId;
	return msg;
}

uint8_t MyMessage_getDestination(MyMessage *msg)
{
	return msg->destination;
}

MyMessage* MyMessage_setDestination(MyMessage *msg, const uint8_t destinationId)
{
	msg->destination = destinationId;
	return msg;
}

bool MyMessage_isEcho(MyMessage *msg)
{
	return (bool)BF_GET(msg->command_echo_payload, V2_MYS_HEADER_CEP_ECHO_POS,
	                    V2_MYS_HEADER_CEP_ECHO_SIZE);
}

MyMessage* MyMessage_setEcho(MyMessage *msg, const bool echo)
{
	BF_SET(msg->command_echo_payload, echo, V2_MYS_HEADER_CEP_ECHO_POS,
	       V2_MYS_HEADER_CEP_ECHO_SIZE);
	return msg;
}

bool MyMessage_getRequestEcho(MyMessage *msg)
{
	return (bool)BF_GET(msg->command_echo_payload, V2_MYS_HEADER_CEP_ECHOREQUEST_POS,
	                    V2_MYS_HEADER_CEP_ECHOREQUEST_SIZE);
}

MyMessage* MyMessage_setRequestEcho(MyMessage *msg,const bool requestEcho)
{
	BF_SET(msg->command_echo_payload, requestEcho, V2_MYS_HEADER_CEP_ECHOREQUEST_POS,
	       V2_MYS_HEADER_CEP_ECHOREQUEST_SIZE);
	return msg;
}

uint8_t MyMessage_getVersion(MyMessage *msg)
{
	return (uint8_t)BF_GET(msg->version_length, V2_MYS_HEADER_VSL_VERSION_POS,
	                       V2_MYS_HEADER_VSL_VERSION_SIZE);
}

MyMessage* MyMessage_setVersion(MyMessage *msg)
{
	BF_SET(msg->version_length, V2_MYS_HEADER_PROTOCOL_VERSION, V2_MYS_HEADER_VSL_VERSION_POS,
	       V2_MYS_HEADER_VSL_VERSION_SIZE);
	return msg;
}

mysensors_command_t MyMessage_getCommand(MyMessage *msg)
{
	return (mysensors_command_t)(BF_GET(msg->command_echo_payload,
	                                        V2_MYS_HEADER_CEP_COMMAND_POS, V2_MYS_HEADER_CEP_COMMAND_SIZE));
}

MyMessage* MyMessage_setCommand(MyMessage *msg, const mysensors_command_t command)
{
	BF_SET(msg->command_echo_payload, (uint8_t)(command), V2_MYS_HEADER_CEP_COMMAND_POS,
	       V2_MYS_HEADER_CEP_COMMAND_SIZE);
	return msg;
}

mysensors_payload_t MyMessage_getPayloadType(MyMessage *msg)
{
	return (mysensors_payload_t)(BF_GET(msg->command_echo_payload,
	                                        V2_MYS_HEADER_CEP_PAYLOADTYPE_POS, V2_MYS_HEADER_CEP_PAYLOADTYPE_SIZE));
}

MyMessage* MyMessage_setPayloadType(MyMessage *msg, const mysensors_payload_t payloadType)
{
	BF_SET(msg->command_echo_payload, (uint8_t)(payloadType),
	       V2_MYS_HEADER_CEP_PAYLOADTYPE_POS, V2_MYS_HEADER_CEP_PAYLOADTYPE_SIZE);
	return msg;
}

bool MyMessage_getSigned(MyMessage *msg)
{
	return (bool)BF_GET(msg->version_length, V2_MYS_HEADER_VSL_SIGNED_POS,
	                    V2_MYS_HEADER_VSL_SIGNED_SIZE);
}

MyMessage* MyMessage_setSigned(MyMessage *msg, const bool signedFlag)
{
	BF_SET(msg->version_length, signedFlag, V2_MYS_HEADER_VSL_SIGNED_POS,
	       V2_MYS_HEADER_VSL_SIGNED_SIZE);
	return msg;
}

uint8_t MyMessage_getLength(MyMessage *msg)
{
	uint8_t length = BF_GET(msg->version_length, V2_MYS_HEADER_VSL_LENGTH_POS,
	                        V2_MYS_HEADER_VSL_LENGTH_SIZE);
	// limit length
	if (length > MAX_PAYLOAD_SIZE) {
		length = MAX_PAYLOAD_SIZE;
	}
	return length;
}

MyMessage* MyMessage_setLength(MyMessage *msg, const uint8_t length)
{
	uint8_t finalLength = length;
	// limit length
	if (finalLength > MAX_PAYLOAD_SIZE) {
		finalLength = MAX_PAYLOAD_SIZE;
	}

	BF_SET(msg->version_length, finalLength, V2_MYS_HEADER_VSL_LENGTH_POS,
	       V2_MYS_HEADER_VSL_LENGTH_SIZE);
	return msg;
}

/* Getters for payload converted to desired form */
void* MyMessage_getCustom(MyMessage *msg)
{
	return (void *)msg->data;
}

const char* MyMessage_getString(MyMessage *msg)
{
	if (MyMessage_getPayloadType(msg) == P_STRING) {
		return msg->data;
	} else {
		return NULL;
	}
}

char* MyMessage_getStringBuffer(MyMessage *msg, char *buffer)
{
	/*
	if (buffer != NULL) {
		const uint8_t payloadType = MyMessage_getPayloadType(msg);
		if (payloadType == P_STRING) {
			(void)strncpy(buffer, msg->data, MyMessage_getLength(msg));
			buffer[MyMessage_getLength(msg)] = 0;
		} else if (payloadType == P_BYTE) {
			(void)itoa(msg->bValue, buffer, 10);
		} else if (payloadType == P_INT16) {
			(void)itoa(msg->iValue, buffer, 10);
		} else if (payloadType == P_UINT16) {
			(void)utoa(msg->uiValue, buffer, 10);
		} else if (payloadType == P_LONG32) {
			(void)ltoa(msg->lValue, buffer, 10);
		} else if (payloadType == P_ULONG32) {
			(void)ultoa(msg->ulValue, buffer, 10);
		} else if (payloadType == P_FLOAT32) {
			//TODO
			//(void)dtostrf(msg->fValue, 2, min(msg->fPrecision, (uint8_t)8u), buffer);
		} 
		return buffer;
	} else {
		return NULL;
	}
	*/
	//TODO test codesize
	return NULL;
}

bool MyMessage_getBool(MyMessage *msg)
{
	return (bool)MyMessage_getByte(msg);
}

uint8_t MyMessage_getByte(MyMessage *msg)
{
	if (MyMessage_getPayloadType(msg) == P_BYTE) {
		return (uint8_t)msg->data[0];
	} else if (MyMessage_getPayloadType(msg) == P_STRING) {
		//return (uint8_t)atoi(msg->data);
		return 0; // TODO test codesize
	} else {
		return 0;
	}
}

/*
float MyMessage_getFloat(MyMessage *msg)
{
	if (MyMessage_getPayloadType(msg) == P_FLOAT32) {
		return msg->fValue;
	} else if (MyMessage_getPayloadType(msg) == P_STRING) {
		return (float)atof(msg->data);
	} else {
		return 0;
	}
}
*/

int32_t MyMessage_getLong(MyMessage *msg)
{
	if (MyMessage_getPayloadType(msg) == P_LONG32) {
		return msg->lValue;
	} else if (MyMessage_getPayloadType(msg) == P_STRING) {
		return (int32_t)atol(msg->data);
	} else {
		return 0;
	}
}

uint32_t MyMessage_getULong(MyMessage *msg)
{
	if (MyMessage_getPayloadType(msg) == P_ULONG32) {
		return msg->ulValue;
	} else if (MyMessage_getPayloadType(msg) == P_STRING) {
		return (uint32_t)atol(msg->data);
	} else {
		return 0;
	}
}
/*
int16_t MyMessage_getInt(MyMessage *msg)
{
	if (MyMessage_getPayloadType(msg) == P_INT16) {
		return msg->iValue;
	} else if (MyMessage_getPayloadType(msg) == P_STRING) {
		return (int16_t)atoi(msg->data);
	} else {
		return 0;
	}
}

uint16_t MyMessage_getUInt(MyMessage *msg)
{
	if (MyMessage_getPayloadType(msg) == P_UINT16) {
		return msg->uiValue;
	} else if (MyMessage_getPayloadType(msg) == P_STRING) {
		return (uint16_t)atoi(msg->data);
	} else {
		return 0;
	}
}
*/

/*
MyMessage* MyMessage_setCustom(MyMessage *msg, const void* value, const size_t _length)
{
	(void)MyMessage_setLength(msg, (value != NULL) ? _length : 0);
	(void)MyMessage_setPayloadType(msg, P_CUSTOM);
	(void)memcpy((void *)msg->data, value, MyMessage_getLength(msg));
	return msg;
}
*/

MyMessage* MyMessage_setString(MyMessage *msg, const char* value)
{
	(void)MyMessage_setLength(msg,(value != NULL) ? strlen(value) : 0);
	(void)MyMessage_setPayloadType(msg, P_STRING);
	(void)strncpy(msg->data, value, MyMessage_getLength(msg));
	// null terminate string
	msg->data[MyMessage_getLength(msg)] = 0;
	return msg;
}

MyMessage* MyMessage_setBool(MyMessage *msg, const bool value)
{
	return MyMessage_setByte(msg,(uint8_t)value);
}

MyMessage* MyMessage_setByte(MyMessage *msg, const uint8_t value)
{
	(void)MyMessage_setLength(msg,1u);
	(void)MyMessage_setPayloadType(msg,P_BYTE);
	msg->bValue = value;
	return msg;
}

/*
MyMessage* MyMessage_setFloat(MyMessage *msg, const float value, const uint8_t decimals)
{
	(void)MyMessage_setLength(msg,5u); // 32 bit float + persi
	(void)MyMessage_setPayloadType(msg,P_FLOAT32);
	msg->fValue = value;
	msg->fPrecision = decimals;
	return msg;
}
*/

MyMessage* MyMessage_setULong(MyMessage *msg, const uint32_t value)
{
	(void)MyMessage_setLength(msg,4u);
	(void)MyMessage_setPayloadType(msg,P_ULONG32);
	msg->ulValue = value;
	return msg;
}

MyMessage* MyMessage_setLong(MyMessage *msg, const int32_t value)
{
	(void)MyMessage_setLength(msg,4u);
	(void)MyMessage_setPayloadType(msg,P_LONG32);
	msg->lValue = value;
	return msg;
}

MyMessage* MyMessage_setUInt(MyMessage *msg, const uint16_t value)
{
	(void)MyMessage_setLength(msg,2u);
	(void)MyMessage_setPayloadType(msg,P_UINT16);
	msg->uiValue = value;
	return msg;
}

MyMessage* MyMessage_setInt(MyMessage *msg, const int16_t value)
{
	(void)MyMessage_setLength(msg,2u);
	(void)MyMessage_setPayloadType(msg,P_INT16);
	msg->iValue = value;
	return msg;
}
