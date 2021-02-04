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

#include "hal/transport/RFM95/driver/RFM95.h"

bool transportInit(void)
{
	const bool result = RFM95_initialise(MY_RFM95_FREQUENCY);
#if defined(MY_RFM95_TCXO)
	RFM95_enableTCXO();
#endif
#if !defined(MY_GATEWAY_FEATURE) && !defined(MY_RFM95_ATC_MODE_DISABLED)
	// only enable ATC mode in nodes
	RFM95_ATCmode(true, MY_RFM95_ATC_TARGET_RSSI);
#endif
	return result;
}

void transportSetAddress(const uint8_t address)
{
	RFM95_setAddress(address);
}

uint8_t transportGetAddress(void)
{
	return RFM95_getAddress();
}

bool transportSend(const uint8_t to, const void *data, const uint8_t len, const bool noACK)
{
	return RFM95_sendWithRetry(to, data, len, noACK);
}

bool transportDataAvailable(void)
{
	RFM95_handler();
	return RFM95_available();
}

bool transportSanityCheck(void)
{
	return RFM95_sanityCheck();
}

uint8_t transportReceive(void *data)
{
	uint8_t len = RFM95_receive((uint8_t *)data, MAX_MESSAGE_SIZE);
	return len;
}

void transportSleep(void)
{
	(void)RFM95_sleep();
}

void transportStandBy(void)
{
	(void)RFM95_standBy();
}

void transportPowerDown(void)
{
	RFM95_powerDown();
}

void transportPowerUp(void)
{
	RFM95_powerUp();
}

void transportToggleATCmode(const bool OnOff, const int16_t targetRSSI)
{
	RFM95_ATCmode(OnOff, targetRSSI);
}

int16_t transportGetSendingRSSI(void)
{
	return RFM95_getSendingRSSI();
}

int16_t transportGetReceivingRSSI(void)
{
	return RFM95_getReceivingRSSI();
}

int16_t transportGetSendingSNR(void)
{
	return RFM95_getSendingSNR();
}

int16_t transportGetReceivingSNR(void)
{
	return RFM95_getReceivingSNR();
}

int16_t transportGetTxPowerPercent(void)
{
	return RFM95_getTxPowerPercent();
}

int16_t transportGetTxPowerLevel(void)
{
	return RFM95_getTxPowerLevel();
}

bool transportSetTxPowerPercent(const uint8_t powerPercent)
{
	return RFM95_setTxPowerPercent(powerPercent);
}

