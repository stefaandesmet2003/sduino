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
*
*/

#include "MyCryptoSTM8.h"

// SHA256 implementation in ASM, see MyASM.S
void SHA256(uint8_t *dest, const uint8_t *data, size_t dataLength)
{
  (void)dest;
  (void)data;
  (void)dataLength;
}

// SHA256HMAC
void SHA256HMAC(uint8_t *dest, const uint8_t *key, size_t keyLength, const uint8_t *data,
                size_t dataLength)
{
  (void)dest;
  (void)key;
  (void)keyLength;
  (void)data;
  (void)dataLength;
}


// AES
void AES128CBCInit(const uint8_t *key)
{
  (void)key;
}

void AES128CBCEncrypt(uint8_t *iv, uint8_t *buffer, const size_t dataLength)
{
  (void)iv;
  (void)buffer;
  (void)dataLength;
}

void AES128CBCDecrypt(uint8_t *iv, uint8_t *buffer, const size_t dataLength)
{
  (void)iv;
  (void)buffer;
  (void)dataLength;
}
