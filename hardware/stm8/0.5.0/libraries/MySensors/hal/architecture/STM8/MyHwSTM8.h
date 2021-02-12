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

#ifndef MyHwSTM8_h
#define MyHwSTM8_h

/* TODO
#include <libmaple/iwdg.h>
#include <itoa.h>
#include <EEPROM.h>
#include <SPI.h>
*/
#include <EEPROM.h>
#include <SPI.h>

#ifdef __cplusplus
#include <Arduino.h>
#endif

#define CRYPTO_LITTLE_ENDIAN

#ifndef MY_SERIALDEVICE
#define MY_SERIALDEVICE Serial
#define MY_SERIALDEVICE_begin Serial_begin
#define MY_SERIALDEVICE_flush Serial_flush

#endif

#ifndef MY_DEBUGDEVICE
#define MY_DEBUGDEVICE MY_SERIALDEVICE
#endif

#ifndef MY_STM8_TEMPERATURE_OFFSET
#define MY_STM8_TEMPERATURE_OFFSET (0.0f)
#endif

#ifndef MY_STM8_TEMPERATURE_GAIN
#define MY_STM8_TEMPERATURE_GAIN (1.0f)
#endif

// sdccc doesn't understand this
#define __attribute__(X)

// TODO : nodig voor stm8?
/*
// SS default
#ifndef SS
#define SS PA4
#endif
*/

// TODO!
// mapping
#define snprintf_P snprintf
#define vsnprintf_P vsnprintf
#define strncpy_P strncpy
#define printf_P printf

#ifndef digitalPinToInterrupt
#define digitalPinToInterrupt(__pin) (__pin)
#endif

#define hwDigitalWrite(__pin, __value) digitalWrite(__pin, __value)
#define hwDigitalRead(__pin) digitalRead(__pin)
#define hwPinMode(__pin, __value) pinMode(__pin, __value)
// TODO!
#define hwWatchdogReset()
#define hwReboot()
#define hwMillis() millis()
#define hwGetSleepRemaining() (0ul)

// TODO implement this!
char* dtostrf (double val, signed char width, unsigned char prec, char *s);

void serialEventRun(void);
bool hwInit(void);
void hwRandomNumberInit(void);
#define hwReadConfig(__pos) EEPROM_read((uint16_t)__pos)
#define hwWriteConfig(__pos, __val) EEPROM_update((uint16_t)__pos, (uint8_t)__val)
// for some weird reason sduino changed the order of parameters for these functions
#define hwReadConfigBlock(__buf, __pos, __length) eeprom_read_block((const uint16_t)__pos, (uint8_t *)__buf, (size_t)__length)
#define hwWriteConfigBlock(__buf, __pos, __length) eeprom_update_block((uint16_t)__pos, (uint8_t *)__buf, (size_t)__length)

// SOFTSPI
#ifdef MY_SOFTSPI
#error Soft SPI is not available on this architecture!
#endif
#define hwSPI SPI //!< hwSPI


#ifndef DOXYGEN
#define MY_CRITICAL_SECTION
#endif  /* DOXYGEN */

#endif
