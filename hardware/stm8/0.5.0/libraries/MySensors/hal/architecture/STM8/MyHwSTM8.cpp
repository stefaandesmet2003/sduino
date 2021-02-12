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

#include "MyHwSTM8.h"

/*
* Pinout STM32F103C8 dev board:
* http://wiki.stm32duino.com/images/a/ae/Bluepillpinout.gif
*
* Wiring RFM69 radio / SPI1
* --------------------------------------------------
* CLK	PA5
* MISO	PA6
* MOSI	PA7
* CSN	PA4
* CE	NA
* IRQ	PA3 (default)
*
* Wiring RF24 radio / SPI1
* --------------------------------------------------
* CLK	PA5
* MISO	PA6
* MOSI	PA7
* CSN	PA4
* CE	PB0 (default)
* IRQ	NA
*
*/

// TODO STM8 (voor P_FLOAT32 message type)
char* dtostrf (double val, signed char width, unsigned char prec, char *s) {
	(void) val;
	(void) width;
	(void) prec;
	memcpy(s,"3.14",5);
}

bool hwInit(void)
{
#if !defined(MY_DISABLED_SERIAL)
	MY_SERIALDEVICE_begin(MY_BAUD_RATE);
#if defined(MY_GATEWAY_SERIAL)
	while (!MY_SERIALDEVICE) {}
#endif
#endif
	return true;
}

int8_t hwSleep(uint32_t ms)
{
	// TODO STM8!
	// TODO: Not supported!
	(void)ms;
	return MY_SLEEP_NOT_POSSIBLE;
}

int8_t hwSleep3(const uint8_t interrupt, const uint8_t mode, uint32_t ms)
{
	// TODO STM8!
	// TODO: Not supported!
	(void)interrupt;
	(void)mode;
	(void)ms;
	return MY_SLEEP_NOT_POSSIBLE;
}

int8_t hwSleep5(const uint8_t interrupt1, const uint8_t mode1, const uint8_t interrupt2,
               const uint8_t mode2,
               uint32_t ms)
{
	// TODO STM8!
	// TODO: Not supported!
	(void)interrupt1;
	(void)mode1;
	(void)interrupt2;
	(void)mode2;
	(void)ms;
	return MY_SLEEP_NOT_POSSIBLE;
}


void hwRandomNumberInit(void)
{
	/* TODO STM8
	// use internal temperature sensor as noise source
	adc_reg_map *regs = ADC1->regs;
	regs->CR2 |= ADC_CR2_TSVREFE;
	regs->SMPR1 |= ADC_SMPR1_SMP16;

	uint32_t seed = 0;
	uint16_t currentValue = 0;
	uint16_t newValue = 0;

	for (uint8_t i = 0; i < 32; i++) {
		const uint32_t timeout = hwMillis() + 20;
		while (timeout >= hwMillis()) {
			newValue = adc_read(ADC1, 16);
			if (newValue != currentValue) {
				currentValue = newValue;
				break;
			}
		}
		seed ^= ( (newValue + hwMillis()) & 7) << i;
	}
	randomSeed(seed);
	regs->CR2 &= ~ADC_CR2_TSVREFE; // disable VREFINT and temp sensor
	*/
}

bool hwUniqueID(unique_id_t *uniqueID)
{
	// TODO STM8
	(void)uniqueID;
	return false; // no unique ID returned
	/*
	(void)memcpy((uint8_t *)uniqueID, (uint32_t *)0x1FFFF7E0, 16); // FlashID + ChipID
	return true;
	*/
}

uint16_t hwCPUVoltage(void)
{
	// TODO STM8
	return 1023;
	/*
	adc_reg_map *regs = ADC1->regs;
	regs->CR2 |= ADC_CR2_TSVREFE; // enable VREFINT and temp sensor
	regs->SMPR1 =  ADC_SMPR1_SMP17; // sample rate for VREFINT ADC channel
	adc_calibrate(ADC1);

	const uint16_t vdd = adc_read(ADC1, 17);
	regs->CR2 &= ~ADC_CR2_TSVREFE; // disable VREFINT and temp sensor
	return (uint16_t)(1200u * 4096u / vdd);
	*/

}

uint16_t hwCPUFrequency(void)
{
	return F_CPU/100000UL;
}

int8_t hwCPUTemperature(void)
{
	// TODO STM8
	return -127; // not available
	/*
	adc_reg_map *regs = ADC1->regs;
	regs->CR2 |= ADC_CR2_TSVREFE; // enable VREFINT and Temperature sensor
	regs->SMPR1 |= ADC_SMPR1_SMP16 | ADC_SMPR1_SMP17;
	adc_calibrate(ADC1);

	//const uint16_t adc_temp = adc_read(ADC1, 16);
	//const uint16_t vref = 1200 * 4096 / adc_read(ADC1, 17);
	// calibrated at 25°C, ADC output = 1430mV, avg slope = 4.3mV / °C, increasing temp ~ lower voltage
	const int8_t temp = static_cast<int8_t>((1430.0 - (adc_read(ADC1, 16) * 1200 / adc_read(ADC1,
	                                        17))) / 4.3 + 25.0);
	regs->CR2 &= ~ADC_CR2_TSVREFE; // disable VREFINT and temp sensor
	return (temp - MY_STM32F1_TEMPERATURE_OFFSET) / MY_STM32F1_TEMPERATURE_GAIN;
	*/
}

uint16_t hwFreeMem(void)
{
	//Not yet implemented
	return FUNCTION_NOT_SUPPORTED;
}
