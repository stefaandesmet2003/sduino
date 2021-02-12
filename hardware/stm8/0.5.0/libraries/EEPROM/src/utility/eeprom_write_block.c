/*
  EEPROM.h - EEPROM library
  Plain-C version for SDuino by Michael Mayer 2018.

  This is a rewrite from scratch. The basic API is inspired by the
  Arduino EEPROM library, but none of the operator overloading functions
  can be ported in any sensible way to C.

  Original Copyright (c) 2006 David A. Mellis.  All right reserved.
  New version by Christopher Andrews 2015.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <string.h>
#include "EEPROM.h"


/**
 * write data into EEPROM area
 *
 * The EEPROM area is unlocked (if needed) and re-locked after the write.
 * Data is written byte-wise, word programming or block programming is not
 * supported.
 *
 */
void eeprom_write_block(uint16_t __dst, const uint8_t *__src, size_t len)
{
	// make sure not to write data beyond the end of the EEPROM area
	// (this could accidentally hit the option byte area)
	if (__dst >= EEPROM_end()) return;
	if (len+__dst > EEPROM_end()) {
		len = EEPROM_end() - __dst;
	}

	eeprom_unlock();
	if (eeprom_is_unlocked())
	{
		// write only after a successful unlock.
		memcpy((void*)(((uint16_t)FLASH_DATA_START_PHYSICAL_ADDRESS)+__dst), __src, len);
		// re-lock the EEPROM again.
		FLASH->IAPSR &= FLASH_FLAG_DUL;
	}
}
