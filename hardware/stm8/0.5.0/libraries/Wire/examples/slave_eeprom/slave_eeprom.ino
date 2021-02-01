/*
 * demo code showing STM8 as a 640-byte serial I2C eeprom
 * behaves like AT24C32, but only 640 bytes
 * using extEEPROM library : instantiate as: extEEPROM eep(kbits_256, 1, 16); on a I2C master device.
 * uses STM8 slave i2c and EEPROM libraries, and shows the use of both receiveEvent and requestEvent callbacks
*/

#include <Wire.h>
#include <EEPROM.h>

#define I2C_ADDRESS 0x50 // typical address for I2C serial eeproms

void receiveEvent(int numBytes);
void requestEvent(void);

uint16_t refAddress = 0;

// function that executes whenever data is requested from master
void requestEvent () {
  // let's just deliver 32 bytes = max size for I2C buffer
  for (int i=0;i<32;i++){
    Wire_write(EEPROM_read(refAddress++));
  }
}

// function that executes whenever data is received from master
void receiveEvent(int numBytes) {
  // Wire library expects us to read the complete buffer
  // first 2 bytes : high & low byte of eeprom address
  uint8_t rxBytes[32], aByte;
  uint8_t rxIndex = 0;
  while (Wire_available()) {
    aByte = Wire_read();
    if (rxIndex < 32)
      rxBytes[rxIndex++] = aByte;
  }

  if (numBytes >= 2)
    refAddress = (rxBytes[0] << 8) | rxBytes[1];
  if (numBytes > 2) {
    for (rxIndex=2;rxIndex<numBytes;rxIndex++) {
      EEPROM_update(refAddress++,rxBytes[rxIndex]); // EEPROM lib prevents writing to non-existing addresses, no need to check here
    }
  }
  // if numBytes == 2 -> we expect a repeated start and a requestEvent will provide the data read from eeprom
  // if numBytes < 2 -> not according to spec
}

void setup() {
  Wire_beginSlave(I2C_ADDRESS); // join i2c bus with this address
  Wire_onReceive(receiveEvent); // register event
  Wire_onRequest(requestEvent); // register event
}

void loop() {
}
