// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.
/* 2021 - modified for sduino
 * Note 1:
 * -> receiveEvent is called from ISR context; calling Serial_print can give rise to a deadlock:
 * -> if serial buffer is full, Serial_print will busy wait for the UART-TX ISR to send some characters
 * -> but this can't work from the TWI ISR, because all ISR's have same priority in sduino
 * -> the only reason is works here is because we are serial printing less data than the SERIAL_BUFFER_SIZE
 * Note 2: it's important to Wire_read all received bytes in receiveEvent (conform arduino Wire implementation)
 *  otherwise receiveEvent is not called on the next frame (rxBufferIndex < rxBufferLength in Wire.c)!
*/

#include <Wire.h>

void receiveEvent(int howMany);

void setup() {
  Wire_beginSlave(8);                // join i2c bus with address #8
  Wire_onReceive(receiveEvent); // register event
  Serial_begin(115200);           // start serial for output
}

void loop() {
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  
  while (1 < Wire_available()) { // loop through all but the last
    char c = Wire_read(); // receive byte as a character
    Serial_print_c(c);         // print the character
  }
  int x = Wire_read();    // receive byte as an integer
  Serial_println_u(x);         // print the integer
  
}
