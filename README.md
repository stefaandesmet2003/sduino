# Note! 

this is not a fork from the official sduino repo, but a photo of my sduino package files
and so doesn't include the files for building a release for instance

I uncommented AWU, TIM2, I2C interrupt handlers in stm8_it.h for my personal use.
there are empty weak definitions added to keep the linker happy if the ISRs are not present otherwise (for instance when Wire library). As such the examples continue to build as before.

## Slave I2C
implemented using the framework interrupt handler from ST (AN3281). 
The master I2C code in sduino is untouched.

slave_receiver and slave_sender examples have been updated with the sduino C-style function definitions

added an example, showing STM8 as a I2C serial eeprom. The example demonstrates the use of both onReceive() and onRequest() callbacks



