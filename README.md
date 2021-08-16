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

## NeoPixel
a rudimentary port of the Adafruit_NeoPixel library  
supports 800kHz strips @ 16MHz clock frequency (62.5ns cycle time). 
Other configurations will result in wrong output timing and won't work correctly.

The output signal timing is not completely according to WS2812B specification:  
- 0 & 1 timings are ok, 
- but bit0 time is app. 2100ns (34cycles) , ie. longer than 1850ns, due to additional setup (instruction cycles) after each byte

Code works for my led strip, so didn't bother to unfold the asm code per bit (as eg. in https://github.com/thielj/FastLED-STM8/tree/master/platforms/stm8) to achieve better bit timing.

Note : Adafruit library has lots of functions, but SDCC doesn't optimize the unused ones. 

## MySensors Light - work in progress
A lightweight version of the popular MySensors library ported to STM8.

Library is using the RF24 code integrated with MySensors; other transports are not included (yet)

Some functionality of MySensors has been dropped, in order to fit in 8kB flash:
- no gateway functionality (makes no sense on STM8)
- no dynamic routing: a fixed parentID can to be assigned (default=GATEWAY)
- not all types are ported, in particular string buffer, to avoid expensive stdlib functions.

STM8Node shows a working example.

Todo : 
- code size optimization. For the moment the working example takes 94% of 8kB flash. Probably need to drop some more MySensors functionality
- integrate smart sleep code for low power nodes



