// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library
// SDS 2021 : modified for STM8

#include <NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        PD2 

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 16 // Popular NeoPixel ring size

#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

void setup() {
  // When setting up the NeoPixel library, we tell it how many pixels,
  // and which pin to use to send signals. Note that for older NeoPixel
  // strips you might need to change the third parameter -- see the
  // strandtest example for more information on possible values.
  NeoPixel_init (NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
  NeoPixel_begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

void loop() {
  NeoPixel_clear(); // Set all pixel colors to 'off'

  // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    NeoPixel_setPixelColor2(i, NeoPixel_Color(0, 150, 0));
    NeoPixel_show();   // Send the updated pixel colors to the hardware.

    delay(DELAYVAL); // Pause before next pass through loop
  }
}
