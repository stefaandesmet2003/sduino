/*
 * a basic implementation of tone(), noTone() on STM8, using TIM2
 * TIM4 is used for millis() / micros()
 * TIM2 = 16-bit general purpose, TIM1 = 16-bit advanced timer
 * -> use TIM2
 * 
 * STM8S103K3, STM8S103F3, STM8S103F2 have TIM1,TIM2,TIM4
 * Note STM8 TIM2 is 16-bit timer <> AVR timer2 is 8-bit timer
 */
#include "Arduino.h"
#include "pins_arduino.h"

// timerx_toggle_count:
//  > 0 - duration specified
//  = 0 - stopped
//  < 0 - infinitely (until noTone() method called, or another tone() called)

volatile long timer2_toggle_count;
volatile uint8_t *timer2_pin_port;
volatile uint8_t timer2_pin_mask;

// kept for consistency with AVR implementation ..
#define AVAILABLE_TONE_PINS 1
#define USE_TIMER2

static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255 /*, 255, 255, 255, 255, 255 */ };

static int8_t toneBegin(uint8_t _pin)
{
  // if we're already using the pin, the timer should not (?) be configured.  
  if (tone_pins[0] == _pin) {
    return 2;
  }
  tone_pins[0] = _pin;

  // Set timer specific stuff
  TIM2->CR1 = 0; // stop counter
  TIM2->IER = 0; // disable update interrupt

  timer2_pin_port = portOutputRegister(digitalPinToPort(_pin));
  timer2_pin_mask = digitalPinToBitMask(_pin);

  return 2; // always using TIM2 for STM8
}

// frequency (in hertz) and duration (in milliseconds).
void tone(uint8_t _pin, unsigned int frequency, unsigned long duration)
{
  long toggle_count = 0;
  uint32_t arr = 0;
  uint8_t psc = 0;

  toneBegin(_pin);

  // Set the pinMode as OUTPUT
  pinMode(_pin, OUTPUT);

  // keep it simple : two choices for the 16 bit timers: ck/1 or ck/64
  arr = F_CPU / frequency / 2 - 1;
  if (arr > 0xFFFF)
  {
    arr = F_CPU / frequency / 2 / 64 - 1;
    psc = 6; // prescale 2^6 = 64
    if (arr > 0xFFFF) arr = 0xFFFF; // corner case for 1Hz/2Hz
  }

  // Calculate the toggle count
  if (duration > 0)
   toggle_count = 2 * frequency * duration / 1000;
  else
    toggle_count = -1;

  timer2_toggle_count = toggle_count;

  TIM2->CR1 = TIM2_CR1_URS; // =4, counter disable & URS=1
  TIM2->PSCR = psc;
  TIM2->ARRH = (uint8_t)((arr>>8) & 0xFF); // write MSB first
  TIM2->ARRL = (uint8_t) (arr & 0xFF); 
  TIM2->CNTRH = 0;
  TIM2->CNTRL = 0;
  TIM2->EGR = 1; // UG, forced update of ARRH & PSCR, not waiting for UEV, URS=1, so no update interrupt after this manual update
  TIM2->SR1 = 0; // clear interrupts, just to be sure there are no outstanding ints
  TIM2->IER = TIM2_IER_UIE; // =1, update interrupt enable
  TIM2->CR1 |= TIM2_CR1_CEN; // counter enable (we could disable URS again, don't need it anymore)
}

void disableTimer(uint8_t _timer)
{
  (void) _timer; // kept for consistency with avr implementation
  TIM2->CR1 = 0; // disable counter
}

void noTone(uint8_t _pin)
{
  tone_pins[0] = 255;
  disableTimer(2);
  digitalWrite(_pin, 0);
}

#ifdef USE_TIMER2
INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, ITC_IRQ_TIM2_OVF) {
  TIM2->SR1 = 0;

  if (timer2_toggle_count != 0)
  {
    // toggle the pin
    *timer2_pin_port ^= timer2_pin_mask;

    if (timer2_toggle_count > 0)
      timer2_toggle_count--;
  }
  else
  {
    // need to call noTone() so that the tone_pins[] entry is reset, so the
    // timer gets initialized next time we call tone().
    // XXX: this assumes timer 2 is always the first one used.
    noTone(tone_pins[0]);
//    disableTimer(2);
//    *timer2_pin_port &= ~(timer2_pin_mask);  // keep pin low after stop
  }
}
#endif

