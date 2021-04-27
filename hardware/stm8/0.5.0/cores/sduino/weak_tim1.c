/**
 * "weak" functions to be used if no user-defined function is defined.
 *
 * This function stub is compiled into the core library. Functions from
 * libraries are only linked if a referenced function isnt't defined any other
 * object file. This way all library function are a kind of "weak" functions.
 *
 * this interrupt handler is prototyped in stm8s_it.h and needs a definition
 * to avoid linker error
 * If there is no other definition of the interrupt handler in user code
 * this empty definition is pulled in
 * 
 */

#include "stm8s.h"
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, ITC_IRQ_TIM1_OVF) {}

