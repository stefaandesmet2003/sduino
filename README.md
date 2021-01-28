# Note! 

this is not a fork from the official sduino repo, but a photo of the sduino package files
and so doesn't include the files for building a release for instance

AWU, TIM2, I2C interrupt handlers are uncommented in stm8_it.h; 
for now they will need to be defined in user code for linking:

INTERRUPT_HANDLER(AWU_IRQHandler, ITC_IRQ_AWU) {}
INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, ITC_IRQ_TIM2_OVF) {}
INTERRUPT_HANDLER(I2C_IRQHandler, 19){}
