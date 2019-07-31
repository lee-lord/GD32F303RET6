#ifndef _PWM_PPM_CAPTURE_H_
#define _PWM_PPM_CAPTURE_H_
#include "sysType.h"
void PWM_PPM_inInitial(void);
void timer_captur_configuration(U32 FrqHz);
void timer_capture_polarity_set(uint32_t timer_periph,U16 channel,U16 polarity);
extern volatile uint16_t PWM1Value, PWM2Value;

#endif
