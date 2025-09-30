#ifndef _AS5048_H_
#define _AS5048_H_

#include "sysinc.h"

#define AS5048_TIMERX TIM3
#define AS5048_HW_GPIOx HW_GPIOA
#define AS5048_GPIO_Pin_x GPIO_Pin_12
#define ECHO PAin(12)

void AS5048_pwm_init(void);
u16 get_AS5048_pwm_value(void);

#endif
