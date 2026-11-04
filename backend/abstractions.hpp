#ifndef APP_ABSTRACTIONS_HPP
#define APP_ABSTRACTIONS_HPP

#include "hal.h"

#define OUTPUT          PAL_MODE_OUTPUT_PUSHPULL
#define INPUT_PULLDOWN  PAL_MODE_INPUT_PULLDOWN
#define INPUT_PULLUP    PAL_MODE_INPUT_PULLUP

void pinMode(ioline_t pin_line, int mode);
void digitalWrite(ioline_t pin_line, bool state);
void digitalToggle(ioline_t pin_line);
ioportmask_t digitalRead(ioline_t pin_line);

#endif
