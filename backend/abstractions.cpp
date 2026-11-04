#include "abstractions.hpp"

void pinMode(ioline_t pin_line, int mode) {
  palSetLineMode(pin_line, mode);
}

void digitalWrite(ioline_t pin_line, bool state) {
  if (state) {
    palSetLine(pin_line);
  } else {
    palClearLine(pin_line);
  }
}

void digitalToggle(ioline_t pin_line) {
  if (palReadLine(pin_line)) {
    palClearLine(pin_line);
  } else {
    palSetLine(pin_line);
  }
}

ioportmask_t digitalRead(ioline_t pin_line) {
    return palReadLine(pin_line);
}