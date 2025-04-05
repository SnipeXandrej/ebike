#include "hal/gpio_ll.h"

void pinDigitalWrite(int pin, int state) {
    if (state) {
        GPIO.out_w1ts = ((uint32_t)1 << pin);
        // REG_WRITE(GPIO_OUT_W1TS_REG, pin);
    } else {
        GPIO.out_w1tc = ((uint32_t)1 << pin);
        // REG_WRITE(GPIO_OUT_W1TC_REG, pin);
    }

// switch (pin) {
//     case 1:
//         int espPin = BIT1;
//         break;
//     case 2:
//         int espPin = BIT2;
//         break;
// }

}

bool pinDigitalRead(int pin) {
    return (REG_READ(GPIO_IN_REG) & pin);
}

int pinAnalogRead(int pin) {
    return (REG_READ(GPIO_IN1_REG) & pin);
}
