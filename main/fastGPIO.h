void pinDigitalWrite(int BITpinnumber, int state) {
    if (state) {
        REG_WRITE(GPIO_OUT_W1TS_REG, BITpinnumber);
    } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, BITpinnumber);
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
    return 0;
}
