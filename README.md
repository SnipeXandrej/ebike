# E-BIKE
This repository houses the source code (and later the schematics/boards) for my E-BIKE project that uses the Raspberry Pi 3B+ v1.2 as the main computer, plus some peripherals to make it all work.

Most of the stuff I want to implement is already implemented, but of course there's still the rest of the stuff that's not implemented, or not even wired up, so I don't bother implementing it right now. But I will get to it soon enough, I hope :)

# Hardware
The main hardware that's needed (or that I used) for this contraption to work
- Raspberry Pi 3B+ v1.2 (The main computer)
- Flipsky 75200 V2 (VESC Motor driver)
- MCP3008   (SPI 8 channel 10bit ADC)
- MCP23017  (I2C 16 channel GPIO Expander)
- ADS1115   (I2C 4/2 channel 16bit ADC, used for battery current readout)

# Software
The codebase is separated into two parts, the backend and the frontend, to keep things contained.

Raspbian OS Lite (Debian 13) is used as the Operating System.

You will also need to have the WiringPi library installed, as thats used for the backend.

Dear ImGui with SDL3 + OpenGL 2 is used for the frontend.

## Status of features
- [x] Motor temperature
- [x] VESC MOSFET temperature
- [x] Speed (calculated from the motor speed)
- [x] Odometer
- [x] Trip A / Trip B with Wh/km usage separate for both trips
- [x] Voltage reading
- [x] Current reading
- [x] Immediate Wh/km usage
- [x] Battery Percentage calculated from amphour usage
- [ ] Battery cycle counter
- [x] Immediate Wh/km usage
- [x] Longterm Wh/km usage
- [x] Estimated range
- [x] Software poweroff
- [ ] Hardware poweroff (disconnect everything from battery, to prevent and battery drain)
- [x] Runtime changing of power profiles
- [ ] Lights
    - [x] Front light
    - [ ] Back light
    - [ ] Turn lights
- [ ] Bike unlocking (through a card? NFC? 1-Wire?)
- [ ] Water resistant
- [x] 100km range (at maybe like 30-35km/h)
- [ ] Charge the battery with USB-C PD Charger
- [ ] Charge the battery with a custom power supply
- [ ] Onboard USB-C PD Charger 65W + 18W (USB-C + USB-A) (for a phone for example)
- [ ] Logging of stats like the battery voltage, current, motor speed, phase current, etc.
- [ ] Screen recording
