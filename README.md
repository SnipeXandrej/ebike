# E-BIKE
This repository houses the source code (and later the schematics/boards) for my E-BIKE project that uses the STM32H723 as the main ECU.

Most of the stuff I want to implement is already implemented, but of course there's still the rest of the stuff that's not implemented, or not even wired up, so I don't bother implementing it right now. But I will get to it soon enough, I hope :)

# Hardware
The main hardware that's needed (or that I used) for this contraption to work
- WeAct Studio STM32H723 MCU
- 2x Flipsky 75200 V2 (VESC Motor drivers)
- several INA241A3 current-sensing ICs

# Software
The codebase is separated into two parts, the backend and the frontend, to keep things contained.

ChibiOS is used as the Operating System on the STM32.

Dear ImGui with SDL3 + OpenGL 2 is used for the frontend.

## Status of features
- [x] Motor temperature
- [x] VESC MOSFET temperature
- [ ] Outside temperature
- [x] Speed (calculated from the motor speed)
- [x] Odometer
- [x] Trip A / Trip B with Wh/km usage separate for both trips
- [x] Voltage reading
- [x] Current reading
- [x] Immediate Wh/km usage
- [x] Battery Percentage calculated from amphour usage
- [x] Battery cycle counter
- [ ] External battery
- [x] Immediate Wh/km usage
- [x] Longterm Wh/km usage
- [x] Estimated range
- [x] Software poweroff
- [ ] Hardware poweroff (disconnect everything from battery, to prevent battery drain)
- [x] Runtime changing of power profiles
- [ ] Lights
    - [x] Front light (physical switch)
    - [ ] Back light
    - [ ] Turn lights
- [ ] Motor fan cooling
- [ ] Bike unlocking (through a card? NFC? 1-Wire?)
- [ ] Water resistant
- [x] 100km range (at maybe like 30km/h)
- [ ] Charge the battery with USB-C PD Charger
- [x] Charge the battery with a custom power supply (84V 16A Max)
- [ ] Onboard USB-C PD Charger 65W + 18W (USB-C + USB-A) (for a phone for example)
- [ ] Logging of stats like the battery voltage, current, motor speed, phase current, etc.
