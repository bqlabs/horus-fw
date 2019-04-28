# Horus 3D Scanner Firmware for WAVGAT + ZUMSCAN boards

This firmware is derived from Horus firmware v 2.0
Derived from Grbl v0.9 by Jesús Arroyo (Mundo Reader S.L.)
Grbl's lead developer is Simen Svale Skogsrud. Sonney Jeon (Chamnit) improved some parts of grbl.

Firmware adds extra initialization code to setup WAVGAT boards. 
It also should be compatible with generic arduino boards.

This firmware get back some grbl features stripped out by Horus/Ciclop:
- multiple axis support (X,Y,Z)
- limit switches and homing
- G0, G4 G Codes support

So you can use this firmware with your experimental hardware.

To use second stepper on ZUMSCAN shield with this firmware it is needed to rewire two pins on the shield board: 
- desolder or cut pins 6,7 on ZUMSCAN shield so they not connected to arduino
- wire 6->9, 7->10


## Features

*   Angular stepper motor movement
*   Interrupt based movement with real angular acceleration
*   Laser modules control
*   Analog sensor read
*   Configuration interface with $ commands
*   Up to 3 steppers
*   Use limit switches

The default baudrate is 115200.


## Implemented G Codes

*   G0,G1- Angular movement
*   G4   - Dwell (delay)
*   G50  - Reset all positions to zero
*   M0   - Program pause
*   M2   - Program end and reset
*   M17  - Enable/Power stepper motor
*   M18  - Disable stepper motor
*   M50  - Read LDR
*   M70  - Laser off
*   M71  - Laser on

## Build

### Arduino

Open *horus-fw.ino*, select your board

Adjust configuration for your hardware:

  - axis resolution (defaults.h, section DEFAULTS_HORUS)
    For Ciclop hardware with Nema-17 stepper and x16 microstepping driver
    DEFAULT_X_STEPS_PER_DEG 16\*200/360 // step/deg

  - limit switches
    cpu_map.h, section CPU_MAP_ATMEGA328P_ZUMSCAN: adjust limit switches pins. More detailed info in cpu_map.h
    config.h: homing probe order and params HOMING_INIT_LOCK, HOMING_CYCLE_\*, N_HOMING_LOCATE_CYCLE


Build and upload.

### Make

```bash
sudo apt-get install gcc-avr avr-libc
make
```

The binary *horus-fw.hex* can be flashed with [Horus GUI](https://github.com/bqlabs/horus).
