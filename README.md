# LARS Hardware Interface

This repository contains the schematic, PCB design files and a basic firmware stack for the Hardware Interface of the [Lightweight Autonomous Racing System (LARS)](https://github.com/CAuDri/LARS). 

The Hardware Interface is based on a STM32F4 microcontroller and is designed to interface with various sensors and actuators used on the vehicle. 

## License
This project is licensed under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html). You can freely use, modify, and distribute this code as long as you comply with the terms of the license.

The hardware design files are released under the [CERN Open Hardware Licence v2](https://ohwr.org/cernohl/).

## Features
- STM32F429 microcontroller (Cortex-M4, 180MHz, 2MB Flash, 256KB RAM)
- Board dimensions: 85mm x 85mm
- SWD interface for programming and debugging
- 5V power supply via XT30 connector or USB
- 2x USB-C (2.0 Fullspeed) for host PC and debugging
- CAN-Bus
- 8x I2C ports
- 3x UART ports
- 2x SPI ports
- 2x Servo connectors (with analog feedback)
- 2x WS2812 LED strip connectors
- Buffered analog inputs
- Additional Timer channels and GPIOs
- User Button and Debug LEDs
- 3.3V/5V output for external devices

## PCB-Files
The PCB design files are available in the `PCB` directory. The design is done using KiCAD and includes the schematic, PCB layout, and bill of materials (BOM). You can open the files using KiCAD or any compatible EDA tool.