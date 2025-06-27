# LARS Hardware Interface

This repository contains the schematic, PCB design files and a basic firmware stack for the Hardware Interface of the [Lightweight Autonomous Racing System (LARS)](https://github.com/CAuDri/LARS). 

<p align="center">
  <img width="600" src="https://github.com/CAuDri/LARS-Hardware-Interface/blob/main/PCB/images/hardware_interface_3D_render.png?raw=true">
</p>

The board is based on a STM32F4 microcontroller and is designed to interface with various sensors and actuators on the vehicle. The communication with the host PC is done via USB, using [micro-ROS](https://micro.ros.org/) and the underlying [micro-XRCE DDS](https://micro-xrce-dds.docs.eprosima.com/en/latest/).

*A basic firmware stack will be released soon.*

## License
This project is licensed under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html). You can freely use, modify, and distribute this code as long as you comply with the terms of the license.

The hardware design files are released under the [CERN Open Hardware Licence v2](https://ohwr.org/cernohl/).

## Features
- STM32F429 microcontroller (Cortex-M4, 180MHz, 2MB Flash, 256KB RAM)
- Board dimensions: 86mm x 86mm
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
- 3.3V and 5V output for external devices

## PCB-Files
The board was designed using KiCad 9.0. All PCB design files, a bill of materials and Gerber Files for manufacturing at [JLCPCB](https://jlcpcb.com/) can be found in the `/PCB` directory.

### Here is a link to the [Schematic](https://github.com/CAuDri/LARS-Hardware-Interface/blob/main/PCB/schematic.pdf)
