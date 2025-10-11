# LARS Hardware Interface

This repository contains the schematic, PCB design files and a basic firmware stack for the Hardware Interface of the [Lightweight Autonomous Racing System (LARS)](https://github.com/CAuDri/LARS). 

<p align="center">
  <img width="600" src="https://github.com/CAuDri/LARS-Hardware-Interface/blob/main/PCB/export/hardware_interface_3D_render.png?raw=true">
</p>

The board is based on a STM32F4 microcontroller and is designed to interface with various sensors and actuators on the vehicle. The communication with the host PC is done via USB, using [micro-ROS](https://micro.ros.org/) and the underlying [micro-XRCE DDS](https://micro-xrce-dds.docs.eprosima.com/en/latest/).


## License

This project is licensed under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html). You can freely use, modify, and distribute this code as long as you comply with the terms of the license.

The hardware design files are released under the [CERN Open Hardware Licence v2](https://ohwr.org/cernohl/).

# Hardware

##  Features

### General
- STM32F429 microcontroller (Cortex-M4, 180MHz, 2MB Flash, 256KB RAM)
- Board dimensions: 86 x 86 mm
- SWD interface for programming and debugging
- Boot-/Reset header for flashing via USB
- User- and Reset buttons
- RGB Debug LEDs

### Power
- 5V power supply via XT30 connector or USB (XT30 will be prioritized)
- Power input range: **4.8V - 5.3V**
- "Reverse polarity protection"
- On-board 3.3V voltage regulator
- Switchable 5V output for external devices
- Additional 3.3V and 5V output

### Interfaces
- 2x USB-C (2.0 Fullspeed) for host PC and debugging/flashing
- CAN-Bus (with optional termination)
- 8x I2C
- 3x UART
- 2x SPI
- 2x Servo (with analog feedback)
- 2x WS2812 LED strip 
- Buffered and filtered analog inputs
- Additional Timer channels and GPIOs
- 3.3V and 5V output for external devices

### Protection
- 4A/8A polyfuse on the main power input
- "Rudimentary" reverse polarity protection
- Back-power protection for both power inputs (USB and XT30)
- TVS diodes against voltage spikes on all power rails
- Current limited 5V output (max 2.4 A)
- All data lines are 5V tolerant
- Additional short-circuit, overcurrent and -voltage protection up to ~20V on all interfaces

## PCB-Files

The board was designed in KiCad 9.0. The project file can be found in the `/PCB/KiCad` directory.

A full [Schematic](https://github.com/CAuDri/LARS-Hardware-Interface/blob/main/PCB/schematic.pdf) of the board can be found as a PDF in the `/PCB` directory.

All PCB design files, a bill of materials and Gerber Files for manufacturing at [JLCPCB](https://jlcpcb.com/) can be found in the `/PCB/manufacturing` directory.

A 3D STEP model of the board can be found in the `/PCB/export` directory.