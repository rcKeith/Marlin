# Marlin 3D Printer Firmware

Additional documentation can be found in the 
repository [DerAndere1/Marlin at https://github.com](https://github.com/DerAndere1/Marlin/tree/2.0.x_E_homing) 
For CNC machines with E-axis repurposed to drive pumps or other tools,
e.g. lab robots (liquid handling robots, "pipetting robots"). Note that the focus 
of our development efforts shifted towards [adding true multi-axis support to Marlin 
(up to six axes plus extruders)](https://github.com/DerAndere1/Marlin/commits/bf2_6axis_dev13). 
Only use Marlin 2.0.x_E_homing if you are unable to contribute to [bf2_6axis_dev13](https://github.com/DerAndere1/Marlin/commits/bf2_6axis_dev13) or [Marlin2ForPipetBot](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot).  
Please test this firmware and let us know if it misbehaves in any way. 
Volunteers are standing by!

## Marlin 2.0.x_E_homing Branch

__Not for production use. Use with caution!__

Marlin 2.0.x_E_homing is a branch of the Marlin fork by DerAndere (based on 
https://github.com/MarlinFirmware/Marlin/tree/bugfix-2.0.x). 

This branch is for patches to the latest Marlin 2.0.x_E_homing release version.

Marlin 2.0.x_E_homing adds the E_AXIS_HOMING feature (status: Experimental).
See https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot-v1 .
Should NEVER be enabled if the E-axis is used for an actual extruder.
Only use if the E-axis stepper motor is repurposed (4 axis robot).
Uncommenting the option below enables homing of four axes (XYZ and E).
Currently only works if all of the following conditions are true:

1) Cartesian 4 axis robot (CORE, DELTA and SCARA kinematics are untested).
2) Not more than one endstop (limit switch) per axis (untested).
3) E_STOP_PIN or (E_MIN_PIN and/or E_MIN_PIN) defined and positive
    (usually in pins_MOTHERBOARD.h file, where
    MOTHERBOARD is the identifier for the controller board in use) and
    connected to the limit switch for the E-axis
4) No DUAL_X_CARRIAGE (untested)
5) No multiple stepper drivers per axis (untested)
6) No LINEAR_ADVANCE
7) No SENSORLESS_HOMING (untested)
8) Only EXTRUDERS 1. EXTRUDERS > 1 may work if SINGLENOZZLE is enabled (untested)

When E_AXIS_HOMING is enabled (#define E_AXIS_HOMING), enabling
(USE_EMIN_PLUG or USE_EMAX_PLUG), MIN_SOFTWARE_ENDSTOP_E and MAX_SOFTWARE_ENDSTOP_E is recommended.

## Building Marlin 2.0.x_E_homing

To build Marlin 2.0.x_E_homing you'll need [PlatformIO](http://docs.platformio.org/en/latest/ide.html#platformio-ide).you'll need [PlatformIO](http://docs.platformio.org/en/latest/ide.html#platformio-ide). The MarlinFirmware team has posted detailed instructions on [Building Marlin with PlatformIO](https://marlinfw.org/docs/basics/install_platformio.html).

## Hardware Abstraction Layer (HAL)

Marlin 2.0 introduces a layer of abstraction so that all the existing high-level code can be built for 32-bit platforms while still retaining full 8-bit AVR compatibility. Retaining AVR compatibility and a single code-base is important to us, because we want to make sure that features and patches get as much testing and attention as possible, and that all platforms always benefit from the latest improvements.

### Current HALs

  #### AVR (8-bit)

  board|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Arduino AVR](https://www.arduino.cc/)|ATmega, ATTiny, etc.|16-20MHz|64-256k|2-16k|5V|no

  #### DUE

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Arduino Due](https://www.arduino.cc/en/Guide/ArduinoDue), [RAMPS-FD](http://www.reprap.org/wiki/RAMPS-FD), etc.|[SAM3X8E ARM-Cortex M3](http://www.microchip.com/wwwproducts/en/ATsam3x8e)|84MHz|512k|64+32k|3.3V|no

  #### ESP32

  board|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [ESP32](https://www.espressif.com/en/products/hardware/esp32/overview)|Tensilica Xtensa LX6|240MHz|---|---|3.3V|---

  #### LPC1768 / LPC1769

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Re-ARM](https://www.kickstarter.com/projects/1245051645/re-arm-for-ramps-simple-32-bit-upgrade)|[LPC1768 ARM-Cortex M3](http://www.nxp.com/products/microcontrollers-and-processors/arm-based-processors-and-mcus/lpc-cortex-m-mcus/lpc1700-cortex-m3/512kb-flash-64kb-sram-ethernet-usb-lqfp100-package:LPC1768FBD100)|100MHz|512k|32+16+16k|3.3-5V|no
  [MKS SBASE](http://forums.reprap.org/read.php?13,499322)|LPC1768 ARM-Cortex M3|100MHz|512k|32+16+16k|3.3-5V|no
  [Selena Compact](https://github.com/Ales2-k/Selena)|LPC1768 ARM-Cortex M3|100MHz|512k|32+16+16k|3.3-5V|no
  [Azteeg X5 GT](https://www.panucatt.com/azteeg_X5_GT_reprap_3d_printer_controller_p/ax5gt.htm)|LPC1769 ARM-Cortex M3|120MHz|512k|32+16+16k|3.3-5V|no
  [Smoothieboard](http://reprap.org/wiki/Smoothieboard)|LPC1769 ARM-Cortex M3|120MHz|512k|64k|3.3-5V|no

  #### SAMD51

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Adafruit Grand Central M4](https://www.adafruit.com/product/4064)|[SAMD51P20A ARM-Cortex M4](https://www.microchip.com/wwwproducts/en/ATSAMD51P20A)|120MHz|1M|256k|3.3V|yes

  #### STM32F1

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Arduino STM32](https://github.com/rogerclarkmelbourne/Arduino_STM32)|[STM32F1](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html) ARM-Cortex M3|72MHz|256-512k|48-64k|3.3V|no
  [STEVAL-3DP001V1](http://www.st.com/en/evaluation-tools/steval-3dp001v1.html)|[STM32F401VE Arm-Cortex M4](http://www.st.com/en/microcontrollers/stm32f401ve.html)|84MHz|512k|64+32k|3.3-5V|yes

  #### Teensy++ 2.0

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Teensy++ 2.0](http://www.microchip.com/wwwproducts/en/AT90USB1286)|[AT90USB1286](http://www.microchip.com/wwwproducts/en/AT90USB1286)|16MHz|128k|8k|5V|no

  #### Teensy 3.1 / 3.2

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Teensy 3.2](https://www.pjrc.com/store/teensy32.html)|[MK20DX256VLH7](https://www.mouser.com/ProductDetail/NXP-Freescale/MK20DX256VLH7) ARM-Cortex M4|72MHz|256k|32k|3.3V-5V|yes

  #### Teensy 3.5 / 3.6

  boards|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Teensy 3.5](https://www.pjrc.com/store/teensy35.html)|[MK64FX512VMD12](https://www.mouser.com/ProductDetail/NXP-Freescale/MK64FX512VMD12) ARM-Cortex M4|120MHz|512k|192k|3.3-5V|yes
  [Teensy 3.6](https://www.pjrc.com/store/teensy36.html)|[MK66FX1M0VMD18](https://www.mouser.com/ProductDetail/NXP-Freescale/MK66FX1M0VMD18) ARM-Cortex M4|180MHz|1M|256k|3.3V|yes

## Submitting Patches

Proposed patches should be submitted as a Pull Request against the ([bugfix-2.0.x](https://github.com/MarlinFirmware/Marlin/tree/2.0.x_E_homing)) branch.

- This branch is for fixing bugs and integrating any new features for the duration of the Marlin 2.0.x life-cycle.
- Follow the [Coding Standards](http://marlinfw.org/docs/development/coding_standards.html) to gain points with the maintainers.
- Please submit your questions and concerns to the [Issue Queue](https://github.com/DerAndere1/Marlin/issues).

### [RepRap.org Wiki Page](http://reprap.org/wiki/Marlin)

## Credits

The current Marlin dev team consists of:

 - Scott Lahteine [[@thinkyhead](https://github.com/thinkyhead)] - USA &nbsp; [Donate](http://www.thinkyhead.com/donate-to-marlin) / Flattr: [![Flattr Scott](http://api.flattr.com/button/flattr-badge-small.png)](https://flattr.com/submit/auto?user_id=thinkyhead&url=https://github.com/MarlinFirmware/Marlin&title=Marlin&language=&tags=github&category=software)
 - Roxanne Neufeld [[@Roxy-3D](https://github.com/Roxy-3D)] - USA
 - Chris Pepper [[@p3p](https://github.com/p3p)] - UK
 - Bob Kuhn [[@Bob-the-Kuhn](https://github.com/Bob-the-Kuhn)] - USA
 - João Brazio [[@jbrazio](https://github.com/jbrazio)] - Portugal
 - Erik van der Zalm [[@ErikZalm](https://github.com/ErikZalm)] - Netherlands &nbsp; [![Flattr Erik](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=ErikZalm&url=https://github.com/MarlinFirmware/Marlin&title=Marlin&language=&tags=github&category=software)

Marlin 2.0.x_E_homing is modified by:

 - DerAndere [[@DerAndere1](https://github.com/DerAndere1)] - Germany

## License

Marlin 2.0.x_E_homing is published under the [GPL license](/LICENSE) because we believe in open development. The GPL comes with both rights and obligations. Whether you use Marlin firmware as the driver for your open or closed-source product, you must keep Marlin open, and you must provide your compatible Marlin source code to end users upon request. The most straightforward way to comply with the Marlin license is to make a fork of Marlin on Github, perform your modifications, and direct users to your modified fork.

While we can't prevent the use of this code in products (3D printers, CNC, etc.) that are closed source or crippled by a patent, we would prefer that you choose another firmware or, better yet, make your own.
