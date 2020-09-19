# Marlin2ForPipetBot 3D Printer and Lab Robot CNC Firmware
 
Additional documentation can be found in the 
repository [DerAndere1/Marlin at https://github.com](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot), the [Wiki](https://github.com/DerAndere1/Marlin/wiki)
or on the [PipetBot-A8 project homepage](https://derandere.gitlab.io/pipetbot-a8) 
that is part of the [authors homepage](https://derandere.gitlab.io). 
For CNC machines with additional axes I, (J, (K)) that drive pumps or other tools,
e.g. lab robots (liquid handling robots, "pipetting robots"). 
Please test this firmware and let us know if it misbehaves in any way. 
Volunteers are standing by!

## Marlin2ForPipetBot Branch

__Not for production use. Use with caution!__

Marlin2forPipetBot is a branch of the Marlin fork by DerAndere (based on 
https://github.com/DerAndere1/Marlin/commit/638f6f0f0607399bce82123663f5463380f83ce4 ). 

This branch is for patches to the latest Marlin2ForPipetBot release version.

Marlin2ForPipetBot supports up to six non-extruder axes (NON_E_AXES) plus 
extruders (e.g. XYZIJK+E or XYZUVW+E). 
Currently, only a subset of the Marlin G-code dialect is supported:

- G1, G28, G81 (untested), G82 (untested), G83 (untested), G90, G91, G92 (partially) 
- M17, M18, M43, M85, M92, M111, M114 (partially), M201, M202, M203, M206 (partially), M500, M502, M503, M504

Default axis names are:

| NON_E_AXES | Axis codes        |
|------------|-------------------|
|           3|X, Y, Z, E         |
|           4|X, Y, Z, I, E      |
|           5|X, Y, Z, I, J, E   |
|           6|X, Y, Z, I, J, K, E|

Example syntax for movement (G-code G1) with NON_E_AXES 6: 
```
G1 [Xx.xxxx] [Yy.yyyy] [Zz.zzzz] [Ii.iiii] [Jj.jjjj] [Kk.kkkk] [Ee.eeee] [Ff.ffff]
```

## Configuration

Configuration is done by editing the file Marlin/Configuration.h. E.g. change

`define NON_E_AXES 3`

to: 

`define NON_E_AXES 4`

Important options are:

### `ASYNC_SECONDARY_AXES`

Define `ASYNC_SECONDARY_AXES` for a CNC machine with NON_E_AXES > 3 where primary axes XYZ are
coordinated. Optional additional axes I(, J(, K)) are uncoordinated
(asynchronous). Leave disabled (commented out) for coordinated movement of all axes.

### `FOAMCUTTER_XYUV`

Define `FOAMCUTTER_XYUV` kinematics for a hot wire cutter with parallel horizontal axes X, U where the hights
of the two wire ends are controlled by parallel axes Y, V.

### `NON_E_AXES`

`NON_E_AXES`: 
The number of axes that are not used for extruders (axes that
benefit from endstops and homing). `NON_E_AXES` > `3` requires definition of
`[[I, [J, [K]]]_STEP_PIN`, `[I, [J, [K]]]_ENABLE_PIN`, `[I, [J, [K]]]_DIR_PIN`,
`[I, [J, [K]]]_STOP_PIN`, `USE_[I, [J, [K]]][MIN || MAX]_PLUG` and definition of the
respective values of `DEFAULT_AXIS_STEPS_PER_UNIT`, `DEFAULT_MAX_FEEDRATE`,
`DEFAULT_MAX_ACCELERATION`, `AXIS_RELATIVE_MODES`, `MICROSTEP_MODES` and
`MANUAL_FEEDRATE`.

Allowed values: [3, 4, 5, 6]


### `AXIS4_NAME`

`AXIS4_NAME`, `AXIS5_NAME`, `AXIS6_NAME`:
Axis codes for additional axes:
This defines the axis code that is used in G-code commands to 
reference a specific axis. 

   * 'I' for generic 4th axis
   * 'J' for generic 5th axis
   * 'K' for generic 6th axis
   * 'A' for rotational axis parallel to X
   * 'B' for rotational axis parallel to Y
   * 'C' for rotational axis parallel to Z
   * 'U' for secondary linear axis parallel to X
   * 'V' for secondary linear axis parallel to Y
   * 'W' for secondary linear axis parallel to Z

Regardless of the settings, firmware-internal axis names are
I (AXIS4), J (AXIS5), K (AXIS6).

Allowed values: ['I', 'J', 'K', 'A', 'B', 'C', 'U', 'V', 'W'] 

## Building Marlin2ForPipetBot

To build Marlin2ForPipetBot you'll need [PlatformIO](http://docs.platformio.org/en/latest/ide.html#platformio-ide). The MarlinFirmware team has posted detailed instructions on [Building Marlin with PlatformIO](https://marlinfw.org/docs/basics/install_platformio.html).

The different branches in the git repository https://github.com/DerAndere1/Marlin reflect different stages of development: 

- [6axis_dev](https://github.com/DerAndere1/Marlin/tree/6axis_dev) branch is the current feature branch used for testing of Multi-axis Marlin. It features the best 6axis support available and is intended to be frequently rebased on the latest [MarlinFirmware/Marlin bugfix-2.0.x. Currently it is based on bugfix-2.0.x from 2020-08-22, [https://github.com/MarlinFirmware/Marlin/commit/434e43cc42f782d5fbe89db21f97571c71ad62f3](https://github.com/MarlinFirmware/Marlin/commit/434e43cc42f782d5fbe89db21f97571c71ad62f3) or later. If you want to participate in the development, (re)base your contributions on this branch. Pull requests targeted at the 6axis_dev branch are very welcome.  

- [6axis_PR1](https://github.com/DerAndere1/Marlin/tree/6axis_PR1) branch is like 6axis_dev but without config files. A long-time goal is to bring multi-axis support into upstream MarlinFirmware/Marlin (pull request https://github.com/MarlinFirmware/Marlin/pull/19112)

- [bf2_6axis_dev14](https://github.com/DerAndere1/Marlin/tree/bf2_6axis_dev14) branch and similar: Untested internal development branches have consecutivly numbered branch names. Do not expect that anything works.   

- [Marlin2ForPipetBot](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot) branch is more stable but outdated. This branch is the release branch for [tagged releases of Marlin2ForPipetBot firmware](https://github.com/DerAndere1/Marlin/tags). It is based on Marlin bugfix-2.0.x from August 2020 (commit https://github.com/DerAndere1/Marlin/commit/638f6f0f0607399bce82123663f5463380f83ce4) 

- [Marlin2ForPipetBot_dev](https://github.com/DerAndere1/Marlin/tree/Marlin2ForPipetBot_dev) is used to develop and test bugfixes for Marlin2ForPipetBot. After successful testing, it will be merged into Marlin2ForPipetBot. 

- Other branches: Deprecated legacy code.

## Hardware Abstraction Layer (HAL)

Marlin 2.0 introduces a layer of abstraction so that all the existing high-level code can be built for 32-bit platforms while still retaining full 8-bit AVR compatibility. Retaining AVR compatibility and a single code-base is important to us, because we want to make sure that features and patches get as much testing and attention as possible, and that all platforms always benefit from the latest improvements.

### Current HALs

  name|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [Arduino AVR](https://www.arduino.cc/)|ATmega, ATTiny, etc.|16-20MHz|64-256k|2-16k|5V|no
  [Teensy++ 2.0](http://www.microchip.com/wwwproducts/en/AT90USB1286)|[AT90USB1286](http://www.microchip.com/wwwproducts/en/AT90USB1286)|16MHz|128k|8k|5V|no
  [Arduino STM32](https://github.com/rogerclarkmelbourne/Arduino_STM32)|[STM32F1](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html) ARM-Cortex M3|72MHz|256-512k|48-64k|3.3V|no
  [Due](https://www.arduino.cc/en/Guide/ArduinoDue), [RAMPS-FD](http://www.reprap.org/wiki/RAMPS-FD), etc.|[SAM3X8E ARM-Cortex M3](http://www.microchip.com/wwwproducts/en/ATsam3x8e)|84MHz|512k|64+32k|3.3V|no
  [Re-ARM](https://www.kickstarter.com/projects/1245051645/re-arm-for-ramps-simple-32-bit-upgrade)|[LPC1768 ARM-Cortex M3](http://www.nxp.com/products/microcontrollers-and-processors/arm-based-processors-and-mcus/lpc-cortex-m-mcus/lpc1700-cortex-m3/512kb-flash-64kb-sram-ethernet-usb-lqfp100-package:LPC1768FBD100)|100MHz|512k|32+16+16k|3.3-5V|no
  [MKS SBASE](http://forums.reprap.org/read.php?13,499322)|LPC1768 ARM-Cortex M3|100MHz|512k|32+16+16k|3.3-5V|no
  [Azteeg X5 GT](https://www.panucatt.com/azteeg_X5_GT_reprap_3d_printer_controller_p/ax5gt.htm)|LPC1769 ARM-Cortex M3|120MHz|512k|32+16+16k|3.3-5V|no
  [Selena Compact](https://github.com/Ales2-k/Selena)|LPC1768 ARM-Cortex M3|100MHz|512k|32+16+16k|3.3-5V|no
  [Teensy 3.5](https://www.pjrc.com/store/teensy35.html)|ARM-Cortex M4|120MHz|512k|192k|3.3-5V|yes
  [Teensy 3.6](https://www.pjrc.com/store/teensy36.html)|ARM-Cortex M4|180MHz|1M|256k|3.3V|yes

### HALs in Development

  name|processor|speed|flash|sram|logic|fpu
  ----|---------|-----|-----|----|-----|---
  [STEVAL-3DP001V1](http://www.st.com/en/evaluation-tools/steval-3dp001v1.html)|[STM32F401VE Arm-Cortex M4](http://www.st.com/en/microcontrollers/stm32f401ve.html)|84MHz|512k|64+32k|3.3-5V|yes
  [Smoothieboard](http://reprap.org/wiki/Smoothieboard)|LPC1769 ARM-Cortex M3|120MHz|512k|64k|3.3-5V|no
  [Adafruit Grand Central M4](https://www.adafruit.com/product/4064)|ARM-Cortex M4|120MHz|1M|256k|3.3V|yes

## Submitting Patches

Proposed patches should be submitted as a Pull Request against the [Marlin2ForPipetBot](https://github.com/MarlinFirmware/Marlin/tree/Marlin2ForPipetBot) branch.

- This branch is for fixing bugs and integrating any new features for the duration of the Marlin 2.0.x life-cycle.
- Follow the [Coding Standards](http://marlinfw.org/docs/development/coding_standards.html) to gain points with the maintainers.
- Please submit your questions and concerns to the [Issue Queue](https://github.com/DerAndere1/Marlin/issues).

### [RepRap.org Wiki Page](http://reprap.org/wiki/Marlin)

## Credits

The current Marlin dev team consists of:

 - Scott Lahteine [[@thinkyhead](https://github.com/thinkyhead)] - USA &nbsp; [![Flattr Scott](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=thinkyhead&url=https://github.com/MarlinFirmware/Marlin&title=Marlin&language=&tags=github&category=software)
 - Roxanne Neufeld [[@Roxy-3D](https://github.com/Roxy-3D)] - USA
 - Bob Kuhn [[@Bob-the-Kuhn](https://github.com/Bob-the-Kuhn)] - USA
 - Chris Pepper [[@p3p](https://github.com/p3p)] - UK
 - João Brazio [[@jbrazio](https://github.com/jbrazio)] - Portugal
 - Erik van der Zalm [[@ErikZalm](https://github.com/ErikZalm)] - Netherlands &nbsp; [![Flattr Erik](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=ErikZalm&url=https://github.com/MarlinFirmware/Marlin&title=Marlin&language=&tags=github&category=software)

Marlin2ForPipetBot is modified by:

 - DerAndere [[@DerAndere1](https://github.com/DerAndere1)] - Germany
 - Garbriel Beraldo [@GabrielBeraldo](https://github.com/GabrielBeraldo)] - Brasil
 - Olivier Briand [@hobiseven](https://github.com/hobiseven)] - France
 - Wolverine [@MohammadSDGHN](https://github.com/MohammadSDGHN) - Undisclosed 
 - bilsef [@bilsef](https://github.com/bilsef) - Undisclosed 
 - FNeo31 [@FNeo31](https://github.com/FNeo31) - Undisclosed 

## License

Marlin2ForPipetBot is published under the [GPL license](https://github.com/DerAndere1/Marlin/blob/Marlin2ForPipetBot/LICENSE) because we believe in open development. The GPL comes with both rights and obligations. Whether you use Marlin firmware as the driver for your open or closed-source product, you must keep Marlin open, and you must provide your compatible Marlin source code to end users upon request. The most straightforward way to comply with the Marlin license is to make a fork of Marlin on Github, perform your modifications, and direct users to your modified fork.

While we can't prevent the use of this code in products (3D printers, CNC, etc.) that are closed source or crippled by a patent, we would prefer that you choose another firmware or, better yet, make your own.
