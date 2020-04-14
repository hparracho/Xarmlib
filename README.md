# Xarmlib - Works like a charm!
Experimental C++ library for bare-metal ARM Cortex-M platforms

## Warning:
***This library is mainly a personal project. For now, it is nothing more than a testbed for experiments and tests. It is in a very immature and early state of development and contains most certainly lots of bugs and issues that need to be fixed. You can use it freely in any way you like but it comes with absolutely no warranty at all! - You have been warned! :-)***

---
## Supported Targets:
- NXP LPC81x
- NXP LPC84x
- NXP KV4x
- NXP KV5x *(work in progress...)*

## Supported Toolchains:
- [GNU Arm Embedded Toolchain 7-2017q4-major or greater](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)
- [	MCUXpresso IDE v10.1.0 or greater](https://www.nxp.com/support/developer-resources/software-development-tools/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE)

## External Dependencies:
- [bitmask class v1.1.2](https://github.com/oliora/bitmask) - Copyright (c) 2016-2017 [Andrey Upadyshev](https://github.com/oliora) 
- [CMSIS-Core(M)](https://github.com/ARM-software/CMSIS_5) - currently using master commit as of 9 April 2020
- [FatFs: Generic FAT Filesystem Module Source Files R0.14](http://elm-chan.org/fsw/ff/00index_e.html)
- [fsl: Freescale drivers from Kinetis SDK Version 2.7.0](https://www.nxp.com/downloads/en/software-development-kits/KINETIS-EXPERT-TOOL.html)
- [lfs: Little fail-safe filesystem v2.2.1](https://github.com/ARMmbed/littlefs) - currently using master commit as of 9 April 2020
- [span: std::span implementation for C++11 and later](https://github.com/tcbrindle/span) - currently using master commit as of 31 January 2020
- [usflib: Micro String Format (μSF) library](https://github.com/hparracho/usflib) - currently using master commit as of 14 January 2019

---
## Credit where it is due:

***Alone we don't go far...*** This library uses some code and ideas taken from other persons and projects:  

- The startup code is *heavily* based on the [Cortex-M Startup](https://github.com/micro-os-plus/cortexm-startup) project, part of the [µOS++ IIIe](https://github.com/micro-os-plus) project. <-- *Great project, check it out!* Copyright (c) 2016 [Liviu Ionescu](https://github.com/ilg-ul).
- The Delegate class is based on the [Delegate](https://github.com/nikitablack/cpp-tests/blob/master/Delegate/Delegate.h) implementation from [Nikita Chernyi](https://github.com/nikitablack). A detailed description of the original implementation can be found [here](https://nikitablack.github.io/2016/04/12/Generic-C-delegates.html).
- The low level peripheral drivers for the LPC81x family of MCUs are based on the [NXP LPCOpen LPC8xx](https://www.nxp.com/downloads/en/libraries/lpcopen_3_02_lpcxpresso_nxp_lpcxpresso_812.zip) supplied by NXP. All rights reserved. (c) 2017 NXP B.V.
- The low level peripheral drivers for the LPC84x family of MCUs are based on the [LPC845 Example Code Bundle MCUXpresso](https://www.nxp.com/downloads/en/software/LPC845-Example-Code-Bundle-MCUXpresso.zip) supplied by NXP. All rights reserved. (c) 2017 NXP B.V.

