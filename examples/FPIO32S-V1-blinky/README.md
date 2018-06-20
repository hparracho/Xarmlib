# FPIO32S-V1-blinky example
Small example for FPIO32S-V1 board showing basic Timer, DigitalIn and DigitalOut classes usage.

## Dependencies

- [GNU Arm Embedded Toolchain "7-2017-q4-major"](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) or [NXP MCUXpresso IDE v10.2.0 - GNU ARM Embedded Toolchain "7-2017-q4-major"](https://www.nxp.com/support/developer-resources/software-development-tools/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE?tab=Design_Tools_Tab#nogo)
- [CMake](https://cmake.org/download/)
- [MinGW-w64](https://sourceforge.net/projects/mingw-w64)

## Building (Windows example)

```
mkdir build
cd build
cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCOMPILER_PATH=<FULL_PATH_TO_GNU_ARM_TOOLCHAIN> ..
mingw32-make
```

change `CMAKE_BUILD_TYPE=Debug` to build debug version.
`mingw32-make.exe` must be in your path.
