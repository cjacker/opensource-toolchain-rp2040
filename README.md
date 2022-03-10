# Opensource toolchain for RPI RP2040

The RP2040 is a 32-bit dual ARM Cortex-M0+ microcontroller integrated circuit by Raspberry Pi Foundation. At the same time, it was released as part of the Raspberry Pi Pico board.

It announced on 21st January 2021, the RP2040 is the first microcontroller designed by the Raspberry Pi Foundation. The microcontroller is low cost, with the Raspberry Pi Pico being introduced at US$4 and the RP2040 itself costing US$1. The microcontroller can be programmed in Assembly, Rust, C/C++ and MicroPython.[1] It is powerful enough to run TensorFlow Lite.

Per the [datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf), there are multiple versions of the chip:
"The full source for the RP2040 bootROM can be found at https://github.com/raspberrypi/pico-bootrom.
This includes both version 1 and version 2 of the bootROM, which correspond to the B0 and B1 silicon revisions, respectively." 

For more info about RP2040, please refer to https://www.raspberrypi.com/products/rp2040/

A number of manufacturers have announced their own boards using the RP2040, among them, 'Raspberry Pi Pico' is a tiny, fast, and versatile and official reference board produced by Raspberry Pi. 

**There is a [very good manual](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) for Raspberry Pi Pico, If you already read this official manual, you can just ignore this tutorial**. 

# Hardware requirements

* A RP2040 board. Here I use PICO, and you have many other choices such as seeedstudio XIAO and various other PICO compatible boards.
* A USB SWD adapter. or another PICO using picoprobe as a debugger and programmer.

**NOTE**
RP2040 can work as a USB storage if 'holding the bootsel button down and plug in', you can mount it and DND hex file to flash. but it does not support debugging and a little bit slow, so you'd better prepare a SWD adapter for debugging.

# Toolchain overview
* Compiler, arm gnu toolchain
* Debugger, OpenOCD/gdb
* SDK, pico-sdk/pico-extras
* Flashing tool, OpenOCD or via USB storage.


# ARM GNU Toolchain
As same as STM32 and various ARM based MCU, RP2040 use the 'arm-none-eabi' GNU toolchain. it's not neccesary to build the toolchain yourself, since there are already a lot of well supported prebuilt release and already widely used by developers. If you insist to build it yourself, you can refer to [linaro project](https://www.linaro.org/).

Here I use linaro prebuilt toolchain with X86_64 Linux, you can download it from https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads.

Download and extract the toolchain
```
wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
sudo tar xf gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 -C /opt
```

And add `/opt/gcc-arm-none-eabi-10.3-2021.10/bin` to PATH env according to the shell you used.

NOTE the toolchain's tripplet is 'arm-none-eabi'.

There are also a lot of prebuilt arm-none-eabi toolchains from other vendors, you can also use them as you like.

# SDK












