# Opensource toolchain for RPI RP2040

The RP2040 is a 32-bit dual ARM Cortex-M0+ microcontroller integrated circuit by Raspberry Pi Foundation. At the same time, it was released as part of the Raspberry Pi Pico board.

It announced on 21st January 2021, the RP2040 is the first microcontroller designed by the Raspberry Pi Foundation. The microcontroller is low cost, with the Raspberry Pi Pico being introduced at US$4 and the RP2040 itself costing US$1. The microcontroller can be programmed in Assembly, Rust, C/C++ and MicroPython. It is powerful enough to run TensorFlow Lite.

Per the [datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf), there are multiple versions of the chip:
"The full source for the RP2040 bootROM can be found at https://github.com/raspberrypi/pico-bootrom.
This includes both version 1 and version 2 of the bootROM, which correspond to the B0 and B1 silicon revisions, respectively." 

For more info about RP2040, please refer to https://www.raspberrypi.com/products/rp2040/

A number of manufacturers have announced their own boards using the RP2040, among them, 'Raspberry Pi Pico' is a tiny, fast, and versatile and official reference board produced by Raspberry Pi. 

**There is a [very good manual](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) for Raspberry Pi Pico, If you already read this official manual, you can just ignore this tutorial**. 

# Hardware requirements

* A RP2040 board. Here I use PICO, and you have many other choices such as seeedstudio XIAO and various other PICO compatible boards.
* A USB CMSIS DAP adapter with SWD interface.

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
## pico-sdk

The official SDK for RP2040 is 'pico-sdk'. Pico SDK provides the headers, libraries and build system necessary to write programs for the RP2040-based devices such as the Raspberry Pi Pico in C, C++ or assembly language.

The SDK is designed to provide an API and programming environment that is familiar both to non-embedded C developers and embedded C developers alike. A single program runs on the device at a time and starts with a conventional main() method. Standard C/C++ libraries are supported along with C level libraries/APIs for accessing all of the RP2040's hardware include PIO (Programmable IO).

Additionally the SDK provides higher level libraries for dealing with timers, synchronization, USB (TinyUSB) and multi-core programming along with various utilities.

Please refer to https://github.com/raspberrypi/pico-sdk for more info. it's very easy to setup pico-sdk:

```
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
```

You can put pico-sdk system-wide or use it within your project. here we put it system-wide

```
sudo mv pico-sdk /opt
```

And set `PICO_SDK_PATH` to the SDK location (here is `/opt/pico-sdk`) in your environment, or pass it (-DPICO_SDK_PATH=/opt/pico-sdk) to cmake later.

After SDK setup, you can fetch the `pico-examples` and try to build it.

```
git clone https://github.com/raspberrypi/pico-examples.git
cd pico-examples
mkdir build
cd build
cmake .. -DPICO_SDK_PATH=/opt/pico-sdk
cd blink
make
```

After built successfully, a lot of file include 'blink.elf', 'blink.hex' and 'blink.uf2' will be generated.

## pico-extras

There are also some additional libraries provided in 'pico-extras' that are not yet ready for inclusion the Pico SDK proper, or are just useful but don't necessarily belong in the Pico SDK. the corresponding examples is 'pico-playground'.

NOTE, it depend on pico-sdk, you need setup pico-sdk correctly first.

```
git clone https://github.com/raspberrypi/pico-extras.git
sudo mv pico-extras /opt
```

And set `PICO_EXTRAS_PATH` to `/opt/pico-extras` in your environment.

for 'pico-playground':

```
git clone https://github.com/raspberrypi/pico-playground.git
cd pico-playground
mkdir build
cd build
cmake ..
make
```

# Flashing
There are a lot of way to flash the firmware of RP2040,  as mentioned above, here will introduce the 'usb-storage' way and 'OpenOCD' way.

## USB storage

**'Holding the bootsel button down and plug in'** will enter USB storage mode. you can mount it and DND the 'uf2' firmware into RP2040 and it will flash the firmware automatically, generally, you will wait a little moment until it done.

If the board has 'reset' button, you can also 'Holding the bootset button down, then press and release reset button, then release bootsel button' to enter USB storage mode.

Here the device is sda1 and using blink example in 'pico-examples', the flashing process looks like:

```
sudo mount /dev/sda1 /mnt/rp2040
sudo cp blink.uf2 /mnt/rp2040
sudo umount /mnt/rp2040
```

After flashing complete, the LED on board will blink.

## OpenOCD/SWD

The upstream OpenOCD is lack of supporting for RP2040, so you can not use official OpenOCD with RP2040, and have to build the fork version from RPI yourself.

Building and Installation:
```
git clone https://github.com/raspberrypi/openocd.git --recursive --branch rp2040 --depth=1
cd openocd
./configure --prefix=/opt/pico-openocd --program-prefix=pico- --enable-cmsis-dap --disable-werror
make
sudo make install
```
NOTE here use 'pico-' program prefix to avoid conflict with system wide openocd. and install it to `/opt/pico-openocd/`, you need add `/opt/pico-openocd/bin` to PATH env.

After pico-openocd installed, please wire up your SWD adapter with the corresponding PINs of PICO, there are 3 pins (GND/SWDIO/SWCLK) marked as 'debug' in PICO board. you can use USB cable to supply power and connect that 3pins, or use SWD adapter only to supply power (3.3v, connect to VSYS/VIN pin). 

<img src="https://user-images.githubusercontent.com/1625340/157585375-e06f80d3-d4ec-46a2-9f09-caf213f32a20.png"/>

There is also a lot of DOCKs for pico with CMSIS DAP integrated, it's more convenient, you can also use such a DOCK.

Here I use standalone [tigard](https://github.com/tigard-tools/tigard) FT2232 board as SWD adapter.

to flashing the 'blink.hex' from blink example:

```
pico-openocd -f tigard-swd.cfg -f /opt/pico-openocd/share/openocd/scripts/target/rp2040.cfg -c "init; reset halt; targets rp2040.core0; flash write_image erase blink.hex; reset; shutdown;"
```

A wrapper script 'pico-swd' is provided within this repo, you can modify (change it to use your own interface cfg file) and use it instead of typing command everytime. the script support 'reset', 'write firmware', 'dump firmware' and 'attach'(useful for debugging) to rp2040 board. for example, above flashing command can be shortten to:

```
pico-swd write blink.hex
```

# Debugging

First, you need attach to the MCU, launch a terminal and run:

```
sudo pico-openocd -f tigard-swd.cfg -f /opt/pico-openocd/share/openocd/scripts/target/rp2040.cfg
```

or simply run:
```
pico-swd attach
```

the output looks like:
```
Open On-Chip Debugger 0.11.0-g610f137 (2022-02-23-00:47)
Licensed under GNU GPL v2
For bug reports, read
        http://openocd.org/doc/doxygen/bugs.html
Info : FTDI SWD mode enabled
Info : Hardware thread awareness created
Info : Hardware thread awareness created
Info : RP2040 Flash Bank Command
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : clock speed 2000 kHz
Info : SWD DPIDR 0x0bc12477
Info : SWD DLPIDR 0x00000001
Info : SWD DPIDR 0x0bc12477
Info : SWD DLPIDR 0x10000001
Info : rp2040.core0: hardware has 4 breakpoints, 2 watchpoints
Info : rp2040.core1: hardware has 4 breakpoints, 2 watchpoints
Info : starting gdb server for rp2040.core0 on 3333
Info : Listening on port 3333 for gdb connections
```

Then launch another terminal, build the blink example with debug infomation:

```
cd pico-examples
mkdir build-debug
cd build-debug
cmake -DCMAKE_BUILD_TYPE=debug ..
cd blink
make
arm-none-eabi-gdb ./blink.elf
```

And a gdb session will opened:

```
Reading symbols from ./blink.elf...
(gdb) target remote :3333
Remote debugging using :3333
warning: multi-threaded target stopped without sending a thread-id, using first non-exited thread
main () at blink/blink.c:18
18              sleep_ms(500);
(gdb) list main
4        * SPDX-License-Identifier: BSD-3-Clause
5        */
6
7       #include "pico/stdlib.h"
8
9       int main() {
10      #ifndef PICO_DEFAULT_LED_PIN
11      #warning blink example requires a board with a regular LED
12      #else
13          const uint LED_PIN = PICO_DEFAULT_LED_PIN;
(gdb) list
14          gpio_init(LED_PIN);
15          gpio_set_dir(LED_PIN, GPIO_OUT);
16          while (true) {
17              gpio_put(LED_PIN, 1);
18              sleep_ms(500);
19              gpio_put(LED_PIN, 0);
20              sleep_ms(500);
21          }
22      #endif
23      }
(gdb) break 18
Breakpoint 1 at 0x10000328: file pico-examples/blink/blink.c, line 18.
Note: automatically using hardware breakpoints for read-only addresses.
(gdb) c
Continuing.
target halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0x00000178 msp: 0x20041f00

Thread 1 hit Breakpoint 1, main () at pico-examples/blink/blink.c:18
18              sleep_ms(500);
(gdb)
```





















