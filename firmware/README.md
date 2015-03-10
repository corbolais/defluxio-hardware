Netzsinus Firmware
==================

Prerequisites
-------------
The firmware has been successfully built on the following host systems:

* Ubuntu Linux 14.04
* Mac OS Mavericks

Please install the following prerequisites first:

* Install [gcc-arm-embedded](https://launchpad.net/gcc-arm-embedded) and
  make sure the arm-none-eabi-*-commands are in your path
* Install [OpenOCD](http://openocd.sourceforge.net/)

Then, clone the repository and change into the firmware subdirectory:

	$ cd defluxio-hardware/firmware

Building the firmware
---------------------

First, you need to grab and compile a copy of libopencm3:

	$ git submodule init
	$ git submodule update
	$ cd libopencm3
	$ make

This will compile the library. Afterwards, change into the src directory and
build the example:

	$ cd ../src
	$ make

Connect your STM32-Discovery board and run 

	$ make flash

to upload the binary into the microcontroller. In order to see the USART output,
connect a 3V3 TTL adaptor to PA2 (USART2_TX). You can use `screen` to see the output:

	$ screen /dev/ttyUSB0 115200

Please note: as of commit adc3d7d4, the firmware does only print an info
line when starting up. It will print frequency information only if a
plausible sine wave is detected, so please connect the AC adaptor. You
should see something similar to this:

````
I;Defluxio Frequency Measurement Hardware started.
I;Frequency: 50.022183 Hz, delta:  22 mHz
F;50.02218
I;Frequency: 50.018508 Hz, delta:  19 mHz
F;50.01851
I;Frequency: 50.016797 Hz, delta:  17 mHz
F;50.01680
I;Frequency: 50.015841 Hz, delta:  16 mHz
F;50.01584
I;Frequency: 50.016847 Hz, delta:  17 mHz
F;50.01685
````
