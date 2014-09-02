Hardware for the netzsinus project
===================================

This repository contains the measurement hardware and firmware of the
netzsinus project. The current hardware is still under development -
you have to solder your own hardware. The hardware is, however, pretty
simple and easy to build.

Our vision is to develop a network analyzer in the sense of [DIN EN
50160](https://de.wikipedia.org/wiki/EN_50160). Currently, only the
network frequency is measured, but other measurements such as voltage
and harmonics will be included step-by-step.

Overview
--------

The hardware consists of

* A 9VAC wall-wart power adaptor. I have only tested Ideal Power model
DE-06-09, but others should also work.
* A STM32F4 Discovery board. This evaluation kit offers a Cortex M4
microcontroller which will become handy if we implement the harmonics
analysis.
* An adapter board which contains the analog frondend and connects all
the components.
* A raspberry pi which runs a simple Go daemon to read the data from the
hardware and transmit it to https://netzsin.us.

Building your own device
------------------------

Currently, you just have to solder your own hardware. But since the
hardware is pretty simple this is an easy task. Please use this
schematic:

![Schematic](https://raw.githubusercontent.com/netzsinus/defluxio-hardware/master/hardware/pics/schaltplan-v0.2.jpg)

The opamp is an TLC393 comparator. I used a protoboard and THT components to build this circuit as a shield
for the STM32F4 discovery board. This is my setup:

![Overview](https://raw.githubusercontent.com/netzsinus/defluxio-hardware/master/hardware/pics/hw-v02-overview.jpg)

![Front](https://raw.githubusercontent.com/netzsinus/defluxio-hardware/master/hardware/pics/hw-v02-frontside.jpg)

![Back](https://raw.githubusercontent.com/netzsinus/defluxio-hardware/master/hardware/pics/hw-v02-backside.jpg)

The STM32F4 discovery costs around 15 Euro, the 9VAC adaptor around 5,
everything else maybe 2 Euros.
