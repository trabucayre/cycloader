.. _todo:

To Do
#####

Global
======

* improve error message (be more precise)
* catch all exception
* documentation (code + API)

Cable
=====

* fix *ch552* (*Sipeed tangNano*): works with *SRAM*, fails with *Flash*
* *busblaster* support
* *anlogic* cable support

Devices/boards
==============

* improve frequency configuration. Use FPGA, cable or --freq args maximum frequency
* rework *cyclone10* eeprom access to avoid using FT2232 interfaceB Spi emulation (only supported by trenz board)
* fix checksum computation with *gowin GW2A*
* add support for *tangPrimer* (*anlogic EG4S20*)

Misc
====

* fix spiFlash class to be able to write everywhere (currently offset is hardcoded to 0)
