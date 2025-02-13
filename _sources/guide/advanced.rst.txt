.. _advanced-usage:

Advanced usage of openFPGALoader
################################

Resetting an FPGA
=================

.. code-block:: bash

    openFPGALoader [options] -r

Using negative edge for TDO's sampling
======================================

If transaction are unstable you can try to change read edge by using

.. code-block:: bash

    openFPGALoader [options] --invert-read-edge

Reading the bitstream from STDIN
================================

.. code-block:: bash

    cat /path/to/bitstream.ext | openFPGALoader --file-type ext [options]

``--file-type`` is required to detect file type.

.. NOTE::
  It's possible to load a bitstream through network:

  .. code-block:: bash

    # FPGA side
    nc -lp port | openFPGALoader --file-type xxx [option

    # Bitstream side
    nc -q 0 host port < /path/to/bitstream.ext

Automatic file type detection bypass
====================================

Default behavior is to use file extension to determine file parser.
To avoid this mechanism ``--file-type type`` must be used.

FT231/FT232 bitbang mode and pins configuration
===============================================

FT232R and ft231X may be used as JTAG programmer.
JTAG communications are emulated in bitbang mode.

To use these devices user needs to provides both the cable and the pin mapping:

.. code-block:: bash

    openFPGALoader [options] -cft23XXX --pins=TDI:TDO:TCK:TMS /path/to/bitstream.ext

where:

* ft23XXX may be ``ft232RL`` or ``ft231X``.
* TDI:TDO:TCK:TMS may be the pin ID (0 <= id <= 7) or string value.

allowed values are:

===== ==
value ID
===== ==
 TXD  0
 RXD  1
 RTS  2
 CTS  3
 DTR  4
 DSR  5
 DCD  6
 RI   7
===== ==

Writing to an arbitrary address in flash memory
===============================================

With FPGA using an external SPI flash (*xilinx*, *lattice ECP5/nexus/ice40*, *anlogic*, *efinix*) option ``-o`` allows
one to write raw binary file to an arbitrary adress in FLASH.

Using an alternative directory for *spiOverJtag*
================================================

By setting ``OPENFPGALOADER_SOJ_DIR`` it's possible to override default
*spiOverJtag* bitstreams directory:

.. code-block:: bash

    export OPENFPGALOADER_SOJ_DIR=/somewhere
    openFPGALoader xxxx

or

.. code-block:: bash

    OPENFPGALOADER_SOJ_DIR=/somewhere openFPGALoader xxxx
