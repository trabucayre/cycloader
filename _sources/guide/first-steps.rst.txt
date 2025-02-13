.. _first-steps:

First steps with openFPGALoader
###############################

Install
=======

Packages are available for Linux distributions, Windows (MSYS2) and macOS:

* *Arch Linux*: ``sudo pacman -S openfpgaloader``

* *Fedora*: ``sudo dnf copr enable mobicarte/openFPGALoader; sudo dnf install openFPGALoader``

* *MSYS2*: ``pacman -S mingw-w64-ucrt-x86_64-openFPGALoader``

* *macOS*: ``brew install openfpgaloader``

More instructions for other installation scenarios are available in :ref:`install`.

Programming a development board
===============================

Just simply replace ``my_fpga_board`` with any FPGA board from :ref:`compatibility:boards`
(or ``openFPGALoader --list-boards``) in any of the two commands below, depending on if you want to program the volatile
part of your FPGA (faster but not persistent) or the flash part of your FPGA (slower but persistent):

.. code-block:: bash

    openFPGALoader -b my_fpga_board my_bitstream.bit # Program to SRAM
    openFPGALoader -b my_fpga_board -f my_bitstream.bit # Program to flash

.. NOTE::
  When a bitstream file is compatible with both memory load and FLASH write, the default behavior is to load bitstream
  in memory.

Programming an "standalone" FPGA
================================

If your FPGA doesn't come with a built-in programmer or if you prefer to use an external cable, you can specify a cable
to use from :ref:`compatibility:cables` (or ``openFPGALoader --list-cables``):

.. code-block:: bash

    openFPGALoader -c my_cable my_bitstream.bit # Program to SRAM
    openFPGALoader -c my_cable -f my_bitstream.bit # Program to flash

.. NOTE::
  For some cable (like digilent adapters) signals from the converter are not just directly to the FPGA.
  For this case, the ``-c`` must be added.

.. HINT::
  FTDI/FTDI-compatible cable users: the ``-d`` option lets you specify a specific FTDI device:

  .. code-block:: bash

      openFPGALoader -d /dev/ttyUSBX

  When the ``-d`` option is not provided, openFPGALoader will opens the first FTDI adapter it finds.
  Therefore it is preferable to use this flag if your computer is connected to multiple FTDI devices.

Troubleshooting
===============

Please refer to :ref:`troubleshooting`.
