.. _compatibility:boards:

Boards
######

.. NOTE::
  `arty` can be any of the board names from the first column.

.. code-block:: bash

    openFPGALoader -b arty bitstream.bit # Loading in SRAM (volatile)
    openFPGALoader -b arty -f bitstream.bit # Writing in flash (non-volatile)

.. include:: boards.inc

* IF: Internal Flash
* EF: External Flash
* AS: Active Serial flash mode
* NA: Not Available
* NT: Not Tested
