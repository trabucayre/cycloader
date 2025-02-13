.. _anlogic:

Anlogic notes
#############

Sipeed Lichee Tang
==================

For this target, *openFPGALoader* supports *svf* and *bit*.

bit file load (memory)
----------------------

.. code-block:: bash

    openFPGALoader -m -b licheeTang /somewhere/project/prj/*.bit

Since ``-m`` is the default, this argument is optional.

bit file load (spi flash)
-------------------------

.. code-block:: bash

    openFPGALoader -f -b licheeTang /somewhere/project/prj/*.bit

svf file load
-------------

It's possible to produce this file by using *TD*:

* Tools->Device Chain
* Add your bit file
* Option : Create svf

or by using `prjtang project <https://github.com/mmicko/prjtang>`__:

.. code-block:: bash

    mkdir build
    cd build
    cmake ../
    make

Now a file called *tangbit* is present in current directory and has to be used as follows:

.. code-block:: bash

    tangbit --input /somewhere.bit --svf bitstream.svf

.. code-block:: bash

    openFPGALoader -b licheeTang /somewhere/*.svf
