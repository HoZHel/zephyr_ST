.. _steval_idb013v1:

ST BlueNRG-LPS Evaluation Platform
##################################

Overview
********


Hardware
********


Supported Features
==================

The Zephyr steval_idb013v1 board configuration supports the following hardware features:


+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| NVIC      | on-chip    | nested vector interrupt controller  |
+-----------+------------+-------------------------------------+
| UART      | on-chip    | serial port-polling;                |
|           |            | serial port-interrupt               |
+-----------+------------+-------------------------------------+
| PINMUX    | on-chip    | pinmux                              |
+-----------+------------+-------------------------------------+
| ...       | on-chip    | ...                                 |
+-----------+------------+-------------------------------------+

Other hardware features are not yet supported on this Zephyr port.

The default configuration can be found in the defconfig file:
:zephyr_file:`boards/st/steval_idb013v1/steval_idb013v1_defconfig`


Connections and IOs
===================

Programming and Debugging
*************************


Flashing
========


Flashing an application to STEVAL_IDB013v1
------------------------------------------


Debugging
=========
