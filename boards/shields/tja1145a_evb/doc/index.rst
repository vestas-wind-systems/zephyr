.. _tja1145a_evb_shield:

NXP TJA1145A-EVB
################

Overview
********

The NXP `TJA1145A-EVB`_ features a `TJA1145A`_ high speed CAN transceiver with support for Partial
Networking and CAN FD bitrates up to 5 Mbit/s.

.. figure:: tja1145a_evb.jpg
   :align: center
   :alt: TJA1145A-EVB

   TJA1145A-EVB (Credit: NXP)

Requirements
************

This shield can only be used with a board which provides a configuration for Arduino connectors and
defines node aliases for SPI and GPIO interfaces (see :ref:`shields` for more details).

Pin Assignments
===============

+-----------------------+---------------------------------------------+
| Shield Connector Pin  | Function                                    |
+=======================+=============================================+
| D10                   | SCS                                         |
+-----------------------+---------------------------------------------+
| D11                   | SDI                                         |
+-----------------------+---------------------------------------------+
| D12                   | SDO                                         |
+-----------------------+---------------------------------------------+
| D13                   | SCK                                         |
+-----------------------+---------------------------------------------+

The TJA1145A RXD and TXD signals are not present in the Arduino connectors. These signals must be
manually connected to the CAN RX and CAN TX signals of the board used.

.. note::
    The TJA1145A-EVB requires an external power source connected in order to operate correctly.

Programming
***********

Set ``-DSHIELD=tja1145a_evb`` when you invoke ``west build``. For example:

.. zephyr-app-commands::
   :zephyr-app: tests/drivers/can/api
   :board: frdm_k64f
   :shield: tja1145a_evb
   :goals: build

.. _TJA1145A-EVB:
   https://www.nxp.com/products/interfaces/can-transceivers/can-with-flexible-data-rate/tja1145a-evaluation-board:TJA1145A-EVB

.. _TJA1145A:
   https://www.nxp.com/products/interfaces/can-transceivers/can-with-flexible-data-rate/high-speed-can-transceiver-with-partial-networking-can-fd-data-rates-up-to-5-mbit-s:TJA1145A
