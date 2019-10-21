.. _lmp90100_evb_thermocouple_sample:

LMP90100 Sensor AFE Evaluation Board: Thermocouple Sample
#########################################################

Overview
********

This sample is provided as an example of how to read the absolute
temperature of a thermocouple (TC) sensor using the LMP90100 Sensor
AFE Evaluation Board shield. The sample is designed for use with a
K-type thermocouple.

Please refer to :ref:`lmp90100_evb_shield` for more information on
this shield.

Requirements
************

Prior to running the sample application, the LMP90100 EVB must be
connected to the development board according to Example #4
("Thermocouple and LM94022 Application") in the `LMP90100 Sensor AFE
Evaluation Board User's Guide`_.

Building and Running
********************

This sample runs with the LMP90100 EVB connected to any development
board with a matching Arduino connector. For this example, we use a
:ref:`frdm_k64f` board.

.. zephyr-app-commands::
   :zephyr-app: samples/shields/lmp90100_evb/thermocouple
   :board: frdm_k64f
   :goals: build
   :compact:

Sample Output
=============

 .. code-block:: console

    TODO

.. _LMP90100 Sensor AFE Evaluation Board User's Guide:
   http://www.ti.com/lit/pdf/snau028
