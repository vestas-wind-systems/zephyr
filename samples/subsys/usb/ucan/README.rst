.. _usb_ucan:

USB CAN Adapter Sample Application
##################################

Overview
********

Requirements
************

Building and Running
********************

.. code-block:: console

   sudo modprobe ucan
   echo 2fe3 000e | sudo tee /sys/bus/usb/drivers/ucan/new_id
