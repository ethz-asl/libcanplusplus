Author: Christian Gehring
Date: Mar 16, 2012

This program controls a 4pole Maxon Motor using an EPOS 2 motor controller.
It communicates by means of CAN channel 0.
The libCAN is without SL and with PThread.
The program runs a state machine, which initializes the motor first and turns the motor with constant speed.
The program was successfully tested.


