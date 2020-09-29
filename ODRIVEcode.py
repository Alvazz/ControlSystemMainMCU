#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
from datetime import datetime

odrive_velocity_gain_parameter = 0.0005000000237487257/2.2

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
my_drive = odrive.find_any(serial_number = "206E39864D4D")
my_drive.axis0.requested_state = AXIS_STATE_IDLE
my_drive.axis1.requested_state = AXIS_STATE_IDLE
time.sleep(1)
# Find an ODrive that is connected on the serial port /dev/ttyUSB0
#my_drive = odrive.find_any("serial:/dev/ttyUSB0")

#Odrive Parameter Adjustment:
my_drive.axis0.controller.config.vel_gain = odrive_velocity_gain_parameter
my_drive.axis1.controller.config.vel_gain = odrive_velocity_gain_parameter
time.sleep(1)

# Calibrate motor and wait for it to finish
print("starting calibration...")
my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while ((my_drive.axis0.current_state and my_drive.axis1.current_state) != AXIS_STATE_IDLE):
    time.sleep(0.1)

my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# To read a value, simply read the property
print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

# Or to change a value, just assign to the property
my_drive.axis0.controller.pos_setpoint = 3.14
my_drive.axis1.controller.pos_setpoint = 3.14
print("Position setpoint is " + str(my_drive.axis0.controller.pos_setpoint))
print("Position setpoint is " + str(my_drive.axis1.controller.pos_setpoint))

# And this is how function calls are done:
for i in [1,2,3,4]:
    print('voltage on GPIO{} is {} Volt'.format(i, my_drive.get_adc_voltage(i)))

# A sine wave to test
t0 = time.monotonic()
"""
while True:
    now = datetime.now().strftime("%H:%M:%S") 
    setpoint = 10000.0 * math.sin((time.monotonic() - t0)*20)
    my_drive.axis0.controller.pos_setpoint = setpoint
    my_drive.axis1.controller.pos_setpoint = setpoint
    print(now, "    Setpoint: ", setpoint, "    Current: ", my_drive.axis0.motor.current_control.Iq_measured, "     ", my_drive.axis1.motor.current_control.Iq_measured)
    time.sleep(0.01)
"""
my_drive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
my_drive.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

while True:
    now = datetime.now().strftime("%H:%M:%S") 
    my_drive.axis0.controller.vel_setpoint = 1000000
    my_drive.axis1.controller.vel_setpoint = 1000000
    print(now, "    Current:    ", round(my_drive.axis0.motor.current_control.Iq_measured, 5), "     ", round(my_drive.axis1.motor.current_control.Iq_measured, 5))
    




