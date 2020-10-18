#!/usr/bin/python3

from gpiozero import DigitalInputDevice
import os

monitor_gpio = 1 #State of the power switch

switch = DigitalInputDevice(monitor_gpio, pull_up=None, active_state=False)
if switch.value == True:
	print("No Autoshutdown circuit, so terminate script")
	quit()

#Wait until power switch off, the shut down	
switch.wait_for_active()
os.system("shutdown now")


