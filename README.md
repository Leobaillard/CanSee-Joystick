# CanSee Joystick

Use your Renault Zoe as a joystick!

## Disclaimer
This project is still a __work in progress__, so take it as is.
No safety is in place for now to prevent you from using this when the car is not parked, so use at your own risk.
I will not be held accountable for any damage done to you or your car resulting from using this software.
Please be careful but don't forget to have fun!

## How it works
It uses the computer's bluetooth to connect to a [CanSee](https://gitlab.com/jeroenmeijer/cansee) dongle
and then sends CAN messages to ask for input values used as joystick controls. Currently
it uses:
 - steering wheel angle (actually a portion of it due to bad code, but that has the advantage to not require too much
   input for the full joystick range, thus saving the tires a bit)
 - throttle pedal position
 - brake pedal status (pressed or not)
 - steering wheel buttons voltages (to figure out which button is pressed)

For this to work, the CanSee dongle has to be plugged in, the car has to be ON and the "engine" (stupid terminology)
has to be ON as well.
Then, launch the script using root privileges (e.g using sudo). Once the script runs, you can test the controls
by verifying the console output. Once everything works, use a joystick calibration utility (e.g jstest or jstest-gtk if 
you want a GUI) and calibrate the axis and buttons. For now, the steering wheel axis has to be __reversed__.

Once everything is running, launch your favorite game and play!

You can find an example of this working using Super Tux Kart here: https://www.youtube.com/watch?v=4IjWyTW1NXk

## Requirements
 - Python 3 and uinput package
 - Root privileges (for now as it uses rfcomm for the Bluetooth but this will change)
 - A phase 2 Renault Zoe
 - A [CanSee](https://gitlab.com/jeroenmeijer/cansee) dongle

## TODO
 - Find a better way to implement Bluetooth, without needing root privileges
 - Add safeties to prevent use when the car is not parked
 - Fix the crappy implementation of fields parsing
 - Fix the steering wheel axis and value mapping
 - Add Kalman filters to the input (they are pretty stable as is though)