#!/usr/bin/python3

"""
    CanSee Joystick
    Use your Renault Zoe as a joystick

    Copyright (C) 2020 - Léopold Baillard

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or any
    later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import os
import sys
import serial
import time
import subprocess
import signal
import fcntl
import uinput

HCI_DEV = "/dev/rfcomm0"
FIELD_TYPE_MASK = 0x700
FIELD_TYPE_STRING = 0x200
FIELD_TYPE_SIGNED = 0x100
FIELDS = {
    "gateway_open": {
        "id": "0x18daf1d2",
        "request": "5003",
        "options": None
    },
    "steering_angle": {
        "id": "0x00000700",
        "request": "220113",
        "response": "620113",
        "options": 0x2ff,
        "offset": 0,
        "resolution": 0.1
    },
    "throttle_position": {
        "id": "0x18daf1da",
        "request": "22202e",
        "response": "62202e",
        "options": 0xff,
        "offset": 0,
        "resolution": 0.125
    },
    "break_pressed": {
        "id": "0x18daf1da",
        "request": "22200f",
        "response": "62200f",
        "options": 0xff,
        "offset": 0,
        "resolution": 1
    },
    "button_pressed": {
        "id": "0x18daf1da",
        "request": "222039",
        "response": "622039",
        "options": 0xff,
        "offset": 0,
        "resolution": 1
    }
}
BT_ADDR = "xx:xx:xx:xx:xx:x"  # CanSee BT address
BT_DEV = "hci0"
HCI_OPEN_TIMEOUT = 5
CAN_GATEWAY_REOPEN_TIMEOUT = 1500

# Joystick config
joystick_events = (
    uinput.ABS_WHEEL,
    uinput.ABS_THROTTLE,
    uinput.BTN_0,
    uinput.BTN_1,
    uinput.BTN_2
)


class CanSeeJoystick:
    def __init__(self):
        self.hci_proc = None
        self.joystick = None
        self.throttle_position = 0
        self.steering_angle = 0
        self.break_pressed = 0
        self.button_pressed = 0
        self.gateway_last_open = 0

    def run(self):
        # Init bluetooth
        self.hci_proc = self.init_bluetooth()

        # Init serial
        ser = self.init_serial()

        # Init joystick
        self.joystick = uinput.Device(joystick_events, "CanSee-Joystick")

        # Start command loop
        self.start_command_loop(ser, self.hci_proc)

    @staticmethod
    def non_block_read(output):
        fd = output.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        try:
            return output.read()
        except:
            # Fail silently, not mission critical
            return ""

    @staticmethod
    def field_is_string(field):
        return (field["options"] & FIELD_TYPE_MASK) == FIELD_TYPE_STRING

    @staticmethod
    def field_is_signed(field):
        return (field["options"] & FIELD_TYPE_MASK) == FIELD_TYPE_SIGNED

    def parse_message(self, field, data):
        if len(data) > len(field["response"]):
            data = data[len(field["response"]):]
            if self.field_is_signed(field):
                return int(data, 16) - field["offset"]
            elif self.field_is_string(field):
                if data[:2] == "00":
                    return int(data[2:], 16) - field["offset"]
                elif data[:2] == "ff":
                    # FIXME: quick hack to get value bounds
                    return -255 + int(data[2:], 16) - field["offset"]
            else:
                return int(data, 16) - field["offset"]
        return None

    def response_to_message(self, field, ser, timeout=1000):
        response = self.send_and_wait(field, ser, timeout)

        if len(response) == 0:
            return None

        # Split up the fields
        pieces = response.strip().split(',')
        if len(pieces) < 2:
            return None

        # Check for NaN
        try:
            message_id = int(pieces[0].strip(), 16)
        except:
            # If the int cast errors out, return None to ignore the value
            return None

        # Check that ID matches what we want
        if message_id != int(field["id"], 16):
            return None

        # Parse message
        return self.parse_message(field, pieces[1].strip().lower())

    def send_and_wait(self, field, ser, timeout):
        ser.flushInput()
        ser.flushOutput()
        ser.write(bytes(self.get_command_from_field(field) + "\n", 'UTF-8'))
        response = ''
        start = time.time() * 1000.0

        while True:
            if ser.inWaiting() > 0:
                response += ser.read(ser.inWaiting()).decode('ascii')
                if response.endswith('\n'):
                    return response
            if ((time.time() * 1000.0) - start) > timeout:
                print("Timeout")
                break

        return response

    @staticmethod
    def get_command_from_field(field):
        if "id" not in field or "request" not in field:
            raise ValueError

        if "response" in field:
            return "i{id},{request},{response}".format(
                id=field["id"],
                request=field["request"],
                response=field["response"]
            )
        else:
            return "i{id},{request}".format(
                id=field["id"],
                request=field["request"]
            )

    def init_bluetooth(self):
        # FIXME: use a Python BT library instead
        # Init rfcomm if needed
        proc = None
        hci_open_time = time.time()
        if not os.path.exists(HCI_DEV):
            print("Starting RFcomm on {dev} with address {addr}".format(dev=BT_DEV, addr=BT_ADDR), end='')
            try:
                proc = subprocess.Popen("/usr/bin/rfcomm connect {device} {addr}".format(device=BT_DEV, addr=BT_ADDR),
                                        shell=True,
                                        stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE
                                        )
            except subprocess.CalledProcessError:
                print(" NOK\nERROR: an error occurred while starting rfcomm")
                sys.exit(1)
            else:
                print(" OK")

        # Wait for HCI to open before continuing
        print("Waiting for HCI to open.", end='')
        while not os.path.exists(HCI_DEV):
            # If the process has terminated, we should exit
            if proc.poll() is not None:
                print("ERROR")
                self.non_block_read(proc.stderr)
                # Stop there, no point in continuing
                sys.exit(2)

            if (time.time() - hci_open_time) < HCI_OPEN_TIMEOUT:
                # Wait longer for the hci to open
                time.sleep(0.1)
                print(".", end='')
            else:
                print("ERR")
                # Close remnants of the process
                proc.send_signal(signal.SIGINT)
                # Stop there, no point in continuing
                sys.exit(2)
        print("OK")
        return proc

    @staticmethod
    def init_serial():
        # Open serial port
        serial_conn = serial.Serial(HCI_DEV)
        return serial_conn

    def start_command_loop(self, ser):
        while True:
            try:
                # Keep the gateway opened
                ms = time.time() * 1000.0
                if (ms - self.gateway_last_open) >= CAN_GATEWAY_REOPEN_TIMEOUT:
                    # print("Sending gateway keepalive")
                    self.gateway_last_open = ms
                    ser.write(bytes(self.get_command_from_field(FIELDS["gateway_open"]) + "\n", 'UTF-8'))

                # Send data queries
                self.steering_angle = self.response_to_message(FIELDS["steering_angle"], ser)
                print("Steering: {}".format(self.steering_angle), end=' | ')
                self.throttle_position = self.response_to_message(FIELDS["throttle_position"], ser)
                print("Throttle: {}".format(self.throttle_position), end=' | ')
                self.break_pressed = self.response_to_message(FIELDS["break_pressed"], ser)
                print("Break: {}".format(self.break_pressed), end=' | ')
                self.button_pressed = self.response_to_message(FIELDS["button_pressed"], ser)
                print("Button: {}".format(self.button_pressed), end=' | ')
                print("")

                # TODO: apply Kalman filter

                # Set joystick values
                if self.steering_angle is not None:
                    self.joystick.emit(uinput.ABS_WHEEL, self.steering_angle)
                if self.throttle_position is not None:
                    self.joystick.emit(uinput.ABS_THROTTLE, self.throttle_position)
                if self.break_pressed is not None:
                    self.joystick.emit(uinput.BTN_0, 1 if self.break_pressed == 4 else 0)
                if self.button_pressed is not None:
                    # TODO: better handle buttons voltages
                    if self.button_pressed >= 4000:  # No buttons pressed
                        self.joystick.emit(uinput.BTN_1, 0)
                        self.joystick.emit(uinput.BTN_2, 0)
                    elif self.button_pressed >= 3200:  # Cruise control "+" pressed
                        self.joystick.emit(uinput.BTN_1, 1)
                        self.joystick.emit(uinput.BTN_2, 0)
                    elif self.button_pressed >= 2500:  # Cruise control "-" pressed
                        self.joystick.emit(uinput.BTN_1, 0)
                        self.joystick.emit(uinput.BTN_2, 1)
            except KeyboardInterrupt:
                break
            except serial.SerialException as e:
                print("Serial error: {}".format(e.strerror))
                if ser:
                    ser.close()
                if self.hci_proc:
                    self.hci_proc.send_signal(signal.SIGINT)
                sys.exit(3)

        print("Closing connections...", end='')
        if ser:
            ser.close()
        if self.hci_proc:
            self.hci_proc.send_signal(signal.SIGINT)
        print("OK, bye!")
        sys.exit(0)


if __name__ == "__main__":
    cansee_joystick = CanSeeJoystick()
    cansee_joystick.run()
