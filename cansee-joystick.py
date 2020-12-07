#!/usr/bin/python3

import os
import sys
import serial
import time
import subprocess
import signal
import fcntl
import threading
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
    }
}
BT_ADDR = "10:52:1C:5D:76:C2"  # CanSee
# BT_ADDR = "F0:08:D1:D7:C0:C2"  # Dev
BT_DEV = "hci0"
HCI_OPEN_TIMEOUT = 5
CAN_GATEWAY_REOPEN_TIMEOUT = 1500

# Output values
throttle_position = 0
steering_angle = 0

# Joystick config
joystick_events = (
    uinput.ABS_WHEEL,
    uinput.ABS_THROTTLE,
    uinput.ABS_BRAKE
)


class DataThread():
    def __init__(self, name):
        self.name = name

    def run(self):
        global throttle_position
        global steering_angle

        # Init bluetooth
        self.hci_proc = self.init_bluetooth()

        # Init serial
        ser = self.init_serial()

        # Init joystick
        self.joystick = uinput.Device(joystick_events, "CanSee-Joystick")

        # Start command loop
        self.start_command_loop(ser, self.hci_proc)

    def non_block_read(self, output):
        fd = output.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        try:
            return output.read()
        except:
            return ""

    def field_is_string(self, field):
        return (field["options"] & FIELD_TYPE_MASK) == FIELD_TYPE_STRING

    def field_is_signed(self, field):
        return (field["options"] & FIELD_TYPE_MASK) == FIELD_TYPE_SIGNED

    def parse_message(self, field, data):
        if len(data) > len(field["response"]):
            data = data[len(field["response"]):]
            if self.field_is_signed(field):
                return (int(data, 16) - field["offset"])
            elif self.field_is_string(field):
                if data[:2] == "00":
                    # return (int(data[2:], 16) - field["offset"]) * field["resolution"]
                    return (int(data[2:], 16) - field["offset"])
                elif data[:2] == "ff":
                    # return (- int(data[2:], 16) - field["offset"]) * field["resolution"]
                    return (-255 + int(data[2:], 16) - field["offset"])
            else:
                # return (int(data, 16) - field["offset"]) * field["resolution"]
                return (int(data, 16) - field["offset"])
        return None

    def response_to_message(self, field, serial, timeout=1000):
        response = self.send_and_wait(field, serial, timeout)

        if len(response) == 0:
            return None

        # Split up the fields
        pieces = response.strip().split(',')
        if len(pieces) < 2:
            return None

        # Check for NaN
        try:
            id = int(pieces[0].strip(), 16)
        except:
            return None

        # Check ID
        if id != int(field["id"], 16):
            # print(id)
            return None

        # Parse message
        return self.parse_message(field, pieces[1].strip().lower())

    def send_and_wait(self, field, serial, timeout):
        serial.flushInput()
        serial.flushOutput()
        # print("Send: {}".format(self.get_command_from_field(field)))
        serial.write(bytes(self.get_command_from_field(field) + "\n", 'UTF-8'))
        response = ''
        prev = ''
        start = time.time() * 1000.0

        while True:
            # print("Read")
            # response += serial.read(1).decode('ascii')
            # print(response)
            # if response.endswith('\n'):
            #     return response
            if serial.inWaiting() > 0:
                response += serial.read(serial.inWaiting()).decode('ascii')
                if response.endswith('\n'):
                    return response
            if ((time.time() * 1000.0) - start) > timeout:
                print("Timeout")
                break

        return response

    def get_command_from_field(self, field):
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

    def init_serial(self):
        # Open serial port
        serial_conn = serial.Serial(HCI_DEV)
        self.gateway_last_open = 0
        return serial_conn

    def start_command_loop(self, ser, hci_proc):
        global steering_angle
        global throttle_position
        while True:
            try:
                # if ser.inWaiting() > 0:
                #     data_str = ser.read(ser.inWaiting()).decode('ascii')
                #     print("Received: ", end='')
                #     print(parse_received(data_str, STEERING_WHEEL_ANGLE_CMD), end='')
                #     # print("Received: {}".format(), end='')

                # Keep the gateway opened
                ms = time.time() * 1000.0
                if (ms - self.gateway_last_open) >= CAN_GATEWAY_REOPEN_TIMEOUT:
                    # print("Sending gateway keepalive")
                    self.gateway_last_open = ms
                    ser.write(bytes(self.get_command_from_field(FIELDS["gateway_open"]) + "\n", 'UTF-8'))

                # Send data queries
                # print("Requesting steering angle")
                steering_angle = self.response_to_message(FIELDS["steering_angle"], ser)
                print(steering_angle)
                # print("Requesting throttle position")
                throttle_position = self.response_to_message(FIELDS["throttle_position"], ser)
                print(throttle_position)
                # print("Sending ", end='')
                # ser.write(bytes(STEERING_WHEEL_ANGLE_CMD + "\n", 'UTF-8'))
                # print(bytes(STEERING_WHEEL_ANGLE_CMD + "\n", 'UTF-8'))
                # print(bytes(THROTTLE_PEDAL_POS_CMD + "\n", 'UTF-8'))
                # ser.write(bytes(THROTTLE_PEDAL_POS_CMD + "\n", 'UTF-8'))

                # TODO: apply Kalman filter

                # Set joystick values
                if steering_angle is not None:
                    self.joystick.emit(uinput.ABS_WHEEL, steering_angle)
                if throttle_position is not None:
                    self.joystick.emit(uinput.ABS_THROTTLE, throttle_position)

                time.sleep(0.05)
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
    data_thread = DataThread("dataThread")
    data_thread.run()
    # data_thread.start()
    # data_thread.join()
