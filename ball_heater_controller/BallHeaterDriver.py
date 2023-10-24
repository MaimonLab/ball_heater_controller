#!/usr/bin/env python3

import struct
import time
from sys import platform

import serial

HEATER_COMMANDS = {
    "status": {"code": 0, "arg_names": [], "arg_types": []},
    "status_header": {"code": 1, "arg_names": [], "arg_types": []},
    "set_target_temp": {"code": 2, "arg_names": ["temp"], "arg_types": ["float"]},
    "return_pid_parameters": {"code": 3, "arg_names": [], "arg_types": []},
    "set_pid_parameters": {
        "code": 4,
        "arg_names": ["kp", "ki", "kd"],
        "arg_types": ["float", "float", "float"],
    },
    "set_control_mode": {
        "code": 5,
        "arg_names": ["control_mode"],
        "arg_types": ["uint8"],
    },
    "set_heater_pwm_manual": {
        "code": 6,
        "arg_names": ["ball_heater_pwm"],
        "arg_types": ["float"],
    },
    "return_status_format": {
        "code": None,
        "arg_names": [
            "target_temp",
            "ball_heater_pwm",
            "heater_temp",
            "aux_therm_temp",
            "control_mode",
        ],
        "arg_types": [
            "float",
            "float",
            "float",
            "float",
            "uint8",
        ],
    },
}


class BallHeaterDriver:
    def __init__(self, port=None):
        self.port = port

        self.start = b"\xFE\xED"
        self.end = b"\xBE\xAD"
        self.baud = 115200
        self.read_timeout = 1
        self.max_send_attempts = 5
        # todo: fix this to get from controller.
        self.log_fieldnames = HEATER_COMMANDS["return_status_format"]["arg_names"]

        self.type_lookup = {"uint8": "B", "uint16": "H", "float": "f"}
        self.command_def_dict = HEATER_COMMANDS
        self.control_mode_dict = {
            0: "standby",
            1: "local_control",
            2: "remote_control",
            3: "manual_test",
            6: "high_temp_error",
        }
        # add reverse lookup to control_mode_dict
        self.control_mode_dict.update(
            {item[1]: item[0] for item in self.control_mode_dict.items()}
        )

        # open serial
        if self.port is not None:
            try:
                self.ser = serial.serial_for_url(
                    self.port, self.baud, rtscts=False, do_not_open=True
                )
                self.ser.open()
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except:
                raise serial.SerialException(f"Serial not found.")

    def send_command(self, command_name, arg_list=[], num_tries=1):
        """Sends the specified command to the arduino with provided arguments, after converting."""
        if self.port is None:
            return False, None

        self.command_name = command_name
        self.command = self.command_def_dict[command_name]
        self.command_bytestring = self.start + struct.pack("<B", self.command["code"])

        for arg, arg_type in zip(arg_list, self.command["arg_types"]):
            self.command_bytestring += struct.pack(
                "<" + self.type_lookup[arg_type], arg
            )

        self.command_bytestring += self.end
        self.ser.write(self.command_bytestring)

        success, return_data = self.recieve_response()

        if success:
            return True, return_data
        else:
            if num_tries > self.max_send_attempts:
                # if num_tries greater then max_send_attempts then return failure.
                # TODO: make sure this makes sense  in ROS / goes somewhere useful.

                return (
                    False,
                    f"sending command {command_name} failed after {num_tries} attempts.",
                )
            else:
                # otherwise, reccur with increased num_tries.
                return self.send_command(command_name, arg_list, num_tries + 1)

    def recieve_response(self):
        """Gets data back from the temperature controller and checks / processes it."""
        # Read data until we have the message end sequence (BEAD) or timeout elapsed.
        start_time = time.time()

        if self.port is None:
            return False, None

        in_data = self.ser.read_all()
        while (b"\xBE\xAD" not in in_data) and (
            time.time() - start_time
        ) < self.read_timeout:
            in_data += self.ser.read_all()

        # check to make sure echoed command is the same as the one sent.
        if self.command_bytestring not in in_data:
            return False, None

        returned_data = in_data.split(b"\xFE\xED")[0]
        if self.command_name != "status":
            return True, returned_data

        else:
            # process and unpack status data.
            arg_names = self.command_def_dict["return_status_format"]["arg_names"]
            arg_types = self.command_def_dict["return_status_format"]["arg_types"]
            arg_format_string = "".join(
                [self.type_lookup[arg_type] for arg_type in arg_types]
            )
            expected_size = struct.calcsize(arg_format_string)

            returned_data = in_data.split(b"\xFE\xED")[0]

            if len(returned_data) != expected_size:
                self.ser.reset_input_buffer()
                return False, None
            else:
                parsed_data = struct.unpack(arg_format_string, returned_data)
                self.status_tries = 0
                return True, {key: value for key, value in zip(arg_names, parsed_data)}


def main():
    ball_heater = BallHeater()
    status_dict = ball_heater.send_command("status")
    print(f"status: {status_dict}")


if __name__ == "__main__":
    main()
