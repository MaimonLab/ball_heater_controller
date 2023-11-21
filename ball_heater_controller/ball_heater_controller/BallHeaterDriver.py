#!/usr/bin/env python3

import csv
import os
import struct
import threading
import time
from sys import platform

import serial

try:
    from ball_heater_controller.list_ports import list_ports
except ModuleNotFoundError:
    from list_ports import list_ports


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
    """Class to interface with the ball heater controller PCB."""

    def __init__(self, port=None):
        self.port = port

        self.start = b"\xFE\xED"
        self.end = b"\xBE\xAD"
        self.baud = 115200
        self.read_timeout = 1
        self.max_send_attempts = 5
        # todo: fix this to get from controller.
        self.log_fieldnames = HEATER_COMMANDS["return_status_format"]["arg_names"]
        self.log_fieldnames = ["time"] + self.log_fieldnames

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

        try:
            in_data = self.ser.read_all()
            while (b"\xBE\xAD" not in in_data) and (
                time.time() - start_time
            ) < self.read_timeout:
                in_data += self.ser.read_all()
        except serial.SerialException:
            print("Serial exception")
            return False, None
        except BaseException as e:
            print(f"Error: {e}")
            return False, None
        # check to make sure echoed command is the same as the one sent.
        if self.command_bytestring not in in_data:
            return False, None

        returned_data = in_data.split(b"\xFE\xED")[0]

        if self.command_name not in ["status", "return_pid_parameters"]:
            return True, returned_data

        if self.command_name == "return_pid_parameters":
            arg_names = ["kp", "ki", "kd"]
            arg_types = ["float", "float", "float"]

        elif self.command_name == "status":
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

    def begin_logging(self, log_interval, log_file_path):
        """Starts a loop to regularly get the  from the controller and record.

        ::log_interval:: interval in seconds to get status and record it.
        ::log_file_path::Path to csv file to save data to.
        """
        self.log_file_name = log_file_path
        self.log_interval = log_interval
        self.log_stop_event = threading.Event()
        self.logging_thread = threading.Thread(target=self.logging_thread_call)
        self.logging_thread.start()

    def logging_thread_call(self):
        """Function that runs in separate thread as a loop to log regularly until stopped."""
        while not self.log_stop_event.is_set():
            self.get_status_and_log()
            time.sleep(self.log_interval)

    def stop_logging(self):
        """Stops the logging loop by setting the self.log_stop_event."""
        self.log_stop_event.set()

    def get_status_and_log(self):
        """Gets the current status from the controller and logs to the logfile"""
        current_status = self.send_command("status")
        if current_status[0]:
            log_file_exists = os.path.isfile(self.log_file_name)
            with open(self.log_file_name, "a") as logfile:
                csv_writer = csv.DictWriter(logfile, fieldnames=self.log_fieldnames)

                if not log_file_exists:
                    csv_writer.writeheader()

                data = current_status[1]
                if not isinstance(data, dict):
                    print(f"data is not dict: {data}")
                    return
                if data:
                    data["time"] = time.time()
                    csv_writer.writerow(data)


def main():
    ports = [
        port
        for port in list_ports()
        if port["manufacturer"] is not None and "Silicon Labs" in port["manufacturer"]
    ]
    if len(ports) == 0:
        print("No Silicon Labs ports found.")
        return
    elif len(ports) > 1:
        print("Multiple Silicon Labs ports found, select one")
        for i, port in ports:
            print(f"{i}: {port['port']}")
        port_index = int(input("Enter port index: "))
        port = ports[port_index]
    else:
        print(f"Using port: {ports[0]['port']}")
        port = ports[0]["port"]

    ball_heater = BallHeaterDriver()
    status_dict = ball_heater.send_command("status")
    print(f"status: {status_dict}")


if __name__ == "__main__":
    main()
