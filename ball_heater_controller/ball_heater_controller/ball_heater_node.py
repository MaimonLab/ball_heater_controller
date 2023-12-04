#!/usr/bin/env python3
import struct
import sys
import time
from typing import Optional

import rclpy
import serial.tools.list_ports
from ament_index_python.packages import get_package_share_directory
from ball_heater_interfaces.msg import BallHeaterSetTemp, BallHeaterStatus
from event_data_logging import CSVWriter
from event_data_logging.data_handling import flatten_dictionary
from event_data_logging.ros2_message_handling import (
    convert_ros2_msg_to_nanosecond_stamped_dict,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

"""
When not using the compiled version of the node, we need to import 
    from a different folder structure.
This is the case when executing the node directly: 
    python3 ball_heater.py
"""
if __name__ == "__main__":
    from ball_heater_driver import BallHeaterDriver
else:
    from ball_heater_controller.ball_heater_driver import BallHeaterDriver

WORKSPACE = get_package_share_directory("eternarig_experiment_logic").split("/install")[
    0
]


def find_port_for_serial(serial_id: str) -> Optional[str]:
    """Returns port string if found, otherwise returns None"""

    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        if port.serial_number == serial_id:
            return port.device

    return None


class BallHeaterNode(Node):
    def __init__(self):
        super().__init__("ball_heater_node")

        default_param = {
            "ball_heater_set_temp_topic": "ball_heater_controller/set_temp_topic",
            "ball_heater_status_topic": "ball_heater_controller/status_topic",
            "serial_port": "/dev/ttyACM0",
            "device_serial_number": None,
            "verbose_logging": False,
            "shutdown_on_close": True,
            "output_filename": (
                f"{WORKSPACE}/src/" f"ball_heater_controller/data/test_filename"
            ),
            "status_interval": 1.0,
            "pid_kp": 120,
            "pid_ki": 0.2,
            "pid_kd": 0.1,
        }
        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        self.verbose_logging = self.get_parameter("verbose_logging").value

        port_from_param = self.get_parameter("serial_port").value
        device_serial_number = self.get_parameter("device_serial_number").value

        # throw error if Serial id consists only of numbers and is not specified as string
        if not (type(device_serial_number) is str) and (
            device_serial_number is not None
        ):
            raise TypeError(
                f"device_serial_number should be of type string! It is currently type: {type(device_serial_number)}"
            )

        # if a serial number is specified, get a port to try and open
        if device_serial_number is not None:
            port_to_try = find_port_for_serial(device_serial_number)

            if port_to_try is None:
                self.get_logger().error(
                    f"No serial port found for serial number {device_serial_number}, using mock serial"
                )
            elif port_to_try is not None:
                self.get_logger().info(
                    f"device_serial_number specified in config: {device_serial_number}"
                )
        elif port_from_param is not None:
            port_to_try = port_from_param
        else:
            port_to_try = "not specified"

        # if a port is specified, try to open it. Open a mock part in case of failure
        if port_to_try != "not specified":
            try:
                self.ball_heater = BallHeaterDriver(port_to_try)
                if port_to_try is not None:
                    self.get_logger().info(f"Opening Serial Port {port_to_try}")
            except serial.SerialException as e:
                self.get_logger().error(
                    f"Error opening serial port {port_to_try}. Using mock serial"
                )
                self.ball_heater = BallHeaterDriver()
        else:
            self.get_logger().error(
                f"No port or serial number specified. Using mock serial"
            )
            self.ball_heater = BallHeaterDriver()

        qos_subscription = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        set_temp_topic = self.get_parameter("ball_heater_set_temp_topic").value
        self.create_subscription(
            BallHeaterSetTemp, set_temp_topic, self.set_temp_callback, qos_subscription
        )

        qos_publisher = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        status_topic = self.get_parameter("ball_heater_status_topic").value
        self.pub_status = self.create_publisher(
            BallHeaterStatus, status_topic, qos_publisher
        )

        output_filename = self.get_parameter("output_filename").value
        self.csv_writer = CSVWriter(output_filename + "_light_sugar_commands.csv")

        self.set_pid_parameters()
        self.create_timer(
            self.get_parameter("status_interval").value,
            self.publish_status,
        )

    def set_pid_parameters(self):
        self.pid_params = [
            self.get_parameter("pid_kp").value,
            self.get_parameter("pid_ki").value,
            self.get_parameter("pid_kd").value,
        ]

        self.ball_heater.send_command("set_pid_parameters", self.pid_params)
        if self.verbose_logging:
            self.get_logger().info(f"set pid params to (kp, ki, kd): {self.pid_params}")

    def shutdown(self):
        success, return_data = self.ball_heater.send_command(
            "set_control_mode", [self.ball_heater.control_mode_dict["standby"]]
        )
        if not success:
            self.get_logger().error(f"failed to change control_mode {return_data}")

    def set_temp_callback(self, commands_msg):
        target_temp = commands_msg.target_temp
        success, return_data = self.ball_heater.send_command(
            "set_target_temp", [target_temp]
        )
        if not success:
            self.get_logger().warn(f"Unsuccessful set temp command")

    def publish_status(self):
        status_msg = BallHeaterStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()

        success, status_dict = self.ball_heater.send_command("status")
        if not success:
            self.get_logger().warn(f"unsuccessful command, status: {status_dict}")
            return

        try:
            status_msg.target_temp = status_dict["target_temp"]
            status_msg.heater_temp = status_dict["heater_temp"]
            status_msg.ball_heater_pwm = status_dict["ball_heater_pwm"]
            status_msg.aux_therm_temp = status_dict["aux_therm_temp"]
            status_msg.control_mode = self.ball_heater.control_mode_dict[
                status_dict["control_mode"]
            ]
        except:
            self.get_logger().error(
                f"Getting / parsing status failed, not publishing, {sys.exc_info()}"
            )

        log_dict = convert_ros2_msg_to_nanosecond_stamped_dict(status_msg)
        header, data_line = flatten_dictionary(log_dict)
        if not self.csv_writer.header_initialized:
            self.csv_writer.save_header(header)
        self.csv_writer.save_line(data_line)

        self.pub_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    ball_heater_node = BallHeaterNode()
    try:
        rclpy.spin(ball_heater_node)
    except KeyboardInterrupt:
        if ball_heater_node.verbose_logging:
            ball_heater_node.get_logger().info("node stopped cleanly")
    except BaseException:
        ball_heater_node.get_logger().error(
            f"exception in ball_heater_node:{ sys.exc_info()}"
        )
    finally:
        if ball_heater_node.get_parameter("shutdown_on_close"):
            try:
                ball_heater_node.shutdown()
                if ball_heater_node.verbose_logging:
                    ball_heater_node.get_logger().info(
                        f"Sent ball_heater shutdown command"
                    )
            except:
                ball_heater_node.get_logger().error(
                    f"Failed to send shutdown command: {sys.exc_info()}"
                )

        rclpy.shutdown()


if __name__ == "__main__":
    main()
