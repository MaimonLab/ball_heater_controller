import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import PySimpleGUI as sg
from typing import Callable

import rclpy
from ball_heater_interfaces.msg import BallHeaterSetTemp, BallHeaterStatus
from ament_index_python.packages import get_package_share_directory
from maimon_classes.basic_node import BasicNode
from rclpy.node import Node


@dataclass
class GuiElement:
    name: str
    service_name: str
    min: float
    max: float
    default: float = None
    gui_element: Callable = sg.Slider
    changed: bool = False

    def __post_init__(self) -> None:
        if self.default is None:
            self.default = (self.min + self.max) / 2


MAX_TEMP = 100
MIN_TEMP = 0


class BallHeaterGui(BasicNode):
    def __init__(self):
        super().__init__(
            name="ball_heater_gui",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.default_param = {
            "ball_heater_set_temp_topic": "ball_heater/set_temp_topic",
            "ball_heater_status_topic": "ball_heater/status_topic",
        }

        self.register_publisher(
            BallHeaterSetTemp, "ball_heater_set_temp_topic", reliable=True
        )
        self.register_subscriber(
            BallHeaterStatus,
            "ball_heater_status_topic",
            callback=self.status_callback,
        )

        self.layout = []

        self.layout.append(
            [
                sg.Text("Temperature Setpoint:"),
                sg.InputText("25", key="setpoint", enable_events=True, size=(5, 1)),
                sg.Button("Set", key="set_button", size=(5, 1)),
            ]
        )
        self.layout.append([sg.Multiline(size=(50, 10), key="parameters_box")])
        self.setpoint = 25

        self.window = sg.Window(
            "Ball Heater GUI",
            self.layout,
            location=(800, 400),
            # size=(500, 500),
            return_keyboard_events=True,
            resizable=True,
            finalize=True,
        )
        self.signal = signal.signal(signal.SIGINT, self.signal_interrupt)

        self.create_timer(0.02, self.listen_events)

    def listen_events(self):
        event, values = self.window.read(timeout=10)

        if event == "__TIMEOUT__":
            return
        else:
            print(event, values)

        if event == sg.WIN_CLOSED or event == "Exit":
            self.window.close()
            # self.destroy_node()
            sys.exit(0)

        if "setpoint" in event and values["setpoint"] != "":
            try:
                self.setpoint = float(values["setpoint"])

            except:
                self.window["setpoint"].update(f"{self.setpoint:0.0f}")

        if "set_button" in event or "Return" in event:
            self.setpoint = float(np.clip(self.setpoint, MIN_TEMP, MAX_TEMP))
            self.window["setpoint"].update(f"{self.setpoint}")

            setpoint_msg = BallHeaterSetTemp()
            setpoint_msg.header.stamp = self.get_clock().now().to_msg()
            setpoint_msg.target_temp = self.setpoint
            self.pub_set_temp_topic.publish(setpoint_msg)

    def status_callback(self, status_msg):
        """Print the status data when the status topic is received.

        Args:
            status_msg (ROS message): Status Message
        """
        formatted_msg = (
            f"Target Temp: {status_msg.target_temp:.2f} \n"
            f"Ball Heater PWM: {status_msg.ball_heater_pwm:.2f} \n"
            f"Heater Temp: {status_msg.heater_temp:.2f} \n"
            f"Aux Therm Temp: {status_msg.aux_therm_temp:.2f} \n"
            f"Control Mode: {status_msg.control_mode}"
        )

        self.window["parameters_box"].update(formatted_msg)

    def signal_interrupt(self, *_):
        self.window.close()
        raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    BallHeaterGui().run()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
