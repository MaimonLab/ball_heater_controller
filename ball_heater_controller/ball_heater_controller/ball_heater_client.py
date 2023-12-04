#!/usr/bin/env python3

import sys
import time

import rclpy
from ball_heater_interfaces.msg import BallHeaterSetTemp, BallHeaterStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ExampleBallHeaterClientNode(Node):
    def __init__(self):
        super().__init__("ball_heater_client")

        default_param = {
            "ball_heater_status_topic": "ball_heater_controller/status_topic",
            "ball_heater_set_temp_topic": "ball_heater_controller/temperature_commands",
            "temperature_setpoint": 35.0,
            "print_status": True,
        }
        for key, value in default_param.items():
            if not self.has_parameter(key):
                self.declare_parameter(key, value)

        qos_subscription = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        set_temp_topic = self.get_parameter("ball_heater_set_temp_topic").value
        self.pub_command = self.create_publisher(
            BallHeaterSetTemp, set_temp_topic, qos_subscription
        )
        init_msg = BallHeaterSetTemp()
        init_msg.header.stamp = self.get_clock().now().to_msg()
        init_msg.target_temp = self.get_parameter("temperature_setpoint").value

        time.sleep(0.1)
        self.pub_command.publish(init_msg)

        self.get_logger().info("Ball Heater Test Node Started.")
        self.get_logger().info(f"Setting temperature to {init_msg.target_temp}")

        qos_subscription = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        status_topic = self.get_parameter("ball_heater_status_topic").value
        self.create_subscription(
            BallHeaterStatus, status_topic, self.status_callback, qos_subscription
        )

    def status_callback(self, status_msg):
        if self.get_parameter("print_status").value:
            formatted_msg = (
                f"Target Temp: {status_msg.target_temp:.2f} \n"
                f"Ball Heater PWM: {status_msg.ball_heater_pwm:.2f} \n"
                f"Heater Temp: {status_msg.heater_temp:.2f} \n"
                f"Aux Therm Temp: {status_msg.aux_therm_temp:.2f} \n"
                f"Control Mode: {status_msg.control_mode}"
            )
            self.get_logger().info(formatted_msg)


def main(args=None):
    rclpy.init(args=args)
    ball_heater_node = ExampleBallHeaterClientNode()
    try:
        rclpy.spin(ball_heater_node)
    except KeyboardInterrupt:
        ball_heater_node.get_logger().info("node stopped cleanly")
    except BaseException:
        ball_heater_node.get_logger().error(
            f"exception in ball_heater_client:{ sys.exc_info()}"
        )
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
