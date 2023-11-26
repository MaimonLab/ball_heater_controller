# ball_heater

ROS2 package for the Ball Heater Controller

# Dependencies
<!-- 
# Example

To test the ball_heater installation and communcation you can run:

    ros2 launch ball_heater example_ball_heater.launch.py

This will launch the example client and the eternarig_driver_node. To specify a specific device, you need to provide either the `device_serial_number` or the `device_port` parameter to the `ball_heater` in the `config/example_config.yaml` (the device_serial_number takes precedence). **If neither device serial number or device port are specified, a mock device is used that does not return status updates and throws errors when trying to publish status.**

# Finding device_serial_number

Each serial port device has a device_serial number. To list all serial port devices you can run the script:

    ros2 run light_sugar_driver list_ports -->
