# Ball Heater Controller

This repo contains the schematics and associated code for a controller to control the temperature of a ball holder or other device using closed loop PID drive of a resistive heater. There is a Python class to interact with the controller over USB-Serial, as well as a ROS2 Node to send and receive commands with the controller using ROS2.

For instructions on building a Ball Heater Controller, see [Assembly.](docs/assembly.md)

## Basic Usage

The Ball Heater Controller can be used independently of a computer by simply supplying it with AC power and attaching a ball heater and thermistor assembly to the main 4 pin output port on the front.

You can then change the mode by pressing and holding the control knob (see [Modes](#controller-modes)), and set the temperature setpoint (shown lower left as `SP:` on the LCD) by turning the knob. Current temperature is shown bottom right on the LCD.

### Python

The device can also be used with a computer for greater control and temperature logging. Interaction is done over USB-Serial, and the computer side is managed through the `BallHeaterDriver` Python class found in [`ball_heater_driver.py`](ball_heater_controller/ball_heater_controller/ball_heater_driver.py). An example of how this class can be used can be found in the Jupyter Notebook [`ball_heater_controller_testing.ipynb`](ball_heater_controller/ball_heater_controller/ball_heater_controller_testing.ipynb)

In short, after creating a `BallHeaterDriver` object, providing the correct serial port, commands can be sent using the `send_command` method. The main commands you will use are `status`, `set_target_temp`, and `set_control_mode`. Valid syntax for creating an object and sending these commands is shown below, with the first valid serial port chosen by default.

```python
import ball_heater_driver 
from list_ports import list_ports

ports = [
    port
    for port in list_ports()
    if port["manufacturer"] is not None and "Silicon Labs" in port["manufacturer"]
]
port = ports[0]["port"]
print(f"Using Port: {port}")
ball_heater = ball_heater_driver.BallHeaterDriver(port=port)

# Get and print status
success, status = ball_heater.send_command('status', [])
print('current status is:')
_ = [print(f"{key} : {value}") for key, value in status.items()]


# Set the target temperature to 35 (will also set control mode to remote)
ball_heater.send_command("set_target_temp", [45])

# Set the control mode back to standby
ball_heater.send_command("set_control_mode", ["standby"])
```

### ROS2

There is also a ROS2 Node provided for interacting with the Ball Heater Controller, [`ball_heater_node.py`](ball_heater_controller/ball_heater_controller/ball_heater_node.py).  The launch file [example_ball_heater.launch.py](ball_heater_controller/launch/example_ball_heater.launch.py), along with the config file [example_config.yaml](ball_heater_controller/config/example_config.yaml) will launch the node, as well as an example client node.

Generally, the `BallHeaterNode` must be given a `serial_port` or a `device_serial_number` to specify the controller. Then, the status information will be published to the `ball_heater_status_topic` (default `ball_heater_controller/status_topic`), and the temperature can be set by another node by publishing to the `ball_heater_set_temp_topic` (default `ball_heater_controller/set_temp_topic`)

## Sub-directories

* `ball_heater_controller` -- ROS2 Package with a Node to interact with and control the heater, as well as `ball_heater_driver.py` which contains a Python class `BallHeaterDriver` to interact with the controller.
* `ball_heater_controller_interfaces` -- ROS2 Package with custom message types associated with the above package.
* `ball_heater_controller_firmware` -- Firmware for the microcontroller at the center of the PCB.
* `ball_heater_controller_pcb` -- Design files and gerbers for the printed circuit board for the Ball Heater Controller.
* `ball_heater_controller_case` -- Vector files for laser cuttting the case for the Ball Heater Controller.
* `docs` -- Additional documentation.

## Controller Modes

The controller has several different modes, shown upper left on the LCD display, and its behavior will be different depending on what mode it is in. The mode can be changed using the `set_control_mode` command through the interface, or by pressing and holding the control knob on the front of the device for >= 1 second.

* **Standby** On startup, the controller will be in standby mode, and the heater will not be driven, but the temperature will be shown on screen.
* **Local Control**  Pressing the control from standby mode will switch into `local control` mode, and the temperature setpoint can then be adjusted by turning the control knob.  This mode is useful for running the ball heater without a computer attached, or without needing to set the temp from the computer.
* **Remote Control** In this mode, the setpoint temperature is set from the computer using the `set_target_temp` command. Any `set_target_temp` commands received by the controller will automatically switch the controller into this mode.
* **Error Mode** There are also several error modes discussed below in [Safety Cutoffs](#safety-cutoffs--troubleshooting)

## Install

Run the installation script

    ./install_dependencies.sh

## Safety Cutoffs / Troubleshooting

The ball heater firmware has several different safety cutoffs built in that will trigger different errors and shut down the heater to prevent overheating. Once one of these error modes has been activated, the heater output will shut down, and the type of error will be displayed on the LCD. Pressing and holding the control knob down will reset the device into standby mode if the error state has been resolved.

1. **High temp error:** If the heater temperature sensor reads over 60 °C, the device will go into *High temp error* and shutdown.  
2. **No Temp Sensor:**  If the temperature sensor for the ball heater is not detected because it has been unpluged or a wire has broken, the device will go into *No Temp Sensor Error* and shutdown. When the sensor is replaced, the device will return to standby mode. Note that this is only for the integrated temp sensor in the ball heater, not the optional aux temp sensor.
3. **Heater Non Responsive:** If the control loop drives the  heater at a high power for too long, eg if the temperature is not responding properly, the device will go into *Heater Non Responsive* error and shutdown.
