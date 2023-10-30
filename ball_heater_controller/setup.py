import glob

from setuptools import setup

package_name = "ball_heater_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob.glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jazz Weisman",
    maintainer_email="jweisman@rockefeller.edu",
    description="ROS package to interface with the ball heater controller PCB.",
    license="LGPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ball_heater_node = ball_heater_controller.ball_heater_node:main",
            "ball_heater_client = ball_heater_controller.ball_heater_client:main",
            "list_ports = ball_heater_controller.list_ports:main",
        ],
    },
)
