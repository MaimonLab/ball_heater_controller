#!/bin/bash
# install_dependencies.sh 
# ball_heater 

sudo apt-get update
sudo apt-get install -y python3-pip

pip3 install ruamel.yaml
pip3 install numpy
pip3 install event_data_logging
pip3 install pyserial 
pip3 install pysimplegui