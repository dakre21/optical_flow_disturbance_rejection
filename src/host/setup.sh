#!/bin/bash

# General Dependencies
sudo apt install libcamera-dev libopencv-dev cmake docker-compose docker.io vim

# Luxonis Camera Dependencies
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
sudo cp 80-movidius.rules /etc/udev/rules.d

sudo udevadm control --reload-rules
sudo udevadm trigger